/*
 * OnePlus 9RT (martini), Synaptics S3908 on I2C 0x4B (i2c@990000, GPIO20/21),
 * reset GPIO22, IRQ GPIO23.  TCM framing: 4-byte header (0xA5, code, len lo/hi),
 * payload, 0x5A padding.  A message arrives inside one I2C read transaction.
 * touchscreen=3 runs the diagnostic and logs what the controller answers.
 */
#include <sys/lemonade_radio.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/redwood_input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern void redwood_tl(const char *format, ...) __printflike(1, 2);

#define SDA 20
#define SCL 21
#define RESET 22
#define IRQ 23
#define TCM_ADDRESS 0x4b
#define TCM_MARKER 0xa5
#define TCM_PADDING 0x5a
#define TCM_CMD_ENABLE_REPORT 0x05
#define TCM_REPORT_TOUCH 0x11

static vm_offset_t pins;
static uint32_t mode;
static unsigned msgs, reports, errors;

static uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
	__asm__ volatile("dsb sy" ::: "memory");
}

static void
pause_us(unsigned us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static void
pause_ms(unsigned ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

#define PIN_CFG(n) (pins + 0x1000 * (n))
#define PIN_IO(n)  (pins + 0x1000 * (n) + 4)

static bool
level(unsigned pin)
{
	return (rd(PIN_IO(pin)) & 1) != 0;
}

static void
output(unsigned pin, bool high, unsigned pull, unsigned ma)
{
	wr(PIN_IO(pin), high ? 2 : 0);
	wr(PIN_CFG(pin), pull | (ma / 2 - 1) << 6 | 1u << 9);
}

static void
line(unsigned pin, bool high)
{
	if (!high) {
		wr(PIN_IO(pin), 0);
	}
	wr(PIN_CFG(pin), 3 | (high ? 0 : 1u << 9));
}

static void
i2c_delay(void)
{
	pause_us(5);
}

static bool
clock_high(void)
{
	line(SCL, true);
	for (unsigned i = 0; i < 4000; i++) {
		if (level(SCL)) {
			return true;
		}
		pause_us(5);
	}
	return false;
}

static bool
i2c_start(void)
{
	line(SDA, true);
	if (!clock_high()) {
		return false;
	}
	i2c_delay();
	if (!level(SDA)) {
		return false;
	}
	line(SDA, false);
	i2c_delay();
	line(SCL, false);
	i2c_delay();
	return true;
}

static void
i2c_stop(void)
{
	line(SDA, false);
	i2c_delay();
	line(SCL, true);
	i2c_delay();
	line(SDA, true);
	i2c_delay();
}

static bool
i2c_send(uint8_t byte)
{
	for (int bit = 7; bit >= 0; bit--) {
		line(SDA, (byte >> bit) & 1);
		i2c_delay();
		if (!clock_high()) {
			return false;
		}
		i2c_delay();
		line(SCL, false);
		i2c_delay();
	}
	line(SDA, true);
	i2c_delay();
	if (!clock_high()) {
		return false;
	}
	bool ack = !level(SDA);
	i2c_delay();
	line(SCL, false);
	i2c_delay();
	return ack;
}

static bool
i2c_receive(uint8_t *byte, bool ack)
{
	uint8_t value = 0;
	line(SDA, true);
	for (unsigned bit = 0; bit < 8; bit++) {
		if (!clock_high()) {
			return false;
		}
		value = (uint8_t)((value << 1) | (level(SDA) ? 1 : 0));
		i2c_delay();
		line(SCL, false);
		i2c_delay();
	}
	line(SDA, ack ? false : true);
	i2c_delay();
	if (!clock_high()) {
		return false;
	}
	i2c_delay();
	line(SCL, false);
	i2c_delay();
	line(SDA, true);
	*byte = value;
	return true;
}

static bool
tcm_read(uint8_t *buffer, unsigned length)
{
	if (!i2c_start()) {
		return false;
	}
	if (!i2c_send((uint8_t)(TCM_ADDRESS << 1 | 1))) {
		i2c_stop();
		return false;
	}
	for (unsigned i = 0; i < length; i++) {
		if (!i2c_receive(&buffer[i], i + 1 < length)) {
			i2c_stop();
			return false;
		}
	}
	i2c_stop();
	return true;
}

static bool
tcm_write(const uint8_t *buffer, unsigned length)
{
	if (!i2c_start()) {
		return false;
	}
	if (!i2c_send((uint8_t)(TCM_ADDRESS << 1))) {
		i2c_stop();
		return false;
	}
	for (unsigned i = 0; i < length; i++) {
		if (!i2c_send(buffer[i])) {
			i2c_stop();
			return false;
		}
	}
	i2c_stop();
	return true;
}

static void
dump(const char *tag, const uint8_t *data, unsigned length)
{
	char text[3 * 24 + 1];
	unsigned shown = length < 24 ? length : 24;
	unsigned at = 0;
	for (unsigned i = 0; i < shown; i++) {
		at += snprintf(text + at, sizeof(text) - at, "%02x ", data[i]);
	}
	text[at] = 0;
	redwood_tl("s3908: %s (%u) %s\n", tag, length, text);
}

static void
parse_chunk(const uint8_t *data, unsigned length)
{
	unsigned offset = 0;
	while (offset + 5 <= length) {
		while (offset < length && data[offset] == TCM_PADDING) {
			offset++;
		}
		if (offset + 5 > length) {
			break;
		}
		if (data[offset] != TCM_MARKER) {
			redwood_tl("s3908: chunk sync lost at %u (byte %02x)\n", offset, data[offset]);
			break;
		}
		unsigned code = data[offset + 1];
		unsigned size = (unsigned)data[offset + 2] | ((unsigned)data[offset + 3] << 8);
		if (size > 2048 || offset + 4 + size + 1 > length) {
			redwood_tl("s3908: chunk truncated code %02x size %u\n", code, size);
			break;
		}
		msgs++;
		redwood_tl("s3908: message code %02x length %u\n", code, size);
		dump(code == TCM_REPORT_TOUCH ? "report" : "payload", data + offset + 4, size);
		if (code == TCM_REPORT_TOUCH) {
			reports++;
		}
		offset += 4 + size + 1;
	}
}

static bool
tcm_command(uint8_t command, const uint8_t *data, unsigned length)
{
	uint8_t out[160];
	if (length > sizeof(out) - 5) {
		return false;
	}
	out[0] = TCM_MARKER;
	out[1] = command;
	out[2] = (uint8_t)(length & 0xff);
	out[3] = (uint8_t)(length >> 8);
	for (unsigned i = 0; i < length; i++) {
		out[4 + i] = data[i];
	}
	out[4 + length] = TCM_PADDING;
	return tcm_write(out, 5 + length);
}

static void
touch_thread(__unused void *arg, __unused wait_result_t result)
{
	static uint8_t chunk[2048];
	uint8_t out[8];

	pause_ms(3000); /* let the display driver bring the panel up first */
	pins = ml_io_map(0x0f100000, 0x80000);
	redwood_tl("s3908: martini Synaptics S3908 diagnostic, mode %u\n", mode);
	redwood_tl("s3908: pins SDA=%u SCL=%u IRQ=%u; SDA=%u SCL=%u IRQ=%u\n",
	    SDA, SCL, IRQ, level(SDA), level(SCL), level(IRQ));

	bool s3 = lemonade_radio_supply_on("ldoc3", 3000);
	bool s8 = lemonade_radio_supply_on("ldoc8", 1800);
	redwood_tl("s3908: supplies ldoc3=%d ldoc8=%d\n", s3, s8);

	output(RESET, false, 3, 8);
	pause_ms(20);
	line(SDA, true);
	line(SCL, true);
	output(RESET, true, 3, 8);
	pause_ms(150);
	wr(PIN_CFG(IRQ), 0); /* plain GPIO input, no pull */
	redwood_tl("s3908: reset released; SDA=%u SCL=%u IRQ=%u\n",
	    level(SDA), level(SCL), level(IRQ));

	if (tcm_read(chunk, sizeof(chunk))) {
		redwood_tl("s3908: first read ok, marker %02x code %02x len %u\n",
		    chunk[0], chunk[1], (unsigned)chunk[2] | ((unsigned)chunk[3] << 8));
		parse_chunk(chunk, sizeof(chunk));
	} else {
		redwood_tl("s3908: first read failed\n");
		errors++;
	}

	/* The controller answers NOT_IMPLEMENTED to ENABLE_REPORT; the touch report
	 * format is what turns reporting on.  Ask for the current configuration,
	 * then set a simple one: per active object, index/class/x/y/width/height.
	 */
	static uint8_t report_config[128];
	memset(report_config, 0, sizeof(report_config));
	report_config[0] = 1;              /* foreach active object */
	report_config[1] = 6; report_config[2] = 4;   /* object index, 4 bits */
	report_config[3] = 7; report_config[4] = 4;   /* classification, 4 bits */
	report_config[5] = 8; report_config[6] = 16;  /* x, 16 bits */
	report_config[7] = 9; report_config[8] = 16;  /* y, 16 bits */
	report_config[9] = 11; report_config[10] = 12; /* width, 12 bits */
	report_config[11] = 12; report_config[12] = 12; /* height, 12 bits */
	report_config[13] = 3;             /* foreach end */
	report_config[14] = 0;             /* touch end */
	if (tcm_command(0x26, report_config, sizeof(report_config))) {
		redwood_tl("s3908: touch report config sent (128 bytes)\n");
	} else {
		redwood_tl("s3908: touch report config failed\n");
	}
	pause_ms(80);

	for (unsigned i = 0; i < 3000; i++) {
		bool low = !level(IRQ);
		if (low || (i % 10) == 0) {
			if (tcm_read(chunk, 128)) {
				if (chunk[0] == TCM_MARKER) {
					if (low) {
						redwood_tl("s3908: irq low, bytes %02x %02x %02x %02x\n", chunk[0], chunk[1], chunk[2], chunk[3]);
					}
					parse_chunk(chunk, 128);
				}
			} else {
				errors++;
			}
		}
		pause_ms(20);
	}
	redwood_tl("s3908: diagnostic stopped; messages %u reports %u errors %u\n",
	    msgs, reports, errors);
	thread_terminate(current_thread());
}

void
martini_synaptics_init(void)
{
	thread_t thread;
	if (!PE_parse_boot_argn("touchscreen", &mode, sizeof(mode)) || mode != 3) {
		return;
	}
	redwood_tl("s3908: bootarg touchscreen=3; starting S3908 diagnostic\n");
	if (kernel_thread_start(touch_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
