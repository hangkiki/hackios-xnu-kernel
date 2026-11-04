/*
 * OnePlus 9RT (martini), Synaptics S3908 on I2C 0x4B (i2c@990000, GPIO20/21),
 * reset GPIO22, IRQ GPIO23.  TCM framing: 4-byte header (0xA5, code, len lo/hi),
 * payload, 0x5A padding.  Diagnostic build: touchscreen=3 logs raw messages.
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
#define TCM_STATUS_OK 0x01
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

/* Open drain: high releases the line, low drives it. */
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
	char text[3 * 32 + 1];
	unsigned shown = length < 32 ? length : 32;
	unsigned at = 0;
	for (unsigned i = 0; i < shown; i++) {
		at += snprintf(text + at, sizeof(text) - at, "%02x ", data[i]);
	}
	text[at] = 0;
	redwood_tl("s3908: %s (%u) %s\n", tag, length, text);
}

/* Read one TCM message.  Returns false when the header is not a TCM marker. */
static bool
tcm_message(uint8_t *payload, unsigned payload_size, unsigned *code, unsigned *length)
{
	uint8_t header[4];
	if (!tcm_read(header, sizeof(header))) {
		errors++;
		return false;
	}
	if (header[0] != TCM_MARKER) {
		dump("raw header", header, sizeof(header));
		errors++;
		return false;
	}
	*code = header[1];
	*length = (unsigned)header[2] | ((unsigned)header[3] << 8);
	unsigned want = *length;
	if (want > payload_size) {
		want = payload_size;
	}
	if (want && !tcm_read(payload, want)) {
		errors++;
		return false;
	}
	/* The message ends with a 0x5A padding byte; consume it when it fits. */
	if (*length < payload_size && *length) {
		uint8_t padding;
		if (tcm_read(&padding, 1) && padding != TCM_PADDING) {
			redwood_tl("s3908: padding %02x (expected 5a)\n", padding);
		}
	}
	msgs++;
	return true;
}

static bool
tcm_command(uint8_t command, const uint8_t *data, unsigned length)
{
	uint8_t out[8];
	if (length > 3) {
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
	uint8_t payload[256];
	unsigned code = 0, length = 0;

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
	redwood_tl("s3908: reset released; SDA=%u SCL=%u IRQ=%u\n",
	    level(SDA), level(SCL), level(IRQ));

	for (unsigned i = 0; i < 6; i++) {
		if (tcm_message(payload, sizeof(payload), &code, &length)) {
			redwood_tl("s3908: message code %02x length %u\n", code, length);
			dump("payload", payload, length);
		} else {
			redwood_tl("s3908: no message (%u errors)\n", errors);
			pause_ms(20);
		}
	}

	uint8_t enable = TCM_REPORT_TOUCH;
	if (tcm_command(TCM_CMD_ENABLE_REPORT, &enable, 1)) {
		redwood_tl("s3908: enable report 0x11 sent\n");
	} else {
		redwood_tl("s3908: enable report failed\n");
	}

	for (unsigned i = 0; i < 4000; i++) {
		if (!level(IRQ)) {
			if (tcm_message(payload, sizeof(payload), &code, &length)) {
				if (code == TCM_REPORT_TOUCH) {
					reports++;
					if (reports <= 20) {
						redwood_tl("s3908: touch report %u length %u\n", reports, length);
						dump("report", payload, length);
					}
				} else {
					redwood_tl("s3908: message code %02x length %u\n", code, length);
					dump("payload", payload, length);
				}
			}
		}
		pause_ms(2);
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
