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
static uint32_t last_x[10], last_y[10];
static unsigned active_mask;
static unsigned long posted_down, posted_move, posted_up;
static unsigned max_x_inchip = 8192, max_y_inchip = 16384;
static bool tcm_command(uint8_t command, const uint8_t *data, unsigned length);

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
	pause_us(2);
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

/* The touch report starts with the gesture block (1 + 6 + 24 bytes);
 * then five bytes per active object: index, class, x and y. */
/* GET_APPLICATION_INFO (0x20) carries the controller limits at
 * offset 32 (max_x) and 34 (max_y). */
static void
fetch_limits(void)
{
	uint8_t chunk[128];
	(void)tcm_command(0x20, NULL, 0);
	for (unsigned i = 0; i < 40; i++) {
		if (tcm_read(chunk, sizeof(chunk)) && chunk[0] == TCM_MARKER) {
			unsigned len = (unsigned)chunk[2] | ((unsigned)chunk[3] << 8);
			if (len >= 36) {
				const uint8_t *p = chunk + 4;
				max_x_inchip = (unsigned)p[32] | ((unsigned)p[33] << 8);
				max_y_inchip = (unsigned)p[34] | ((unsigned)p[35] << 8);
				max_x_inchip += 1; max_y_inchip += 1;
				redwood_tl("s3908: limits max_x %u max_y %u\n", max_x_inchip, max_y_inchip);
				return;
			}
		}
		pause_ms(10);
	}
	redwood_tl("s3908: limits unavailable, keeping %u x %u\n", max_x_inchip, max_y_inchip);
}

static void
parse_touch(const uint8_t *data, unsigned length)
{
	static const unsigned object_base = 31;
	unsigned mask = 0;
	if (length < object_base + 5) {
		return;
	}
	unsigned objects = (length - object_base) / 5;
	for (unsigned i = 0; i < objects; i++) {
		const uint8_t *o = data + object_base + 5 * i;
		if (reports <= 20) {
			redwood_tl("s3908: obj %u/%u bytes %02x %02x %02x %02x %02x len %u\n",
			    i, objects, o[0], o[1], o[2], o[3], o[4], length);
		}
		unsigned index = o[0] & 0x0f;
		unsigned cls = o[0] >> 4;
		uint32_t x = (uint32_t)o[1] | ((uint32_t)o[2] << 8);
		uint32_t y = (uint32_t)o[3] | ((uint32_t)o[4] << 8);
		uint32_t sx = x * 65535u / max_x_inchip;
		uint32_t sy = y * 65535u / max_y_inchip;
		if (sx > 65535) sx = 65535;
		if (sy > 65535) sy = 65535;
		if (index >= 10 || cls == 0) {
			continue;
		}
		mask |= 1u << index;
		if (active_mask & (1u << index)) {
			redwood_touch_post(REDWOOD_TOUCH_MOVE, index, sx, sy);
			posted_move++;
		} else {
			redwood_touch_post(REDWOOD_TOUCH_DOWN, index, sx, sy);
			posted_down++;
		}
		last_x[index] = x; last_y[index] = y;
	}
	for (unsigned index = 0; index < 10; index++) {
		if ((active_mask & (1u << index)) && !(mask & (1u << index))) {
			redwood_touch_post(REDWOOD_TOUCH_UP, index, last_x[index], last_y[index]);
			posted_up++;
		}
	}
	active_mask = mask;
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
			parse_touch(data + offset + 4, size);
		}
		offset += 4 + size + 1;
	}
}

static bool
tcm_command(uint8_t command, const uint8_t *data, unsigned length)
{
	uint8_t out[160];
	if (length > sizeof(out) - 3) {
		return false;
	}
	out[0] = command;
	out[1] = (uint8_t)(length & 0xff);
	out[2] = (uint8_t)(length >> 8);
	for (unsigned i = 0; i < length; i++) {
		out[3 + i] = data[i];
	}
	return tcm_write(out, 3 + length);
}

/* Send a command and wait for the controller's status answer.  Log 
every message seen so a refused command is visible. */
static unsigned
tcm_request(const char *name, uint8_t command, const uint8_t *data, unsigned length)
{
	if (!tcm_command(command, data, length)) {
		redwood_tl("s3908: %s write failed\n", name);
		return 0xffff;
	}
	for (unsigned i = 0; i < 40; i++) {
		uint8_t chunk[64];
		if (tcm_read(chunk, sizeof(chunk)) && chunk[0] == TCM_MARKER) {
			unsigned code = chunk[1];
			if (code != 0x1d) {
				redwood_tl("s3908: %s answered %02x\n", name, code);
				return code;
			}
		}
		pause_ms(2);
	}
	redwood_tl("s3908: %s no answer\n", name);
	return 0xfffe;
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
	/* Bring the controller up in the same order the vendor driver does. */
	(void)tcm_request("exit deep sleep", 0x2d, NULL, 0);
	pause_ms(50);
	static uint8_t report_config[128];
	memset(report_config, 0, sizeof(report_config));
	static const uint8_t vendor_config[] = {
		0x10, 0x08, 0xc6, 0x30, 0xc7, 0xc0,
		0x01, 0x06, 0x04, 0x07, 0x04, 0x08, 0x10, 0x09, 0x10, 0x03, 0x00
	};
	for (unsigned i = 0; i < sizeof(vendor_config); i++) {
		report_config[i] = vendor_config[i];
	}
	(void)tcm_request("report config", 0x26, report_config, sizeof(report_config));
	pause_ms(80);
	uint8_t enable_touch = 0x11;
	(void)tcm_request("enable touch", 0x05, &enable_touch, 1);
	pause_ms(50);

	for (unsigned round = 0; round < 1000; round++) {
		for (unsigned i = 0; i < 90000; i++) {
			if (!level(IRQ)) {
				if (tcm_read(chunk, 96)) {
					if (chunk[0] == TCM_MARKER) {
						parse_chunk(chunk, 96);
					}
				} else {
					errors++;
				}
				continue;
			}
			pause_us(300);
		}
		if (reports == 0) {
			redwood_tl("s3908: round %u done without reports\n", round + 1);
		}
		if (reports > 0) {
			/* The controller is scanning; never reset it again. */
			continue;
		}
		/* Power-cycle the controller now that the panel is on. */
		output(RESET, false, 3, 8);
		pause_ms(20);
		output(RESET, true, 3, 8);
		pause_ms(150);
		for (unsigned k = 0; k < 15; k++) {
			if (tcm_read(chunk, 64) && chunk[0] == TCM_MARKER) {
				redwood_tl("s3908: after reset code %02x len %u\n", chunk[1], (unsigned)chunk[2] | ((unsigned)chunk[3] << 8));
				break;
			}
			pause_ms(20);
		}
		static const uint8_t normal_mode[] = { 0x09, 0x00, 0x00 };
		(void)tcm_request("exit deep sleep", 0x2d, NULL, 0);
		pause_ms(50);
		(void)tcm_request("gesture mode off", 0x24, normal_mode, sizeof(normal_mode));
		(void)tcm_request("report config", 0x26, report_config, sizeof(report_config));
		pause_ms(80);
		(void)tcm_request("get config", 0x25, NULL, 0);
		pause_ms(30);
		fetch_limits();
		uint8_t en2 = 0x11;
		(void)tcm_request("enable touch", 0x05, &en2, 1);
		pause_ms(50);
	}
	redwood_tl("s3908: diagnostic stopped; messages %u reports %u errors %u down %lu move %lu up %lu\n", msgs, reports, errors, posted_down, posted_move, posted_up);
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
