/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/drivers/flash.h>

#define ADDR 0xFC0000

const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));

int main(void)
{
	int ret;
	uint8_t id[3];
	uint8_t buf[16];

	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	if (!device_is_ready(dev)) {
		printf("Device not ready.\n");
		return 0;
	}

	flash_read_jedec_id(dev, id);
	printf("JEDEC ID: %02X:%02X:%02X\n", id[0], id[1], id[2]);

	printf("reading from %p\n", (void *)ADDR);
	ret = flash_read(dev, ADDR, buf, sizeof(buf));
	if (ret) {
		printf("flash_read failed: %d\n", ret);
		return 0;
	}

	printf("read: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
	       buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
	       buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);

	printf("erasing sector at %p\n", (void *)ADDR);
	ret = flash_erase(dev, ADDR, 4096);
	if (ret) {
		printf("flash_erase failed: %d\n", ret);
		return 0;
	}

	printf("reading from %p\n", (void *)ADDR);
	ret = flash_read(dev, ADDR, buf, sizeof(buf));
	if (ret) {
		printf("flash_read failed: %d\n", ret);
		return 0;
	}

	printf("read: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
	       buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
	       buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);

	printf("write 0xaa, 0xbb, ... to %p\n", (void *)ADDR);
	buf[0] = 0xAA;
	buf[1] = 0xBB;
	buf[2] = 0xCC;
	buf[3] = 0xDD;
	buf[4] = 0xEE;
	buf[5] = 0xFF;
	buf[6] = 0x11;
	buf[7] = 0x22;
	buf[8] = 0x33;
	buf[9] = 0x44;
	buf[10] = 0x55;
	buf[11] = 0x66;
	buf[12] = 0x77;
	buf[13] = 0x88;
	buf[14] = 0x99;
	buf[15] = 0x00;
	ret = flash_write(dev, ADDR, buf, sizeof(buf));
	if (ret) {
		printf("flash_write failed: %d\n", ret);
		return 0;
	}

	printf("reading from %p\n", (void *)ADDR);
	ret = flash_read(dev, ADDR, buf, sizeof(buf));
	if (ret) {
		printf("flash_read failed: %d\n", ret);
		return 0;
	}

	printf("read: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
	       buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
	       buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);

	return 0;
}
