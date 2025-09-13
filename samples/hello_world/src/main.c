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
	uint8_t buf[128];

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

	printf("read:\n");
	for (int i = 0; i < sizeof(buf); i++) {
		printf("%02x ", buf[i]);
		if ((i + 1) % 16 == 0) {
			printf("\n");
		}
	}

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

	printf("read:\n");
	for (int i = 0; i < sizeof(buf); i++) {
		printf("%02x ", buf[i]);
		if ((i + 1) % 16 == 0) {
			printf("\n");
		}
	}

	printf("write 0x01, 0x02, ... to %p\n", (void *)ADDR);
	for (int i = 0; i < sizeof(buf); i++) {
		buf[i] = i;
	}
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

	printf("read:\n");
	for (int i = 0; i < sizeof(buf); i++) {
		printf("%02x ", buf[i]);
		if ((i + 1) % 16 == 0) {
			printf("\n");
		}
	}

	return 0;
}
