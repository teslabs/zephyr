/*
 * Copyright (c) 2025 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/nvmem_provider.h>
#include <zephyr/internal/syscall_handler.h>

static inline int z_vrfy_nvmem_provider_read(const struct device *dev, unsigned int offset,
					     void *data, size_t size)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_NVMEM_PROVIDER));
	K_OOPS(K_SYSCALL_MEMORY_WRITE(data, size));
	return z_impl_nvmem_provider_read(dev, offset, data, size);
}
#include <zephyr/syscalls/nvmem_provider_read_mrsh.c>

static inline int z_vrfy_nvmem_provider_write(const struct device *dev, unsigned int offset,
					      const void *data, size_t size)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_NVMEM_PROVIDER));
	K_OOPS(K_SYSCALL_MEMORY_READ(data, size));
	return z_impl_nvmem_provider_write(dev, offset, data, size);
}
#include <zephyr/syscalls/nvmem_provider_write_mrsh.c>
