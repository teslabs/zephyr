/*
 * Copyright (c) 2025 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @ingroup nvmem_provider_interface
 * @brief Main header file for the NVMEM provider driver API.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_NVMEM_PROVIDER_H_
#define ZEPHYR_INCLUDE_DRIVERS_NVMEM_PROVIDER_H_

/**
 * @brief Interfaces for Non-Voltatile Memory (NVMEM) providers.
 * @defgroup nvmem_provider_interface NVMEM provider
 * @since 4.3
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */

#include <stddef.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

/**
 * @brief API operation for reading from the NVMEM.
 * @see nvmem_provider_read()
 */
typedef int (*nvmem_provider_api_read)(const struct device *dev, unsigned int offset, void *data,
				       size_t len);

/**
 * @brief API operation for writing to the NVMEM.
 * @see nvmem_provider_write()
 */
typedef int (*nvmem_provider_api_write)(const struct device *dev, unsigned int offset,
					const void *data, size_t len);

__subsystem struct nvmem_provider_driver_api {
	nvmem_provider_api_read read;
	nvmem_provider_api_write write;
};

/** @endcond */

/**
 * @brief Read data from a NVMEM provider.
 *
 * @param dev NVMEM provider device instance.
 * @param offset Address offset to read from.
 * @param data Buffer to store read data.
 * @param len Number of bytes to read.
 *
 * @retval 0 If read succeeded.
 * @retval -EINVAL If the read would exceed the device bounds.
 * @retval -errno In case of other errors.
 */
__syscall int nvmem_provider_read(const struct device *dev, unsigned int offset, void *data,
				  size_t len);

static inline int z_impl_nvmem_provider_read(const struct device *dev, unsigned int offset,
					     void *data, size_t len)
{
	const struct nvmem_provider_driver_api *api =
		(const struct nvmem_provider_driver_api *)dev->api;

	return api->read(dev, offset, data, len);
}

/**
 *  @brief Write data to a NVMEM provider.
 *
 * @param dev NVMEM provider device instance.
 * @param offset Address offset to write data to.
 * @param data Buffer with data to write.
 * @param len Number of bytes to write.
 *
 * @retval 0 If write succeeded.
 * @retval -EINVAL If the write offset/length is invalid (e.g. not aligned, out of bounds).
 * @retval -errno In case of other errors.
 */
__syscall int nvmem_provider_write(const struct device *dev, unsigned int offset, const void *data,
				   size_t len);

static inline int z_impl_nvmem_provider_write(const struct device *dev, unsigned int offset,
					      const void *data, size_t len)
{
	const struct nvmem_provider_driver_api *api =
		(const struct nvmem_provider_driver_api *)dev->api;

	return api->write(dev, offset, data, len);
}

#ifdef __cplusplus
}
#endif

/** @} */

#include <zephyr/syscalls/nvmem_provider.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_NVMEM_PROVIDER_H_ */
