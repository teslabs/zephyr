# Copyright (c) 2025 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/nvmem_provider.h)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_USERSPACE retained_mem_handlers.c)
