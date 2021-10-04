# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_CANOPEN_STACK)
  set(CANOPEN_STACK_DIR ${ZEPHYR_CURRENT_MODULE_DIR})

  zephyr_library()

  zephyr_include_directories(
    ${CANOPEN_STACK_DIR}/canopen/include
    .
  )

  zephyr_library_sources(
    ${CANOPEN_STACK_DIR}/canopen/source/co_core.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_dict.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_emcy.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_if_can.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_if_nvm.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_if_timer.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_if.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_lss.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_nmt.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_obj.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_para.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_pdo.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_sdo_srv.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_sync.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_tmr.c
    ${CANOPEN_STACK_DIR}/canopen/source/co_ver.c
    co_can_zephyr.c
    co_nvm_zephyr.c
    co_timer_zephyr.c
  )

endif()
