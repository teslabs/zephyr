# Copyright (c) 2023, Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=PAC5527" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
