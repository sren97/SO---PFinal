# SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0
from typing import Any

import pytest
from pytest_embedded import Dut


@pytest.mark.generic
@pytest.mark.supported_targets
@pytest.mark.parametrize(
    'config',
    [
        'default'
    ],
    indirect=True,
)
def test_esp_common(dut: Dut) -> None:
    dut.run_all_single_board_cases()


# psram noinit attr tests with psram enabled
@pytest.mark.esp32
@pytest.mark.esp32s2
@pytest.mark.esp32s3
@pytest.mark.esp32p4
@pytest.mark.esp32c5
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'psram_noinit',
    ],
    indirect=True,
)
def test_esp_attr_psram_noinit(dut: Dut) -> None:
    dut.run_all_single_board_cases()


# psram noinit memory tests with psram enabled
@pytest.mark.esp32
@pytest.mark.esp32s2
@pytest.mark.esp32s3
@pytest.mark.esp32p4
@pytest.mark.esp32c5
@pytest.mark.generic
@pytest.mark.supported_targets
@pytest.mark.parametrize(
    'config',
    [
        'psram_noinit'
    ],
    indirect=True,
)
def test_esp_attr_psram_noinit_multiple_stages(case_tester: Any) -> None:
    case_tester.run_all_multi_stage_cases()


# psram attr tests with psram enabled
@pytest.mark.esp32
@pytest.mark.esp32s2
@pytest.mark.esp32s3
@pytest.mark.esp32p4
@pytest.mark.esp32c5
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'psram',
    ],
    indirect=True,
)
def test_esp_attr_psram(dut: Dut) -> None:
    dut.run_all_single_board_cases()


# psram attr tests with xip_psram
@pytest.mark.esp32s2
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'xip_psram_esp32s2'
    ],
    indirect=True,
)
def test_esp_attr_xip_psram_esp32s2(dut: Dut) -> None:
    dut.run_all_single_board_cases()


# psram attr tests with xip_psram
@pytest.mark.esp32s3
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'xip_psram_esp32s3'
    ],
    indirect=True,
)
def test_esp_attr_xip_psram_esp32s3(dut: Dut) -> None:
    dut.run_all_single_board_cases()


# psram attr tests with xip_psram
@pytest.mark.esp32p4
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'xip_psram_esp32p4'
    ],
    indirect=True,
)
def test_esp_attr_xip_psram_esp32p4(dut: Dut) -> None:
    dut.run_all_single_board_cases()
