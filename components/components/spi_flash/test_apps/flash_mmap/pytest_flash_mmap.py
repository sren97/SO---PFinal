# SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import pytest
from pytest_embedded import Dut


@pytest.mark.supported_targets
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'release',
    ],
    indirect=True,
)
def test_flash_mmap(dut: Dut) -> None:
    dut.run_all_single_board_cases(timeout=30)


@pytest.mark.esp32s3
@pytest.mark.esp32c3
@pytest.mark.esp32c2
@pytest.mark.esp32c6
@pytest.mark.esp32h2
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'rom_impl',
    ],
    indirect=True,
)
def test_flash_mmap_rom_impl(dut: Dut) -> None:
    dut.run_all_single_board_cases(timeout=30)


@pytest.mark.supported_targets
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'xip_psram',
    ],
    indirect=True,
)
def test_flash_mmap_xip_psram(dut: Dut) -> None:
    dut.run_all_single_board_cases(timeout=30)


@pytest.mark.supported_targets
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'psram',
    ],
    indirect=True,
)
def test_flash_mmap_psram(dut: Dut) -> None:
    dut.run_all_single_board_cases(timeout=30)


@pytest.mark.supported_targets
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'xip_psram_with_rom_impl',
    ],
    indirect=True,
)
def test_flash_mmap_xip_psram_rom_impl(dut: Dut) -> None:
    dut.run_all_single_board_cases(timeout=30)
