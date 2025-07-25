/**
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef H_BLE_SVC_DIS_
#define H_BLE_SVC_DIS_

/**
 * Example:
 *
 *    char firmware_revision[20] = '?.?.?';
 *    struct image_version iv;
 *    if (!imgr_my_version(&iv)) {
 *	snprintf(firmware_revision, sizeof(firmware_revision),
 *		 "%u.%u.%u", iv.iv_major, iv.iv_minor, iv.iv_revision);
 *    }
 *    ble_svc_dis_manufacturer_name_set("MyNewt");
 *    ble_svc_dis_firmware_revision_set(firmware_revision);
 *
 */

#define BLE_SVC_DIS_UUID16                          0x180A
#define BLE_SVC_DIS_CHR_UUID16_SYSTEM_ID            0x2A23
#define BLE_SVC_DIS_CHR_UUID16_MODEL_NUMBER         0x2A24
#define BLE_SVC_DIS_CHR_UUID16_SERIAL_NUMBER        0x2A25
#define BLE_SVC_DIS_CHR_UUID16_FIRMWARE_REVISION    0x2A26
#define BLE_SVC_DIS_CHR_UUID16_HARDWARE_REVISION    0x2A27
#define BLE_SVC_DIS_CHR_UUID16_SOFTWARE_REVISION    0x2A28
#define BLE_SVC_DIS_CHR_UUID16_MANUFACTURER_NAME    0x2A29
#define BLE_SVC_DIS_CHR_UUID16_IEEE_REG_CERT_LIST   0x2A2A
#define BLE_SVC_DIS_CHR_UUID16_PNP_ID               0x2A50
#define BLE_SVC_DIS_CHR_UUID16_UDI                  0x2BFF

/* One PTS Case requires DIS to be included. 
 * The UUID below is randomly chosen.
 */
static const ble_uuid128_t ble_svc_dis_include_uuid =
    BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0xc8, 0x12,
                     0x99, 0x99, 0x43, 0x95, 0x12, 0x2f, 0x46, 0xFF);

/**
 * Structure holding data for the main characteristics
 */
struct ble_svc_dis_data {
    /**
     * Model number.
     * Represent the model number that is assigned by the device vendor.
     */
    const char *model_number;
    /**
     * Serial number.
     * Represent the serial number for a particular instance of the device.
     */
    const char *serial_number;
    /**
     * Firmware revision.
     * Represent the firmware revision for the firmware within the device.
     */
    const char *firmware_revision;
    /**
     * Hardware revision.
     * Represent the hardware revision for the hardware within the device.
     */
    const char *hardware_revision;
    /**
     * Software revision.
     * Represent the software revision for the software within the device.
     */
    const char *software_revision;
    /**
     * Manufacturer name.
     * Represent the name of the manufacturer of the device.
     */
    const char *manufacturer_name;
    /**
     * System ID.
     * Represent the System Id of the device.
     * Length: 8 Octets
     */
    const char *system_id;

    /**
     * PNP ID.
     * Represent the PNP Id of the device.
     * Length: 7 Octets
     */
    const char *pnp_id;

    /**
     * IEEE 11073-20601 Regulatory Certification Data List
     */
    const char *ieee;

    /**
     * UDI for Medical Devices
     */
    const char *udi;
};

/**
 * Variable holding data for the main characteristics.
 */
extern struct ble_svc_dis_data ble_svc_dis_data;

/**
 * Service initialisation.
 * Automatically called during package initialisation.
 */
void ble_svc_dis_init(void);

/**
 * Service initialisation as an included service.
 */
void ble_svc_dis_included_init(void);

const char *ble_svc_dis_model_number(void);
int ble_svc_dis_model_number_set(const char *value);
const char *ble_svc_dis_serial_number(void);
int ble_svc_dis_serial_number_set(const char *value);
const char *ble_svc_dis_firmware_revision(void);
int ble_svc_dis_firmware_revision_set(const char *value);
const char *ble_svc_dis_hardware_revision(void);
int ble_svc_dis_hardware_revision_set(const char *value);
const char *ble_svc_dis_software_revision(void);
int ble_svc_dis_software_revision_set(const char *value);
const char *ble_svc_dis_manufacturer_name(void);
int ble_svc_dis_manufacturer_name_set(const char *value);
const char *ble_svc_dis_system_id(void);
int ble_svc_dis_system_id_set(const char *value);
int ble_svc_dis_pnp_id_set(const char *value);
const char *ble_svc_dis_pnp_id(void);

#endif
