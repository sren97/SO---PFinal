/*
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

#ifndef H_BLE_ATT_
#define H_BLE_ATT_

/**
 * @brief Bluetooth Attribute Protocol (ATT)
 * @defgroup bt_att Bluetooth Attribute Protocol (ATT)
 * @ingroup bt_host
 * @{
 */

#include "os/queue.h"
#ifdef __cplusplus
extern "C" {
#endif

/** Chained memory buffer. */
struct os_mbuf;

/**
 * @defgroup ble_att_uuids Attribute Protocol (ATT) UUIDs
 * @{
 */
/** UUID for a primary service. */
#define BLE_ATT_UUID_PRIMARY_SERVICE        0x2800

/** UUID for a secondary service. */
#define BLE_ATT_UUID_SECONDARY_SERVICE      0x2801

/** UUID for an include definition. */
#define BLE_ATT_UUID_INCLUDE                0x2802

/** UUID for a characteristic. */
#define BLE_ATT_UUID_CHARACTERISTIC         0x2803

/** @} */

/**
 * @defgroup ble_att_err_codes Attribute Protocol (ATT) Error Codes
 * @{
 */

/** The attribute handle given was not valid on this server */
#define BLE_ATT_ERR_INVALID_HANDLE          0x01

/** The attribute cannot be read. */
#define BLE_ATT_ERR_READ_NOT_PERMITTED      0x02

/** The attribute cannot be written. */
#define BLE_ATT_ERR_WRITE_NOT_PERMITTED     0x03

/** The attribute PDU was invalid. */
#define BLE_ATT_ERR_INVALID_PDU             0x04

/** The attribute requires authentication before it can be read or written. */
#define BLE_ATT_ERR_INSUFFICIENT_AUTHEN     0x05

/** ATT Server does not support the request received from the client. */
#define BLE_ATT_ERR_REQ_NOT_SUPPORTED       0x06

/** Offset specified was past the end of the attribute. */
#define BLE_ATT_ERR_INVALID_OFFSET          0x07

/** The attribute requires authorization before it can be read or written. */
#define BLE_ATT_ERR_INSUFFICIENT_AUTHOR     0x08

/** Too many prepare writes have been queued. */
#define BLE_ATT_ERR_PREPARE_QUEUE_FULL      0x09

/** No attribute found within the given attribute handle range. */
#define BLE_ATT_ERR_ATTR_NOT_FOUND          0x0a

/** The attribute cannot be read using the ATT_READ_BLOB_REQ PDU. */
#define BLE_ATT_ERR_ATTR_NOT_LONG           0x0b

/** The Encryption Key Size used for encrypting this link is too short. */
#define BLE_ATT_ERR_INSUFFICIENT_KEY_SZ     0x0c

/** The attribute value length is invalid for the operation. */
#define BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN  0x0d

/** The attribute request that was requested has encountered an error that was unlikely,
 *  and therefore could not be completed as requested. */
#define BLE_ATT_ERR_UNLIKELY                0x0e

/** The attribute requires encryption before it can be read or written */
#define BLE_ATT_ERR_INSUFFICIENT_ENC        0x0f

/** The attribute type is not a supported grouping attribute as defined by a higher layer specification. */
#define BLE_ATT_ERR_UNSUPPORTED_GROUP       0x10

/**Insufficient Resources to complete the request. */
#define BLE_ATT_ERR_INSUFFICIENT_RES        0x11
#define BLE_ATT_ERR_DB_OUT_OF_SYNC          0x12
#define BLE_ATT_ERR_VALUE_NOT_ALLOWED       0x13

/** @} */

/**
 * @defgroup ble_att_op_codes Attribute Protocol (ATT) Operation Codes
 * @{
 */

/** Error Response. */

#define BLE_ATT_OP_ERROR_RSP                0x01

/** MTU Request. */
#define BLE_ATT_OP_MTU_REQ                  0x02

/** MTU Response. */
#define BLE_ATT_OP_MTU_RSP                  0x03

/** Find Information Request. */
#define BLE_ATT_OP_FIND_INFO_REQ            0x04

/** Find Information Response. */
#define BLE_ATT_OP_FIND_INFO_RSP            0x05

/** Find By Type Value Request. */
#define BLE_ATT_OP_FIND_TYPE_VALUE_REQ      0x06

/** Find By Type Value Response. */
#define BLE_ATT_OP_FIND_TYPE_VALUE_RSP      0x07

/** Read By Type Request. */
#define BLE_ATT_OP_READ_TYPE_REQ            0x08

/** Read By Type Response. */
#define BLE_ATT_OP_READ_TYPE_RSP            0x09

/** Read Request. */
#define BLE_ATT_OP_READ_REQ                 0x0a

/** Read Response. */
#define BLE_ATT_OP_READ_RSP                 0x0b

/** Read Blob Request. */
#define BLE_ATT_OP_READ_BLOB_REQ            0x0c

/** Read Blob Response. */
#define BLE_ATT_OP_READ_BLOB_RSP            0x0d

/** Read Multiple Request. */
#define BLE_ATT_OP_READ_MULT_REQ            0x0e

/** Read Multiple Response. */
#define BLE_ATT_OP_READ_MULT_RSP            0x0f

/** Read By Group Type Request. */
#define BLE_ATT_OP_READ_GROUP_TYPE_REQ      0x10

/** Read By Group Type Response. */
#define BLE_ATT_OP_READ_GROUP_TYPE_RSP      0x11

/** Write Request. */
#define BLE_ATT_OP_WRITE_REQ                0x12

/** Write Response. */
#define BLE_ATT_OP_WRITE_RSP                0x13

/** Prepare Write Request. */
#define BLE_ATT_OP_PREP_WRITE_REQ           0x16

/** Prepare Write Response. */
#define BLE_ATT_OP_PREP_WRITE_RSP           0x17

/** Execute Write Request. */
#define BLE_ATT_OP_EXEC_WRITE_REQ           0x18

/** Execute Write Response. */
#define BLE_ATT_OP_EXEC_WRITE_RSP           0x19

/** Read Multiple Variable Lenght Request */
#define BLE_ATT_OP_READ_MULT_VAR_REQ        0x20

/** Read Multiple Variable Lenght Response */
#define BLE_ATT_OP_READ_MULT_VAR_RSP        0x21

/** Notify Request. */
#define BLE_ATT_OP_NOTIFY_REQ               0x1b

/** Indicate Request. */
#define BLE_ATT_OP_INDICATE_REQ             0x1d

/** Indicate Response. */
#define BLE_ATT_OP_INDICATE_RSP             0x1e

/** Multiple Handle Value Length Notification Request */
#define BLE_ATT_OP_NOTIFY_MULTI_REQ         0x23

/** Write Command. */
#define BLE_ATT_OP_WRITE_CMD                0x52

/** Signed Write Command */
#define BLE_ATT_OP_SIGNED_WRITE_CMD         0xD2

/** @} */

/** Maximum length of an Attribute Protocol (ATT) attribute. */
#define BLE_ATT_ATTR_MAX_LEN                512

/**
 * @defgroup ble_att_flags Attribute Protocol (ATT) Flags
 * @{
 */

/** Read permission flag. */
#define BLE_ATT_F_READ                      0x01

/** Write permission flag. */
#define BLE_ATT_F_WRITE                     0x02

/** Read encrypted permission flag. */
#define BLE_ATT_F_READ_ENC                  0x04

/** Read authenticated permission flag. */
#define BLE_ATT_F_READ_AUTHEN               0x08

/** Read authorized permission flag. */
#define BLE_ATT_F_READ_AUTHOR               0x10

/** Write encrypted permission flag. */
#define BLE_ATT_F_WRITE_ENC                 0x20

/** Write authenticated permission flag. */
#define BLE_ATT_F_WRITE_AUTHEN              0x40

/** Write authorized permission flag. */
#define BLE_ATT_F_WRITE_AUTHOR              0x80

/** Read and write permission flag. */
#define HA_FLAG_PERM_RW                     (BLE_ATT_F_READ | BLE_ATT_F_WRITE)

/** @} */

/**
 * @defgroup ble_att_access_op_codes Attribute Protocol (ATT) Access Operation Codes
 * @{
 */

/** Access operation: Read. */
#define BLE_ATT_ACCESS_OP_READ              1

/** Access operation: Write. */
#define BLE_ATT_ACCESS_OP_WRITE             2

/** @} */

/** Default ATT MTU. Also the minimum. */
#define BLE_ATT_MTU_DFLT                    23

/**
 * An ATT MTU of 527 allows the largest ATT command (signed write) to contain a
 * 512-byte attribute value.
 */
#define BLE_ATT_MTU_MAX                     527

/**
 * Reads a locally registered attribute.  If the specified attribute handle
 * corresponds to a GATT characteristic value or descriptor, the read is
 * performed by calling the registered GATT access callback.
 *
 * @param attr_handle           The 16-bit handle of the attribute to read.
 * @param out_om                On success, this is made to point to a
 *                                  newly-allocated mbuf containing the
 *                                  attribute data read.
 *
 * @return                      0 on success;
 *                              NimBLE host ATT return code if the attribute
 *                                  access callback reports failure;
 *                              NimBLE host core return code on unexpected
 *                                  error.
 */
int ble_att_svr_read_local(uint16_t attr_handle, struct os_mbuf **out_om);

/**
 * Writes a locally registered attribute.  This function consumes the supplied
 * mbuf regardless of the outcome.  If the specified attribute handle
 * corresponds to a GATT characteristic value or descriptor, the write is
 * performed by calling the registered GATT access callback.
 *
 * @param attr_handle           The 16-bit handle of the attribute to write.
 * @param om                    The value to write to the attribute.
 *
 * @return                      0 on success;
 *                              NimBLE host ATT return code if the attribute
 *                                  access callback reports failure;
 *                              NimBLE host core return code on unexpected
 *                                  error.
 */
int ble_att_svr_write_local(uint16_t attr_handle, struct os_mbuf *om);

/**
 * Retrieves the ATT MTU of the specified connection.  If an MTU exchange for
 * this connection has occurred, the MTU is the lower of the two peers'
 * preferred values.  Otherwise, the MTU is the default value of 23.
 *
 * @param conn_handle           The handle of the connection to query.
 *
 * @return                      The specified connection's ATT MTU, or 0 if
 *                                  there is no such connection.
 */
uint16_t ble_att_mtu(uint16_t conn_handle);

/**
 * Retrieves the preferred ATT MTU.  This is the value indicated by the device
 * during an ATT MTU exchange.
 *
 * @return                      The preferred ATT MTU.
 */
uint16_t ble_att_preferred_mtu(void);

/**
 * Sets the preferred ATT MTU; the device will indicate this value in all
 * subsequent ATT MTU exchanges.  The ATT MTU of a connection is equal to the
 * lower of the two peers' preferred MTU values.  The ATT MTU is what dictates
 * the maximum size of any message sent during a GATT procedure.
 *
 * The specified MTU must be within the following range: [23, BLE_ATT_MTU_MAX].
 * 23 is a minimum imposed by the Bluetooth specification; BLE_ATT_MTU_MAX is a
 * NimBLE compile-time setting.
 *
 * @param mtu                   The preferred ATT MTU.
 *
 * @return                      0 on success;
 *                              BLE_HS_EINVAL if the specified value is not
 *                                  within the allowed range.
 */
int ble_att_set_preferred_mtu(uint16_t mtu);

int ble_att_set_default_bearer_using_cid(uint16_t conn_handle, uint16_t cid);
uint16_t ble_att_get_default_bearer_cid(uint16_t conn_handle);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
