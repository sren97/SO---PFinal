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

#ifndef H_BLE_SVC_CTE_
#define H_BLE_SVC_CTE_

struct ble_hs_cfg;

#define BLE_SVC_CTE_UUID16                                      0x184A
#define BLE_SVC_CTE_CHR_UUID16_ENABLE                           0x2BAD
#define BLE_SVC_CTE_CHR_UUID16_MINIMUM_LENGTH                   0x2BAE
#define BLE_SVC_CTE_CHR_UUID16_MINIMUM_TRANSMIT_COUNT           0x2BAF
#define BLE_SVC_CTE_CHR_UUID16_TRANSMIT_DURATION                0x2BB0
#define BLE_SVC_CTE_CHR_UUID16_INTERVAL                         0x2BB1
#define BLE_SVC_CTE_CHR_UUID16_PHY                              0x2BB2


void ble_svc_cte_init(void);

#endif

