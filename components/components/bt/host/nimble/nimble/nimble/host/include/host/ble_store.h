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

#ifndef H_BLE_STORE_
#define H_BLE_STORE_

#include <inttypes.h>
#include "nimble/ble.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_STORE_OBJ_TYPE_OUR_SEC           1
#define BLE_STORE_OBJ_TYPE_PEER_SEC          2
#define BLE_STORE_OBJ_TYPE_CCCD              3
#define BLE_STORE_OBJ_TYPE_PEER_DEV_REC      4

#if MYNEWT_VAL(ENC_ADV_DATA)
#define BLE_STORE_OBJ_TYPE_ENC_ADV_DATA      5
#endif
#define BLE_STORE_OBJ_TYPE_PEER_ADDR         6

#define BLE_STORE_OBJ_TYPE_LOCAL_IRK        7
#define BLE_STORE_OBJ_TYPE_CSFC              8
/** Failed to persist record; insufficient storage capacity. */
#define BLE_STORE_EVENT_OVERFLOW        1

/** About to execute a procedure that may fail due to overflow. */
#define BLE_STORE_EVENT_FULL            2

/**
 * Used as a key for lookups of security material.  This struct corresponds to
 * the following store object types:
 *     o BLE_STORE_OBJ_TYPE_OUR_SEC
 *     o BLE_STORE_OBJ_TYPE_PEER_SEC
 */
struct ble_store_key_sec {
    /**
     * Key by peer identity address;
     * peer_addr=BLE_ADDR_NONE means don't key off peer.
     */
    ble_addr_t peer_addr;

    /** Number of results to skip; 0 means retrieve the first match. */
    uint8_t idx;
};

/**
 * Represents stored security material.  This struct corresponds to the
 * following store object types:
 *     o BLE_STORE_OBJ_TYPE_OUR_SEC
 *     o BLE_STORE_OBJ_TYPE_PEER_SEC
 */
struct ble_store_value_sec {
    ble_addr_t peer_addr;
    uint16_t bond_count;

    uint8_t key_size;
    uint16_t ediv;
    uint64_t rand_num;
    uint8_t ltk[16];
    uint8_t ltk_present:1;

    uint8_t irk[16];
    uint8_t irk_present:1;

    uint8_t csrk[16];
    uint8_t csrk_present:1;
    uint32_t sign_counter;

    unsigned authenticated:1;
    uint8_t sc:1;
};

/**
 * Used as a key for lookups of stored client characteristic configuration
 * descriptors (CCCDs).  This struct corresponds to the BLE_STORE_OBJ_TYPE_CCCD
 * store object type.
 */
struct ble_store_key_cccd {
    /**
     * Key by peer identity address;
     * peer_addr=BLE_ADDR_NONE means don't key off peer.
     */
    ble_addr_t peer_addr;

    /**
     * Key by characteristic value handle;
     * chr_val_handle=0 means don't key off characteristic handle.
     */
    uint16_t chr_val_handle;

    /** Number of results to skip; 0 means retrieve the first match. */
    uint8_t idx;
};

/**
 * Represents a stored client characteristic configuration descriptor (CCCD).
 * This struct corresponds to the BLE_STORE_OBJ_TYPE_CCCD store object type.
 */
struct ble_store_value_cccd {
    ble_addr_t peer_addr;
    uint16_t chr_val_handle;
    uint16_t flags;
    unsigned value_changed:1;
};

/**
 * Used as a key for lookups of stored client supported features characteristic (CSFC).
 * This struct corresponds to the BLE_STORE_OBJ_TYPE_CSFC store object type.
 */
struct ble_store_key_csfc {
    /**
     * Key by peer identity address;
     */
    ble_addr_t peer_addr;

    /** Number of results to skip; 0 means retrieve the first match. */
    uint8_t idx;
};

/**
 * Represents a stored client supported features characteristic (CSFC).
 * This struct corresponds to the BLE_STORE_OBJ_TYPE_CSFC store object type.
 */
struct ble_store_value_csfc {
    ble_addr_t peer_addr;
    uint8_t csfc[MYNEWT_VAL(BLE_GATT_CSFC_SIZE)];
};

#if MYNEWT_VAL(ENC_ADV_DATA)
/**
 * Used as a key for lookups of encrypted advertising data. This struct corresponds
 * to the BLE_STORE_OBJ_TYPE_ENC_ADV_DATA store object type.
 */
struct ble_store_key_ead {
    /**
     * Key by peer identity address;
     * peer_addr=BLE_ADDR_NONE means don't key off peer.
     */
    ble_addr_t peer_addr;

    /** Number of results to skip; 0 means retrieve the first match. */
    uint8_t idx;
};

/**
 * Represents a stored encrypted advertising data. This struct corresponds
 * to the BLE_STORE_OBJ_TYPE_ENC_ADV_DATA store object type.
 */
struct ble_store_value_ead {
    ble_addr_t peer_addr;
    unsigned km_present:1;
    struct key_material *km;
};
#endif

struct ble_store_key_local_irk {
    ble_addr_t addr;

    uint8_t idx;
};

struct ble_store_value_local_irk {
     ble_addr_t addr;

     uint8_t irk[16];
};

struct ble_store_key_rpa_rec{
    ble_addr_t peer_rpa_addr;
    uint8_t idx;
};
struct ble_store_value_rpa_rec{
    ble_addr_t peer_rpa_addr;
    ble_addr_t peer_addr;
};

/**
 * Used as a key for store lookups.  This union must be accompanied by an
 * object type code to indicate which field is valid.
 */
union ble_store_key {
    struct ble_store_key_sec sec;
    struct ble_store_key_cccd cccd;
#if MYNEWT_VAL(ENC_ADV_DATA)
    struct ble_store_key_ead ead;
#endif
    struct ble_store_key_rpa_rec rpa_rec;
    struct ble_store_key_local_irk local_irk;
    struct ble_store_key_csfc csfc;
};

/**
 * Represents stored data.  This union must be accompanied by an object type
 * code to indicate which field is valid.
 */
union ble_store_value {
    struct ble_store_value_sec sec;
    struct ble_store_value_cccd cccd;
#if MYNEWT_VAL(ENC_ADV_DATA)
    struct ble_store_value_ead ead;
#endif
   struct ble_store_value_rpa_rec rpa_rec;
   struct ble_store_value_local_irk local_irk;
   struct ble_store_value_csfc csfc;
};

struct ble_store_status_event {
    /**
     * The type of event being reported; one of the BLE_STORE_EVENT_TYPE_[...]
     * codes.
     */
    int event_code;

    /**
     * Additional data related to the event; the valid field is inferred from
     * the obj_type,event_code pair.
     */
    union {
        /**
         * Represents a write that failed due to storage exhaustion.  Valid for
         * the following event types:
         *     o BLE_STORE_EVENT_OVERFLOW
         */
        struct {
            /** The type of object that failed to be written. */
            int obj_type;

            /** The object that failed to be written. */
            const union ble_store_value *value;
        } overflow;

        /**
         * Represents the possibility that a scheduled write will fail due to
         * storage exhaustion.  Valid for the following event types:
         *     o BLE_STORE_EVENT_FULL
         */
        struct {
            /** The type of object that may fail to be written. */
            int obj_type;

            /** The handle of the connection which prompted the write. */
            uint16_t conn_handle;
        } full;
    };
};

/* Generate LTK, EDIT and Rand */
#define BLE_STORE_GEN_KEY_LTK       0x01
/* Generate IRK */
#define BLE_STORE_GEN_KEY_IRK       0x02
/* Generate CSRK */
#define BLE_STORE_GEN_KEY_CSRK      0x03

struct ble_store_gen_key {
    union {
        uint8_t ltk_periph[16];
        uint8_t irk[16];
        uint8_t csrk[16];
    };
    uint16_t ediv;
    uint64_t rand;
};

/**
 * Generates key required by security module.
 * This can be used to use custom routines to generate keys instead of simply
 * randomizing them.
 *
 * \p conn_handle is set to \p BLE_HS_CONN_HANDLE_NONE if key is not requested
 * for a specific connection (e.g. an IRK).
 *
 * @param key                   Key that shall be generated.
 * @param gen_key               Storage for generated key.
 * @param conn_handle           Connection handle for which keys are generated.
 *
 * @return                      0 if keys were generated successfully
 *                              Other nonzero on error.
 */
typedef int ble_store_gen_key_fn(uint8_t key,
                                 struct ble_store_gen_key *gen_key,
                                 uint16_t conn_handle);

/**
 * Searches the store for an object matching the specified criteria.  If a
 * match is found, it is read from the store and the dst parameter is populated
 * with the retrieved object.
 *
 * @param obj_type              The type of object to search for; one of the
 *                                  BLE_STORE_OBJ_TYPE_[...] codes.
 * @param key                   Specifies properties of the object to search
 *                                  for.  An object is retrieved if it matches
 *                                  these criteria.
 * @param dst                   On success, this is populated with the
 *                                  retrieved object.
 *
 * @return                      0 if an object was successfully retreived;
 *                              BLE_HS_ENOENT if no matching object was found;
 *                              Other nonzero on error.
 */
typedef int ble_store_read_fn(int obj_type, const union ble_store_key *key,
                              union ble_store_value *dst);

/**
 * Writes the specified object to the store.  If an object with the same
 * identity is already in the store, it is replaced.  If the store lacks
 * sufficient capacity to write the object, this function may remove previously
 * stored values to make room.
 *
 * @param obj_type              The type of object being written; one of the
 *                                  BLE_STORE_OBJ_TYPE_[...] codes.
 * @param val                   The object to persist.
 *
 * @return                      0 if the object was successfully written;
 *                              Other nonzero on error.
 */
typedef int ble_store_write_fn(int obj_type, const union ble_store_value *val);

/**
 * Searches the store for the first object matching the specified criteria.  If
 * a match is found, it is deleted from the store.
 *
 * @param obj_type              The type of object to delete; one of the
 *                                  BLE_STORE_OBJ_TYPE_[...] codes.
 * @param key                   Specifies properties of the object to search
 *                                  for.  An object is deleted if it matches
 *                                  these criteria.
 * @return                      0 if an object was successfully retrieved;
 *                              BLE_HS_ENOENT if no matching object was found;
 *                              Other nonzero on error.
 */
typedef int ble_store_delete_fn(int obj_type, const union ble_store_key *key);

/**
 * Indicates an inability to perform a store operation.  This callback should
 * do one of two things:
 *     o Address the problem and return 0, indicating that the store operation
 *       should proceed.
 *     o Return nonzero to indicate that the store operation should be aborted.
 *
 * @param event                 Describes the store event being reported.
 * @param arg                   Optional user argument.
 *
 * @return                      0 if the store operation should proceed;
 *                              nonzero if the store operation should be
 *                                  aborted.
 */
typedef int ble_store_status_fn(struct ble_store_status_event *event,
                                void *arg);

int ble_store_read(int obj_type, const union ble_store_key *key,
                   union ble_store_value *val);
int ble_store_write(int obj_type, const union ble_store_value *val);
int ble_store_delete(int obj_type, const union ble_store_key *key);
int ble_store_overflow_event(int obj_type, const union ble_store_value *value);
int ble_store_full_event(int obj_type, uint16_t conn_handle);

int ble_store_read_our_sec(const struct ble_store_key_sec *key_sec,
                           struct ble_store_value_sec *value_sec);
int ble_store_write_our_sec(const struct ble_store_value_sec *value_sec);
int ble_store_delete_our_sec(const struct ble_store_key_sec *key_sec);
int ble_store_read_peer_sec(const struct ble_store_key_sec *key_sec,
                            struct ble_store_value_sec *value_sec);
int ble_store_write_peer_sec(const struct ble_store_value_sec *value_sec);
int ble_store_delete_peer_sec(const struct ble_store_key_sec *key_sec);

int ble_store_read_cccd(const struct ble_store_key_cccd *key,
                        struct ble_store_value_cccd *out_value);
int ble_store_write_cccd(const struct ble_store_value_cccd *value);
int ble_store_delete_cccd(const struct ble_store_key_cccd *key);

int ble_store_read_csfc(const struct ble_store_key_csfc *key,
                        struct ble_store_value_csfc *out_value);
int ble_store_write_csfc(const struct ble_store_value_csfc *value);
int ble_store_delete_csfc(const struct ble_store_key_csfc *key);

void ble_store_key_from_value_sec(struct ble_store_key_sec *out_key,
                                  const struct ble_store_value_sec *value);
void ble_store_key_from_value_cccd(struct ble_store_key_cccd *out_key,
                                   const struct ble_store_value_cccd *value);
void ble_store_key_from_value_csfc(struct ble_store_key_csfc *out_key,
                                   const struct ble_store_value_csfc *value);

#if MYNEWT_VAL(ENC_ADV_DATA)
int ble_store_read_ead(const struct ble_store_key_ead *key,
                       struct ble_store_value_ead *out_value);
int ble_store_write_ead(const struct ble_store_value_ead *value);
int ble_store_delete_ead(const struct ble_store_key_ead *key);
void ble_store_key_from_value_ead(struct ble_store_key_ead *out_key,
                                  const struct ble_store_value_ead *value);
#endif
/* irk store*/
int ble_store_read_local_irk(const struct ble_store_key_local_irk *key,
                       struct ble_store_value_local_irk *out_value);
int ble_store_write_local_irk(const struct ble_store_value_local_irk *value);
int ble_store_delete_local_irk(const struct ble_store_key_local_irk *key);
void ble_store_key_from_value_local_irk(struct ble_store_key_local_irk *out_key,
                                  const struct ble_store_value_local_irk *value);

/*irk store */
/* rpa mapping*/
int ble_store_read_rpa_rec(const struct ble_store_key_rpa_rec *key,
                       struct ble_store_value_rpa_rec *out_value);
int ble_store_write_rpa_rec(const struct ble_store_value_rpa_rec *value);
int ble_store_delete_rpa_rec(const struct ble_store_key_rpa_rec *key);
void ble_store_key_from_value_rpa_rec(struct ble_store_key_rpa_rec *out_key,
                                  const struct ble_store_value_rpa_rec *value);
/* rpa mapping*/
void ble_store_key_from_value(int obj_type,
                              union ble_store_key *out_key,
                              const union ble_store_value *value);

typedef int ble_store_iterator_fn(int obj_type,
                                  union ble_store_value *val,
                                  void *cookie);

int ble_store_iterate(int obj_type,
                      ble_store_iterator_fn *callback,
                      void *cookie);

int ble_store_clear(void);

/*** Utility functions. */

int ble_store_util_bonded_peers(ble_addr_t *out_peer_id_addrs,
                                int *out_num_peers,
                                int max_peers);
int ble_store_util_delete_all(int type, const union ble_store_key *key);
int ble_store_util_delete_peer(const ble_addr_t *peer_id_addr);
int ble_store_util_delete_oldest_peer(void);
int ble_store_util_count(int type, int *out_count);
int ble_store_util_status_rr(struct ble_store_status_event *event, void *arg);

#ifdef __cplusplus
}
#endif

#endif
