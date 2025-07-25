/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ESP_PARTITION_H__
#define __ESP_PARTITION_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file esp_partition.h
 * @brief Partition APIs
 */

/** @cond */
typedef struct esp_flash_t esp_flash_t;
/** @endcond */

/**
 * @brief Enumeration which specifies memory space requested in an mmap call
 */
typedef enum {
    ESP_PARTITION_MMAP_DATA,    /**< map to data memory (Vaddr0), allows byte-aligned access, (4 MB total - only for esp32) */
    ESP_PARTITION_MMAP_INST,    /**< map to instruction memory (Vaddr1-3), allows only 4-byte-aligned access, (11 MB total - only for esp32) */
} esp_partition_mmap_memory_t;

/**
 * @brief Opaque handle for memory region obtained from esp_partition_mmap.
 */
typedef uint32_t esp_partition_mmap_handle_t;

/**
 * @brief Partition type
 *
 * @note Partition types with integer value 0x00-0x3F are reserved for partition types defined by ESP-IDF.
 * Any other integer value 0x40-0xFE can be used by individual applications, without restriction.
 *
 * @internal Keep this enum in sync with PartitionDefinition class gen_esp32part.py @endinternal
 *
 */
typedef enum {
    ESP_PARTITION_TYPE_APP = 0x00,       //!< Application partition type
    ESP_PARTITION_TYPE_DATA = 0x01,      //!< Data partition type
    ESP_PARTITION_TYPE_BOOTLOADER = 0x02, //!< Bootloader partition type
    ESP_PARTITION_TYPE_PARTITION_TABLE = 0x03, //!< Partition table type

    ESP_PARTITION_TYPE_ANY = 0xff,       //!< Used to search for partitions with any type
} esp_partition_type_t;

/**
 * @brief Partition subtype
 *
 * @note These ESP-IDF-defined partition subtypes apply to partitions of type ESP_PARTITION_TYPE_APP
 * and ESP_PARTITION_TYPE_DATA.
 *
 * Application-defined partition types (0x40-0xFE) can set any numeric subtype value.
 *
 * @internal Keep this enum in sync with PartitionDefinition class gen_esp32part.py @endinternal
 */
typedef enum {
    ESP_PARTITION_SUBTYPE_BOOTLOADER_PRIMARY = 0x00,                          //!< Primary Bootloader
    ESP_PARTITION_SUBTYPE_BOOTLOADER_OTA = 0x01,                              //!< Temporary OTA storage for Bootloader, where the OTA uploads a new Bootloader image

    ESP_PARTITION_SUBTYPE_PARTITION_TABLE_PRIMARY = 0x00,                     //!< Primary Partition table
    ESP_PARTITION_SUBTYPE_PARTITION_TABLE_OTA = 0x01,                         //!< Temporary OTA storage for Partition table, where the OTA uploads a new Partition table image

    ESP_PARTITION_SUBTYPE_APP_FACTORY = 0x00,                                 //!< Factory application partition
    ESP_PARTITION_SUBTYPE_APP_OTA_MIN = 0x10,                                 //!< Base for OTA partition subtypes
    ESP_PARTITION_SUBTYPE_APP_OTA_0 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 0,  //!< OTA partition 0
    ESP_PARTITION_SUBTYPE_APP_OTA_1 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 1,  //!< OTA partition 1
    ESP_PARTITION_SUBTYPE_APP_OTA_2 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 2,  //!< OTA partition 2
    ESP_PARTITION_SUBTYPE_APP_OTA_3 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 3,  //!< OTA partition 3
    ESP_PARTITION_SUBTYPE_APP_OTA_4 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 4,  //!< OTA partition 4
    ESP_PARTITION_SUBTYPE_APP_OTA_5 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 5,  //!< OTA partition 5
    ESP_PARTITION_SUBTYPE_APP_OTA_6 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 6,  //!< OTA partition 6
    ESP_PARTITION_SUBTYPE_APP_OTA_7 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 7,  //!< OTA partition 7
    ESP_PARTITION_SUBTYPE_APP_OTA_8 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 8,  //!< OTA partition 8
    ESP_PARTITION_SUBTYPE_APP_OTA_9 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 9,  //!< OTA partition 9
    ESP_PARTITION_SUBTYPE_APP_OTA_10 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 10,//!< OTA partition 10
    ESP_PARTITION_SUBTYPE_APP_OTA_11 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 11,//!< OTA partition 11
    ESP_PARTITION_SUBTYPE_APP_OTA_12 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 12,//!< OTA partition 12
    ESP_PARTITION_SUBTYPE_APP_OTA_13 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 13,//!< OTA partition 13
    ESP_PARTITION_SUBTYPE_APP_OTA_14 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 14,//!< OTA partition 14
    ESP_PARTITION_SUBTYPE_APP_OTA_15 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 15,//!< OTA partition 15
    ESP_PARTITION_SUBTYPE_APP_OTA_MAX = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 16,//!< Max subtype of OTA partition
    ESP_PARTITION_SUBTYPE_APP_TEST = 0x20,                                    //!< Test application partition

    ESP_PARTITION_SUBTYPE_DATA_OTA = 0x00,                                    //!< OTA selection partition
    ESP_PARTITION_SUBTYPE_DATA_PHY = 0x01,                                    //!< PHY init data partition
    ESP_PARTITION_SUBTYPE_DATA_NVS = 0x02,                                    //!< NVS partition
    ESP_PARTITION_SUBTYPE_DATA_COREDUMP = 0x03,                               //!< COREDUMP partition
    ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS = 0x04,                               //!< Partition for NVS keys
    ESP_PARTITION_SUBTYPE_DATA_EFUSE_EM = 0x05,                               //!< Partition for emulate eFuse bits
    ESP_PARTITION_SUBTYPE_DATA_UNDEFINED = 0x06,                              //!< Undefined (or unspecified) data partition

    ESP_PARTITION_SUBTYPE_DATA_ESPHTTPD = 0x80,                               //!< ESPHTTPD partition
    ESP_PARTITION_SUBTYPE_DATA_FAT = 0x81,                                    //!< FAT partition
    ESP_PARTITION_SUBTYPE_DATA_SPIFFS = 0x82,                                 //!< SPIFFS partition
    ESP_PARTITION_SUBTYPE_DATA_LITTLEFS = 0x83,                               //!< LITTLEFS partition

#if __has_include("extra_partition_subtypes.inc")
    #include "extra_partition_subtypes.inc"
#endif

    ESP_PARTITION_SUBTYPE_ANY = 0xff,                                         //!< Used to search for partitions with any subtype
} esp_partition_subtype_t;

/**
 * @brief Convenience macro to get esp_partition_subtype_t value for the i-th OTA partition
 */
#define ESP_PARTITION_SUBTYPE_OTA(i) ((esp_partition_subtype_t)(ESP_PARTITION_SUBTYPE_APP_OTA_MIN + ((i) & 0xf)))

/**
 * @brief Opaque partition iterator type
 */
typedef struct esp_partition_iterator_opaque_* esp_partition_iterator_t;

/**
 * @brief partition information structure
 *
 * This is not the format in flash, that format is esp_partition_info_t.
 *
 * However, this is the format used by this API.
 */
typedef struct {
    esp_flash_t* flash_chip;            /*!< SPI flash chip on which the partition resides */
    esp_partition_type_t type;          /*!< partition type (app/data) */
    esp_partition_subtype_t subtype;    /*!< partition subtype */
    uint32_t address;                   /*!< starting address of the partition in flash */
    uint32_t size;                      /*!< size of the partition, in bytes */
    uint32_t erase_size;                /*!< size the erase operation should be aligned to */
    char label[17];                     /*!< partition label, zero-terminated ASCII string */
    bool encrypted;                     /*!< flag is set to true if partition is encrypted */
    bool readonly;                      /*!< flag is set to true if partition is read-only */
} esp_partition_t;

/**
 * @brief Find partition based on one or more parameters
 *
 * @param type Partition type, one of esp_partition_type_t values or an 8-bit unsigned integer.
 *             To find all partitions, no matter the type, use ESP_PARTITION_TYPE_ANY, and set
 *             subtype argument to ESP_PARTITION_SUBTYPE_ANY.
 * @param subtype Partition subtype, one of esp_partition_subtype_t values or an 8-bit unsigned integer.
 *                To find all partitions of given type, use ESP_PARTITION_SUBTYPE_ANY.
 * @param label (optional) Partition label. Set this value if looking
 *             for partition with a specific name. Pass NULL otherwise.
 *
 * @return iterator which can be used to enumerate all the partitions found,
 *         or NULL if no partitions were found.
 *         Iterator obtained through this function has to be released
 *         using esp_partition_iterator_release when not used any more.
 */
esp_partition_iterator_t esp_partition_find(esp_partition_type_t type, esp_partition_subtype_t subtype, const char* label);

/**
 * @brief Find first partition based on one or more parameters
 *
 * @param type Partition type, one of esp_partition_type_t values or an 8-bit unsigned integer.
 *             To find all partitions, no matter the type, use ESP_PARTITION_TYPE_ANY, and set
 *             subtype argument to ESP_PARTITION_SUBTYPE_ANY.
 * @param subtype Partition subtype, one of esp_partition_subtype_t values or an 8-bit unsigned integer
 *                To find all partitions of given type, use ESP_PARTITION_SUBTYPE_ANY.
 * @param label (optional) Partition label. Set this value if looking
 *             for partition with a specific name. Pass NULL otherwise.
 *
 * @return pointer to esp_partition_t structure, or NULL if no partition is found.
 *         This pointer is valid for the lifetime of the application.
 */
const esp_partition_t* esp_partition_find_first(esp_partition_type_t type, esp_partition_subtype_t subtype, const char* label);

/**
 * @brief Get esp_partition_t structure for given partition
 *
 * @param iterator  Iterator obtained using esp_partition_find. Must be non-NULL.
 *
 * @return pointer to esp_partition_t structure. This pointer is valid for the lifetime
 *         of the application.
 */
const esp_partition_t* esp_partition_get(esp_partition_iterator_t iterator);

/**
 * @brief Move partition iterator to the next partition found
 *
 * Any copies of the iterator will be invalid after this call.
 *
 * @param iterator Iterator obtained using esp_partition_find. Must be non-NULL.
 *
 * @return NULL if no partition was found, valid esp_partition_iterator_t otherwise.
 */
esp_partition_iterator_t esp_partition_next(esp_partition_iterator_t iterator);

/**
 * @brief Release partition iterator
 *
 * @param iterator Iterator obtained using esp_partition_find.
 *                 The iterator is allowed to be NULL, so it is not necessary to check its value
 *                 before calling this function.
 *
 */
void esp_partition_iterator_release(esp_partition_iterator_t iterator);

/**
 * @brief Verify partition data
 *
 * Given a pointer to partition data, verify this partition exists in the partition table (all fields match.)
 *
 * This function is also useful to take partition data which may be in a RAM buffer and convert it to a pointer to the
 * permanent partition data stored in flash.
 *
 * Pointers returned from this function can be compared directly to the address of any pointer returned from
 * esp_partition_get(), as a test for equality.
 *
 * @param partition Pointer to partition data to verify. Must be non-NULL. All fields of this structure must match the
 * partition table entry in flash for this function to return a successful match.
 *
 * @return
 * - If partition not found, returns NULL.
 * - If found, returns a pointer to the esp_partition_t structure in flash. This pointer is always valid for the lifetime of the application.
 */
const esp_partition_t* esp_partition_verify(const esp_partition_t* partition);

/**
 * @brief Read data from the partition
 *
 * Partitions marked with an encryption flag will automatically be
 * be read and decrypted via a cache mapping.
 *
 * @param partition Pointer to partition structure obtained using
 *                  esp_partition_find_first or esp_partition_get.
 *                  Must be non-NULL.
 * @param dst Pointer to the buffer where data should be stored.
 *            Pointer must be non-NULL and buffer must be at least 'size' bytes long.
 * @param src_offset Address of the data to be read, relative to the
 *                   beginning of the partition.
 * @param size Size of data to be read, in bytes.
 *
 * @return ESP_OK, if data was read successfully;
 *         ESP_ERR_INVALID_ARG, if src_offset exceeds partition size;
 *         ESP_ERR_INVALID_SIZE, if read would go out of bounds of the partition;
 *         or one of error codes from lower-level flash driver.
 */
esp_err_t esp_partition_read(const esp_partition_t* partition,
                             size_t src_offset, void* dst, size_t size);

/**
 * @brief Write data to the partition
 *
 * Before writing data to flash, corresponding region of flash needs to be erased.
 * This can be done using esp_partition_erase_range function.
 *
 * Partitions marked with an encryption flag will automatically be
 * written via the esp_flash_write_encrypted() function. If writing to
 * an encrypted partition, all write offsets and lengths must be
 * multiples of 16 bytes. See the esp_flash_write_encrypted() function
 * for more details. Unencrypted partitions do not have this
 * restriction.
 *
 * @param partition Pointer to partition structure obtained using
 *                  esp_partition_find_first or esp_partition_get.
 *                  Must be non-NULL.
 * @param dst_offset Address where the data should be written, relative to the
 *                   beginning of the partition.
 * @param src Pointer to the source buffer.  Pointer must be non-NULL and
 *            buffer must be at least 'size' bytes long.
 * @param size Size of data to be written, in bytes.
 *
 * @note Prior to writing to flash memory, make sure it has been erased with
 *       esp_partition_erase_range call.
 *
 * @return ESP_OK, if data was written successfully;
 *         ESP_ERR_INVALID_ARG, if dst_offset exceeds partition size;
 *         ESP_ERR_INVALID_SIZE, if write would go out of bounds of the partition;
 *         ESP_ERR_NOT_ALLOWED, if partition is read-only;
 *         or one of error codes from lower-level flash driver.
 */
esp_err_t esp_partition_write(const esp_partition_t* partition,
                              size_t dst_offset, const void* src, size_t size);

/**
 * @brief Read data from the partition without any transformation/decryption.
 *
 * @note This function is essentially the same as \c esp_partition_read() above.
 *       It just never decrypts data but returns it as is.
 *
 * @param partition Pointer to partition structure obtained using
 *                  esp_partition_find_first or esp_partition_get.
 *                  Must be non-NULL.
 * @param dst Pointer to the buffer where data should be stored.
 *            Pointer must be non-NULL and buffer must be at least 'size' bytes long.
 * @param src_offset Address of the data to be read, relative to the
 *                   beginning of the partition.
 * @param size Size of data to be read, in bytes.
 *
 * @return ESP_OK, if data was read successfully;
 *         ESP_ERR_INVALID_ARG, if src_offset exceeds partition size;
 *         ESP_ERR_INVALID_SIZE, if read would go out of bounds of the partition;
 *         or one of error codes from lower-level flash driver.
 */
esp_err_t esp_partition_read_raw(const esp_partition_t* partition,
                                 size_t src_offset, void* dst, size_t size);

/**
 * @brief Write data to the partition without any transformation/encryption.
 *
 * @note This function is essentially the same as \c esp_partition_write() above.
 *       It just never encrypts data but writes it as is.
 *
 * Before writing data to flash, corresponding region of flash needs to be erased.
 * This can be done using esp_partition_erase_range function.
 *
 * @param partition Pointer to partition structure obtained using
 *                  esp_partition_find_first or esp_partition_get.
 *                  Must be non-NULL.
 * @param dst_offset Address where the data should be written, relative to the
 *                   beginning of the partition.
 * @param src Pointer to the source buffer.  Pointer must be non-NULL and
 *            buffer must be at least 'size' bytes long.
 * @param size Size of data to be written, in bytes.
 *
 * @note Prior to writing to flash memory, make sure it has been erased with
 *       esp_partition_erase_range call.
 *
 * @return ESP_OK, if data was written successfully;
 *         ESP_ERR_INVALID_ARG, if dst_offset exceeds partition size;
 *         ESP_ERR_INVALID_SIZE, if write would go out of bounds of the partition;
 *         ESP_ERR_NOT_ALLOWED, if partition is read-only;
 *         or one of the error codes from lower-level flash driver.
 */
esp_err_t esp_partition_write_raw(const esp_partition_t* partition,
                                  size_t dst_offset, const void* src, size_t size);

/**
 * @brief Erase part of the partition
 *
 * @param partition Pointer to partition structure obtained using
 *                  esp_partition_find_first or esp_partition_get.
 *                  Must be non-NULL.
 * @param offset Offset from the beginning of partition where erase operation
 *               should start. Must be aligned to partition->erase_size.
 * @param size Size of the range which should be erased, in bytes.
 *                   Must be divisible by partition->erase_size.
 *
 * @return ESP_OK, if the range was erased successfully;
 *         ESP_ERR_INVALID_ARG, if iterator or dst are NULL;
 *         ESP_ERR_INVALID_SIZE, if erase would go out of bounds of the partition;
 *         ESP_ERR_NOT_ALLOWED, if partition is read-only;
 *         or one of error codes from lower-level flash driver.
 */
esp_err_t esp_partition_erase_range(const esp_partition_t* partition,
                                    size_t offset, size_t size);

/**
 * @brief Configure MMU to map partition into data memory
 *
 * Unlike spi_flash_mmap function, which requires a 64kB aligned base address,
 * this function doesn't impose such a requirement.
 * If offset results in a flash address which is not aligned to 64kB boundary,
 * address will be rounded to the lower 64kB boundary, so that mapped region
 * includes requested range.
 * Pointer returned via out_ptr argument will be adjusted to point to the
 * requested offset (not necessarily to the beginning of mmap-ed region).
 *
 * To release mapped memory, pass handle returned via out_handle argument to
 * esp_partition_munmap function.
 *
 * @param partition Pointer to partition structure obtained using
 *                  esp_partition_find_first or esp_partition_get.
 *                  Must be non-NULL.
 * @param offset Offset from the beginning of partition where mapping should start.
 * @param size Size of the area to be mapped.
 * @param memory  Memory space where the region should be mapped
 * @param out_ptr  Output, pointer to the mapped memory region
 * @param out_handle  Output, handle which should be used for esp_partition_munmap call
 *
 * @return ESP_OK, if successful
 */
esp_err_t esp_partition_mmap(const esp_partition_t* partition, size_t offset, size_t size,
                             esp_partition_mmap_memory_t memory,
                             const void** out_ptr, esp_partition_mmap_handle_t* out_handle);

/**
 * @brief Release region previously obtained using esp_partition_mmap
 *
 * @note Calling this function will not necessarily unmap memory region.
 *       Region will only be unmapped when there are no other handles which
 *       reference this region. In case of partially overlapping regions
 *       it is possible that memory will be unmapped partially.
 *
 * @param handle  Handle obtained from spi_flash_mmap
 */
void esp_partition_munmap(esp_partition_mmap_handle_t handle);

/**
 * @brief Get SHA-256 digest for required partition.
 *
 * For apps with SHA-256 appended to the app image, the result is the appended SHA-256 value for the app image content.
 * The hash is verified before returning, if app content is invalid then the function returns ESP_ERR_IMAGE_INVALID.
 * For apps without SHA-256 appended to the image, the result is the SHA-256 of all bytes in the app image.
 * For other partition types, the result is the SHA-256 of the entire partition.
 *
 * @param[in]  partition    Pointer to info for partition containing app or data. (fields: address, size and type, are required to be filled).
 * @param[out] sha_256      Returned SHA-256 digest for a given partition.
 *
 * @return
 *          - ESP_OK: In case of successful operation.
 *          - ESP_ERR_INVALID_ARG: The size was 0 or the sha_256 was NULL.
 *          - ESP_ERR_NO_MEM: Cannot allocate memory for sha256 operation.
 *          - ESP_ERR_IMAGE_INVALID: App partition doesn't contain a valid app image.
 *          - ESP_FAIL: An allocation error occurred.
 */
esp_err_t esp_partition_get_sha256(const esp_partition_t* partition, uint8_t* sha_256);

/**
 * @brief Check for the identity of two partitions by SHA-256 digest.
 *
 * @param[in] partition_1 Pointer to info for partition 1 containing app or data. (fields: address, size and type, are required to be filled).
 * @param[in] partition_2 Pointer to info for partition 2 containing app or data. (fields: address, size and type, are required to be filled).
 *
 * @return
 *         - True:  In case of the two firmware is equal.
 *         - False: Otherwise
 */
bool esp_partition_check_identity(const esp_partition_t* partition_1, const esp_partition_t* partition_2);

/**
 * @brief Register a partition on an external flash chip
 *
 * This API allows designating certain areas of external flash chips (identified by the esp_flash_t structure)
 * as partitions. This allows using them with components which access SPI flash through the esp_partition API.
 *
 * @param flash_chip  Pointer to the structure identifying the flash chip. If NULL then the internal flash chip is used (esp_flash_default_chip).
 * @param offset  Address in bytes, where the partition starts
 * @param size  Size of the partition in bytes
 * @param label  Partition name
 * @param type  One of the partition types (ESP_PARTITION_TYPE_*), or an integer. Note that applications can not be booted from external flash
 *              chips, so using ESP_PARTITION_TYPE_APP is not supported.
 * @param subtype  One of the partition subtypes (ESP_PARTITION_SUBTYPE_*), or an integer.
 * @param[out] out_partition  Output, if non-NULL, receives the pointer to the resulting esp_partition_t structure
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NO_MEM if memory allocation has failed
 *      - ESP_ERR_INVALID_ARG if the new partition overlaps another partition on the same flash chip
 *      - ESP_ERR_INVALID_SIZE if the partition doesn't fit into the flash chip size
 */
esp_err_t esp_partition_register_external(esp_flash_t* flash_chip, size_t offset, size_t size,
                                          const char* label, esp_partition_type_t type, esp_partition_subtype_t subtype,
                                          const esp_partition_t** out_partition);

/**
 * @brief Deregister the partition previously registered using esp_partition_register_external
 * @param partition  pointer to the partition structure obtained from esp_partition_register_external,
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NOT_FOUND if the partition pointer is not found
 *      - ESP_ERR_INVALID_ARG if the partition comes from the partition table
 *      - ESP_ERR_INVALID_ARG if the partition was not registered using
 *        esp_partition_register_external function.
 */
esp_err_t esp_partition_deregister_external(const esp_partition_t* partition);

/**
 * @brief Unload partitions and free space allocated by them
 */
void esp_partition_unload_all(void);

/**
 * @brief Get the main flash sector size
 * @return
 *      - SPI_FLASH_SEC_SIZE - For esp32xx target
 *      - ESP_PARTITION_EMULATED_SECTOR_SIZE - For linux target
 */
uint32_t esp_partition_get_main_flash_sector_size(void);

/**
 * @brief Copy data from a source partition at a specific offset to a destination partition at a specific offset.
 *
 * The destination offset must be aligned to the flash sector size (SPI_FLASH_SEC_SIZE = 0x1000).
 * If "size" is SIZE_MAX, the entire destination partition (from dest_offset onward) will be erased,
 * and the function will copy all of the source partition starting from src_offset into the destination.
 * The function ensures that the destination partition is erased on sector boundaries (erase size is aligned up SPI_FLASH_SEC_SIZE).
 *
 * This function does the following:
 * - erases the destination partition from dest_offset to the specified size (or the whole partition if "size" == SIZE_MAX),
 * - maps data from the source partition in chunks,
 * - writes the source data into the destination partition in corresponding chunks.
 *
 * @param dest_part   Pointer to a destination partition.
 * @param dest_offset Offset in the destination partition where the data should be written (must be aligned to SPI_FLASH_SEC_SIZE = 0x1000).
 * @param src_part    Pointer to a source partition (must be located on internal flash).
 * @param src_offset  Offset in the source partition where the data should be read from.
 * @param size        Number of bytes to copy from the source partition to the destination partition. If "size" is SIZE_MAX,
 *                    the function copies from src_offset to the end of the source partition and erases
 *                    the entire destination partition (from dest_offset onward).
 *
 * @return ESP_OK, if the source partition was copied successfully to the destination partition;
 *         ESP_ERR_INVALID_ARG, if src_part or dest_part are incorrect, or if dest_offset is not sector aligned;
 *         ESP_ERR_INVALID_SIZE, if the copy would go out of bounds of the source or destination partition;
 *         ESP_ERR_NOT_ALLOWED, if the destination partition is read-only;
 *         or one of the error codes from the lower-level flash driver.
 */
esp_err_t esp_partition_copy(const esp_partition_t* dest_part, uint32_t dest_offset, const esp_partition_t* src_part, uint32_t src_offset, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_PARTITION_H__ */
