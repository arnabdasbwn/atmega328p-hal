/**
 * @file fat16.h
 * @brief Bounded, read-only FAT16 access to one primary partition's root directory.
 * @details Supports 512-byte sectors, FAT16, 8.3 names and root-directory files only. One volume
 * owns one sector buffer and supports one open file at a time. There is no heap allocation.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_FAT16_H
#define AVR_FAT16_H

#include "avr_hal/common.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief FAT16 sectors are 512 bytes in this implementation. */
#define AVR_FAT16_SECTOR_SIZE UINT16_C(512)

    /**
     * @brief Application-supplied callback that reads one logical 512-byte sector.
     * @param[in] context Device state passed unchanged from mount.
     * @param[in] logical_block_address Sector number relative to the whole block device.
     * @param[out] sector Caller-owned 512-byte destination; its prefix may change on failure.
     * @return AVR_OK after a complete sector, otherwise the device error.
     * @details Called synchronously from foreground code. It must have a finite timeout and must
     * not call back into the same FAT16 volume.
     */
    typedef avr_status_t (*avr_fat16_read_sector_fn)(void *context, uint32_t logical_block_address,
                                                     uint8_t sector[AVR_FAT16_SECTOR_SIZE]);

    /**
     * @brief Mounted FAT16 metadata and the one sector buffer owned by the volume.
     * @details Keep this object alive while reading an open file. Its 512-byte buffer is the main
     * SRAM cost; do not allocate another sector-sized buffer on the Uno stack.
     */
    typedef struct
    {
        /** @brief Caller-provided block device function. */
        avr_fat16_read_sector_fn read_sector;
        /** @brief Device context passed to read_sector. */
        void *device_context;
        /** @brief Start LBA of the selected MBR partition. */
        uint32_t partition_start;
        /** @brief Sector count declared by the selected partition. */
        uint32_t partition_sectors;
        /** @brief Start LBA of the first FAT copy. */
        uint32_t fat_start;
        /** @brief Start LBA of the fixed root directory. */
        uint32_t root_start;
        /** @brief Start LBA of the data region. */
        uint32_t data_start;
        /** @brief Number of root-directory entries. */
        uint16_t root_entry_count;
        /** @brief Sectors per FAT copy. */
        uint16_t fat_sector_count;
        /** @brief Number of FAT copies. */
        uint8_t fat_count;
        /** @brief Sectors in one allocation cluster; validated as a power of two. */
        uint8_t sectors_per_cluster;
        /** @brief Number of data clusters, validated to the FAT16 range. */
        uint16_t cluster_count;
        /** @brief True only after the MBR and FAT16 BPB passed validation. */
        bool mounted;
        /** @brief Reused for MBR, BPB, directory, FAT and file data sector reads. */
        uint8_t sector[AVR_FAT16_SECTOR_SIZE];
    } avr_fat16_volume_t;

    /**
     * @brief State for one open root-directory file.
     * @details One volume supports one open file at a time. The caller owns this small structure.
     */
    typedef struct
    {
        /** @brief Current FAT cluster being read. */
        uint16_t current_cluster;
        /** @brief Total file bytes declared in the directory entry. */
        uint32_t file_size;
        /** @brief Bytes remaining to return. */
        uint32_t bytes_remaining;
        /** @brief Offset within the current 512-byte sector. */
        uint16_t sector_offset;
        /** @brief Sector position within current cluster. */
        uint8_t sector_in_cluster;
        /** @brief True after the current data sector is loaded into the volume buffer. */
        bool sector_loaded;
        /** @brief True after a valid file entry has been opened. */
        bool open;
    } avr_fat16_file_t;

    /**
     * @brief Mount one FAT16 primary partition from a block device.
     * @param[out] volume Caller-owned volume state; the 512-byte sector buffer stays in this
     * object.
     * @param[in] read_sector Finite-timeout sector reader callback.
     * @param[in] device_context Opaque device pointer passed to read_sector.
     * @retval AVR_OK A primary MBR partition contains a structurally valid FAT16 volume.
     * @retval AVR_ERROR_ARGUMENT A required pointer is null.
     * @retval AVR_ERROR_UNSUPPORTED No supported FAT16 primary partition or BPB was found.
     * @retval AVR_ERROR_IO The MBR or BPB read failed or contained invalid bounds/signatures.
     * @details Supports partition types 0x04, 0x06 and 0x0E. Rejects FAT12/FAT32, super-floppy
     * layouts, extended partitions, malformed bounds and cluster counts outside the FAT16 range.
     */
    avr_status_t avr_fat16_mount(avr_fat16_volume_t *volume, avr_fat16_read_sector_fn read_sector,
                                 void *device_context);

    /**
     * @brief Open a root-directory file by its exact 11-byte uppercase 8.3 name.
     * @param[in] volume Mounted volume; only one file may be open at a time.
     * @param[out] file Caller-owned file state, written only after a valid entry is found.
     * @param[in] name_83 Exactly 8 base-name bytes followed by 3 extension bytes, space padded.
     * @retval AVR_OK A regular, nonempty or empty root-directory file was found.
     * @retval AVR_ERROR_ARGUMENT Invalid pointer or unmounted volume.
     * @retval AVR_ERROR_UNSUPPORTED File is not representable by the supported FAT16 subset.
     * @retval AVR_ERROR_IO A directory-sector read failed.
     * @retval AVR_ERROR_TIMEOUT or another device error returned by read_sector.
     * @details Skips deleted entries, long-file-name records, directories and volume labels. Does
     * not search subdirectories or interpret Unicode/long names.
     */
    avr_status_t avr_fat16_open_root(avr_fat16_volume_t *volume, avr_fat16_file_t *file,
                                     const uint8_t name_83[11]);

    /**
     * @brief Read up to capacity bytes from the current file position.
     * @param[in,out] volume Mounted volume with its persistent sector buffer.
     * @param[in,out] file Open file state.
     * @param[out] destination Caller-owned output bytes.
     * @param[in] capacity Maximum number of bytes to copy; zero is allowed.
     * @param[out] bytes_read Exact byte count copied before return, including a prefix on error.
     * @retval AVR_OK The requested count was copied or end-of-file was reached.
     * @retval AVR_ERROR_ARGUMENT Invalid state/pointers or a nonzero capacity with null
     * destination.
     * @retval AVR_ERROR_IO A corrupt FAT chain ended before the declared file size.
     * @retval AVR_ERROR_TIMEOUT or another device error returned by read_sector.
     * @details Reads sectors on demand into volume->sector. It never allocates a second sector
     * buffer, reads no bytes beyond the file size, and does not recurse or write to the card.
     */
    avr_status_t avr_fat16_read(avr_fat16_volume_t *volume, avr_fat16_file_t *file,
                                uint8_t *destination, size_t capacity, size_t *bytes_read);

#ifdef __cplusplus
}
#endif
#endif
