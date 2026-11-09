/**
 * @file fat16.c
 * @brief Small, allocation-free FAT16 root-directory reader.
 * @details Validates a primary MBR partition, the BIOS parameter block, FAT bounds and cluster
 * ranges before exposing an 8.3 file stream. It supports a single open file and one reusable
 * sector buffer; unsupported layouts return an explicit status.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "filesystems/fat16.h"
#include <string.h>

/** @brief MBR signature at byte 510. */
#define AVR_FAT16_MBR_SIGNATURE_OFFSET UINT16_C(510)
/** @brief FAT boot-sector signature at byte 510. */
#define AVR_FAT16_BOOT_SIGNATURE_OFFSET UINT16_C(510)
/** @brief Partition table starts at byte 446 and contains four 16-byte entries. */
#define AVR_FAT16_PARTITION_TABLE_OFFSET UINT16_C(446)
/** @brief FAT16 root directory entries are 32 bytes. */
#define AVR_FAT16_DIRECTORY_ENTRY_SIZE UINT8_C(32)
/** @brief FAT16 directory attribute: long-file-name record. */
#define AVR_FAT16_ATTRIBUTE_LFN UINT8_C(0x0F)
/** @brief FAT16 directory attribute: volume label. */
#define AVR_FAT16_ATTRIBUTE_VOLUME UINT8_C(0x08)
/** @brief FAT16 directory attribute: subdirectory. */
#define AVR_FAT16_ATTRIBUTE_DIRECTORY UINT8_C(0x10)
/** @brief FAT16 end-of-chain marker lower bound. */
#define AVR_FAT16_EOC_MIN UINT16_C(0xFFF8)
/** @brief FAT16 reserved cluster marker lower bound. */
#define AVR_FAT16_RESERVED_MIN UINT16_C(0xFFF0)
/** @brief Smallest legal FAT16 data-cluster count. */
#define AVR_FAT16_MIN_CLUSTERS UINT16_C(4085)
/** @brief Largest legal FAT16 data-cluster count. */
#define AVR_FAT16_MAX_CLUSTERS UINT16_C(65524)

/**
 * @brief Decode one little-endian 16-bit value from a sector.
 * @param[in] bytes Pointer to at least two bytes.
 * @return Unsigned 16-bit decoded value.
 */
static uint16_t avr_fat16_read_le16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
}

/**
 * @brief Decode one little-endian 32-bit value from a sector.
 * @param[in] bytes Pointer to at least four bytes.
 * @return Unsigned 32-bit decoded value.
 */
static uint32_t avr_fat16_read_le32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8U) | ((uint32_t)bytes[2] << 16U) |
           ((uint32_t)bytes[3] << 24U);
}

/**
 * @brief Write a candidate volume's logical block address into its reusable sector buffer.
 * @param[in,out] volume Mounted candidate and destination buffer.
 * @param[in] lba Whole-device logical block address.
 * @return The device callback status, or AVR_ERROR_IO when the read escapes the partition.
 */
static avr_status_t avr_fat16_read_block(avr_fat16_volume_t *volume, uint32_t lba)
{
    const uint32_t partition_end = volume->partition_start + volume->partition_sectors;
    if ((lba < volume->partition_start) || (lba >= partition_end))
    {
        return AVR_ERROR_IO;
    }
    return volume->read_sector(volume->device_context, lba, volume->sector);
}

/**
 * @brief Select and validate one supported FAT16 primary MBR entry.
 * @param[in,out] volume Callback state and sector buffer.
 * @param[out] partition_start Selected partition start LBA.
 * @param[out] partition_sectors Selected partition sector count.
 * @retval AVR_OK A supported type 0x04, 0x06 or 0x0E partition was found.
 * @retval AVR_ERROR_UNSUPPORTED No FAT16 primary partition is present.
 * @retval AVR_ERROR_IO MBR signature or partition bounds are invalid.
 * @details Chooses the first supported primary partition; extended and logical partitions are not
 * followed. The MBR is read from whole-device LBA 0.
 */
static avr_status_t avr_fat16_find_partition(avr_fat16_volume_t *volume, uint32_t *partition_start,
                                             uint32_t *partition_sectors)
{
    avr_status_t status = volume->read_sector(volume->device_context, 0U, volume->sector);
    if (status != AVR_OK)
    {
        return status;
    }
    if ((volume->sector[AVR_FAT16_MBR_SIGNATURE_OFFSET] != UINT8_C(0x55)) ||
        (volume->sector[AVR_FAT16_MBR_SIGNATURE_OFFSET + 1U] != UINT8_C(0xAA)))
    {
        return AVR_ERROR_IO;
    }
    for (uint8_t index = 0U; index < 4U; index++)
    {
        const uint16_t offset =
            (uint16_t)(AVR_FAT16_PARTITION_TABLE_OFFSET + ((uint16_t)index * UINT16_C(16)));
        const uint8_t type = volume->sector[offset + 4U];
        if ((type == UINT8_C(0x04)) || (type == UINT8_C(0x06)) || (type == UINT8_C(0x0E)))
        {
            const uint32_t start = avr_fat16_read_le32(&volume->sector[offset + 8U]);
            const uint32_t count = avr_fat16_read_le32(&volume->sector[offset + 12U]);
            if ((start == 0U) || (count == 0U) || (start > (UINT32_MAX - count)))
            {
                return AVR_ERROR_IO;
            }
            *partition_start = start;
            *partition_sectors = count;
            return AVR_OK;
        }
    }
    return AVR_ERROR_UNSUPPORTED;
}

/**
 * @brief Validate BPB fields and calculate the FAT, root and data-region starts.
 * @param[in,out] volume Candidate mount state; partition fields must already be valid.
 * @retval AVR_OK A bounded FAT16 volume was decoded.
 * @retval AVR_ERROR_UNSUPPORTED The BPB is not in the supported FAT16 subset.
 * @retval AVR_ERROR_IO The boot sector read failed or region arithmetic exceeds partition bounds.
 * @details Checks 512-byte sectors, power-of-two cluster size, FAT16 cluster count, partition size,
 * and enough FAT entries to represent every data cluster.
 */
static avr_status_t avr_fat16_read_bpb(avr_fat16_volume_t *volume)
{
    uint32_t total_sectors;
    uint32_t root_sectors;
    uint32_t overhead;
    uint32_t data_sectors;
    uint32_t clusters;
    uint32_t fat_entries;
    avr_status_t status = avr_fat16_read_block(volume, volume->partition_start);
    if (status != AVR_OK)
    {
        return status;
    }
    if ((volume->sector[AVR_FAT16_BOOT_SIGNATURE_OFFSET] != UINT8_C(0x55)) ||
        (volume->sector[AVR_FAT16_BOOT_SIGNATURE_OFFSET + 1U] != UINT8_C(0xAA)) ||
        (avr_fat16_read_le16(&volume->sector[11U]) != AVR_FAT16_SECTOR_SIZE))
    {
        return AVR_ERROR_UNSUPPORTED;
    }
    const uint8_t sectors_per_cluster = volume->sector[13U];
    const uint16_t reserved = avr_fat16_read_le16(&volume->sector[14U]);
    const uint8_t fat_count = volume->sector[16U];
    const uint16_t root_entries = avr_fat16_read_le16(&volume->sector[17U]);
    const uint16_t total16 = avr_fat16_read_le16(&volume->sector[19U]);
    const uint16_t fat_sectors = avr_fat16_read_le16(&volume->sector[22U]);
    total_sectors = (total16 != 0U) ? total16 : avr_fat16_read_le32(&volume->sector[32U]);
    if ((sectors_per_cluster == 0U) || (sectors_per_cluster > 128U) ||
        ((sectors_per_cluster & (uint8_t)(sectors_per_cluster - 1U)) != 0U) || (reserved == 0U) ||
        ((fat_count != 1U) && (fat_count != 2U)) || (root_entries == 0U) || (fat_sectors == 0U) ||
        (total_sectors == 0U) || (total_sectors > volume->partition_sectors))
    {
        return AVR_ERROR_UNSUPPORTED;
    }
    root_sectors =
        (((uint32_t)root_entries * AVR_FAT16_DIRECTORY_ENTRY_SIZE) + (AVR_FAT16_SECTOR_SIZE - 1U)) /
        AVR_FAT16_SECTOR_SIZE;
    overhead = (uint32_t)reserved + ((uint32_t)fat_count * fat_sectors) + root_sectors;
    if ((overhead >= total_sectors) || (volume->partition_start > (UINT32_MAX - total_sectors)))
    {
        return AVR_ERROR_IO;
    }
    data_sectors = total_sectors - overhead;
    clusters = data_sectors / sectors_per_cluster;
    fat_entries = ((uint32_t)fat_sectors * AVR_FAT16_SECTOR_SIZE) / 2U;
    if ((clusters < AVR_FAT16_MIN_CLUSTERS) || (clusters > AVR_FAT16_MAX_CLUSTERS) ||
        (fat_entries < (clusters + 2U)))
    {
        return AVR_ERROR_UNSUPPORTED;
    }
    volume->fat_start = volume->partition_start + reserved;
    volume->root_start = volume->fat_start + ((uint32_t)fat_count * fat_sectors);
    volume->data_start = volume->root_start + root_sectors;
    volume->root_entry_count = root_entries;
    volume->fat_sector_count = fat_sectors;
    volume->fat_count = fat_count;
    volume->sectors_per_cluster = sectors_per_cluster;
    volume->cluster_count = (uint16_t)clusters;
    return AVR_OK;
}

/** @see fat16.h for the public mount contract. */
avr_status_t avr_fat16_mount(avr_fat16_volume_t *volume, avr_fat16_read_sector_fn read_sector,
                             void *device_context)
{
    avr_status_t status;
    if ((volume == NULL) || (read_sector == NULL))
    {
        return AVR_ERROR_ARGUMENT;
    }
    (void)memset(volume, 0, sizeof(*volume));
    volume->read_sector = read_sector;
    volume->device_context = device_context;
    status = avr_fat16_find_partition(volume, &volume->partition_start, &volume->partition_sectors);
    if (status == AVR_OK)
    {
        status = avr_fat16_read_bpb(volume);
    }
    volume->mounted = (status == AVR_OK);
    return status;
}

/** @see fat16.h for the public root-directory lookup contract. */
avr_status_t avr_fat16_open_root(avr_fat16_volume_t *volume, avr_fat16_file_t *file,
                                 const uint8_t name_83[11])
{
    if ((volume == NULL) || (file == NULL) || (name_83 == NULL) || !volume->mounted)
    {
        return AVR_ERROR_ARGUMENT;
    }
    const uint32_t entries_per_sector = AVR_FAT16_SECTOR_SIZE / AVR_FAT16_DIRECTORY_ENTRY_SIZE;
    const uint32_t root_sectors =
        ((uint32_t)volume->root_entry_count + entries_per_sector - 1U) / entries_per_sector;
    uint32_t entry_number = 0U;
    for (uint32_t sector_index = 0U; sector_index < root_sectors; sector_index++)
    {
        const avr_status_t status = avr_fat16_read_block(volume, volume->root_start + sector_index);
        if (status != AVR_OK)
        {
            return status;
        }
        for (uint8_t slot = 0U;
             (slot < entries_per_sector) && (entry_number < volume->root_entry_count);
             slot++, entry_number++)
        {
            const uint16_t offset = (uint16_t)((uint16_t)slot * AVR_FAT16_DIRECTORY_ENTRY_SIZE);
            const uint8_t first = volume->sector[offset];
            const uint8_t attributes = volume->sector[offset + 11U];
            if (first == 0U)
            {
                return AVR_ERROR_UNSUPPORTED;
            }
            if ((first == UINT8_C(0xE5)) || (attributes == AVR_FAT16_ATTRIBUTE_LFN) ||
                ((attributes & (AVR_FAT16_ATTRIBUTE_DIRECTORY | AVR_FAT16_ATTRIBUTE_VOLUME)) != 0U))
            {
                continue;
            }
            if (memcmp(&volume->sector[offset], name_83, 11U) == 0)
            {
                const uint16_t cluster = avr_fat16_read_le16(&volume->sector[offset + 26U]);
                const uint32_t file_size = avr_fat16_read_le32(&volume->sector[offset + 28U]);
                const uint32_t maximum_file_bytes = (uint32_t)volume->cluster_count *
                                                    volume->sectors_per_cluster *
                                                    AVR_FAT16_SECTOR_SIZE;
                if ((file_size > maximum_file_bytes) ||
                    ((file_size != 0U) &&
                     ((cluster < 2U) || (cluster >= (volume->cluster_count + 2U)))))
                {
                    return AVR_ERROR_IO;
                }
                file->current_cluster = cluster;
                file->file_size = file_size;
                file->bytes_remaining = file_size;
                file->sector_offset = 0U;
                file->sector_in_cluster = 0U;
                file->sector_loaded = false;
                file->open = true;
                return AVR_OK;
            }
        }
    }
    return AVR_ERROR_UNSUPPORTED;
}

/**
 * @brief Follow one FAT16 cluster link while a file has bytes remaining.
 * @param[in,out] volume Volume state and shared sector buffer.
 * @param[in,out] file File state; current_cluster changes on success.
 * @retval AVR_OK A valid next data cluster was found.
 * @retval AVR_ERROR_IO The FAT sector failed to read or the file chain ended/was corrupt early.
 * @details FAT12/FAT32 markers, reserved clusters, free clusters and clusters outside the mounted
 * data range are rejected.
 */
static avr_status_t avr_fat16_advance_cluster(avr_fat16_volume_t *volume, avr_fat16_file_t *file)
{
    const uint32_t fat_byte_offset = (uint32_t)file->current_cluster * 2U;
    const uint32_t fat_sector_index = fat_byte_offset / AVR_FAT16_SECTOR_SIZE;
    const uint16_t entry_offset = (uint16_t)(fat_byte_offset % AVR_FAT16_SECTOR_SIZE);
    uint16_t next_cluster;
    avr_status_t status;
    if (fat_sector_index >= volume->fat_sector_count)
    {
        return AVR_ERROR_IO;
    }
    status = avr_fat16_read_block(volume, volume->fat_start + fat_sector_index);
    if (status != AVR_OK)
    {
        return status;
    }
    next_cluster = avr_fat16_read_le16(&volume->sector[entry_offset]);
    if ((next_cluster < 2U) || (next_cluster >= (volume->cluster_count + 2U)) ||
        (next_cluster >= AVR_FAT16_RESERVED_MIN))
    {
        return AVR_ERROR_IO;
    }
    file->current_cluster = next_cluster;
    file->sector_in_cluster = 0U;
    return AVR_OK;
}

/** @see fat16.h for the public streaming-read contract. */
avr_status_t avr_fat16_read(avr_fat16_volume_t *volume, avr_fat16_file_t *file,
                            uint8_t *destination, size_t capacity, size_t *bytes_read)
{
    size_t copied = 0U;
    if ((volume == NULL) || (file == NULL) || (bytes_read == NULL) || !volume->mounted ||
        !file->open || ((destination == NULL) && (capacity != 0U)))
    {
        return AVR_ERROR_ARGUMENT;
    }
    *bytes_read = 0U;
    while ((copied < capacity) && (file->bytes_remaining != 0U))
    {
        if (!file->sector_loaded)
        {
            const uint32_t cluster_offset =
                (uint32_t)(file->current_cluster - 2U) * volume->sectors_per_cluster;
            const uint32_t lba = volume->data_start + cluster_offset + file->sector_in_cluster;
            const avr_status_t status = avr_fat16_read_block(volume, lba);
            if (status != AVR_OK)
            {
                *bytes_read = copied;
                return status;
            }
            file->sector_loaded = true;
        }
        const uint16_t sector_available = (uint16_t)(AVR_FAT16_SECTOR_SIZE - file->sector_offset);
        const uint32_t remaining_available =
            (file->bytes_remaining < sector_available) ? file->bytes_remaining : sector_available;
        const size_t caller_available = capacity - copied;
        const size_t amount = (remaining_available < caller_available) ? (size_t)remaining_available
                                                                       : caller_available;
        (void)memcpy(&destination[copied], &volume->sector[file->sector_offset], amount);
        copied += amount;
        file->bytes_remaining -= (uint32_t)amount;
        file->sector_offset = (uint16_t)(file->sector_offset + amount);
        if ((file->sector_offset == AVR_FAT16_SECTOR_SIZE) && (file->bytes_remaining != 0U))
        {
            file->sector_offset = 0U;
            file->sector_loaded = false;
            file->sector_in_cluster++;
            if (file->sector_in_cluster == volume->sectors_per_cluster)
            {
                const avr_status_t status = avr_fat16_advance_cluster(volume, file);
                if (status != AVR_OK)
                {
                    *bytes_read = copied;
                    return status;
                }
            }
        }
    }
    *bytes_read = copied;
    return AVR_OK;
}
