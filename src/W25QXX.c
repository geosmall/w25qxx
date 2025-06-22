#include "W25QXX.h"

// Portable little-endian reads from unaligned memory
// These work regardless of host endianness and handle unaligned access safely
#define READ_LE32_UNALIGNED(ptr) (((uint32_t)(ptr)[0]) | \
                                  ((uint32_t)(ptr)[1] << 8) | \
                                  ((uint32_t)(ptr)[2] << 16) | \
                                  ((uint32_t)(ptr)[3] << 24))

#define READ_LE24_UNALIGNED(ptr) (((uint32_t)(ptr)[0]) | \
                                  ((uint32_t)(ptr)[1] << 8) | \
                                  ((uint32_t)(ptr)[2] << 16))

/* ------------------Start local helpers ------------------------ */


/**
 * @brief  Probes the flash geometry using SFDP.
 * @param  *flash_bytes  Pointer to a variable to receive the flash size in bytes.
 * @param  *block_cnt    Pointer to a variable to receive the number of 64 KiB blocks.
 * @return status code:  W25QXX_Err if SFDP read fails or signature is invalid, else W25QXX_Ok.
 * @note   Uses w25qxx_get_sfdp() to read the 256-byte SFDP data.
 *         (JESD216 spec §6.4, 9 DWORDs).  Works with all Winbond W25Q parts
 */
static uint8_t w25qxx_probe_geometry_sfdp(W25QXX_handle_t *w25qxx, uint32_t *flash_bytes, uint32_t *block_cnt)
{
    uint8_t buf[256];
    uint8_t res = w25qxx_get_sfdp(&w25qxx->w25qxx_hdl, buf); /* 0x5A read */
    if (res) return W25QXX_Err; /* SFDP read failed */

    /* 1. Check the “SFDP” signature (0x53 46 44 50) */
    if (memcmp(buf, "SFDP", 4) != 0) {
        w25qxx_interface_debug_print("SFDP: bad signature\r\n");
        return W25QXX_Err;
    }    /* 2. Grab the first parameter header (always Basic Flash) */
    const uint8_t *hdr = &buf[8];                  /* header #0, §6.3 */

    /* Offset 4-6 = 24-bit Parameter-Table Pointer (little-endian) */
    uint32_t ptp = READ_LE24_UNALIGNED(&hdr[4]);   /* read 24-bit little-endian pointer */

    if (ptp + 8 > sizeof buf) {                    /* need at least DWORD1 */
        w25qxx_interface_debug_print("SFDP: Basic table outside 256-B window\r\n");
        return W25QXX_Err;
    }

    /* 3. DWORD1 (offset +4 inside Basic table) holds density info */
    uint32_t dword1 = READ_LE32_UNALIGNED(&buf[ptp + 4]);   /* read little-endian DWORD */

    uint64_t total_bits;
    if (dword1 & 0x80000000UL)                     /* “big-density” encoding */
        total_bits = 1ULL << (dword1 & 0x7FFFFFFFUL);
    else
        total_bits = (uint64_t)dword1 + 1ULL;      /* bits-1 → bits */

    uint64_t bytes = total_bits >> 3;              /* /8 → bytes */
    *flash_bytes = (uint32_t)bytes;                /* Winbond ≤ 256 MBytes */
    *block_cnt   = (uint32_t)(bytes / 65536UL);    /* 64 KiB blocks */

    return W25QXX_Ok;
}

/**
 * @brief Fallback geometry map based on Device ID.
 * @param dev_id    Device ID (low byte of the 16-bit ID read by w25qxx_get_manufacturer_device_id()).
 * @return          Number of 64 KiB blocks, or 0 if unknown part.
 */
static uint32_t fallback_block_count(uint16_t dev_id)
{
    /* Device-ID low byte encodes density monotonically (0x13 → 8 Mbit … 0x21 → 2 Gbit) */
    uint8_t code = (uint8_t)dev_id;

    if (code >= 0x13 && code <= 0x19)          /* 8 Mbit … 512 Mbit */
        return 16UL << (code - 0x13);          /* 16,32,64,128,256,512,1024 */

    if (code == 0x20) return 2048UL;           /* W25Q01 */
    if (code == 0x21) return 4096UL;           /* W25Q02 */

    return W25QXX_Ok;                          /* unknown */
}

/**
 * @brief Get device geometry from either SFDP or fallback ID mapping
 * @param[in]  *w25qxx pointer to W25QXX handle structure
 * @param[out] *flash_bytes total flash size in bytes
 * @param[out] *block_count number of 64KB blocks
 * @return status code: 0=success, non-zero=error
 */
static uint8_t w25qxx_get_device_geometry(W25QXX_handle_t *w25qxx, uint32_t *flash_bytes, uint32_t *block_count)
{
    uint8_t manufacturer_id, device_id;

    // Get device IDs first for fallback
    uint8_t res = w25qxx_get_manufacturer_device_id(&w25qxx->w25qxx_hdl, &manufacturer_id, &device_id);
    if (res != 0) {
        return W25QXX_Err;
    }

    // Try SFDP first for accurate geometry
    res = w25qxx_probe_geometry_sfdp(w25qxx, flash_bytes, block_count);
    if (res == 0) {
        return W25QXX_Ok; // SFDP succeeded
    }

    // SFDP failed, use fallback mapping
    *block_count = fallback_block_count(device_id);
    if (*block_count == 0) {
        return W25QXX_Err; // Unknown device
    }

    *flash_bytes = (*block_count) * W25Q_BLOCK_SIZE_BYTES;
    return W25QXX_Ok;
}

/**
 * @brief  Fill a W25QXX_handle_t struct with live data from the flash.
 * @param[in]  *w25qxx pointer to W25QXX handle structure
 * @retval 0        Success
 * @retval non-zero Driver-level error (SPI fault, unsupported device, …)
 */
static uint8_t w25qxx_populate_info(W25QXX_handle_t *w25qxx)
{
    uint8_t manufacturer_id, device_id;
    uint8_t jedec_id[2];
    uint32_t flash_bytes, block_count;

    // Get manufacturer & device ID
    uint8_t res = w25qxx_get_manufacturer_device_id(&w25qxx->w25qxx_hdl, &manufacturer_id, &device_id);
    if (res != 0) {
        return W25QXX_Err;
    }

    // Get JEDEC ID
    res = w25qxx_get_jedec_id(&w25qxx->w25qxx_hdl, &manufacturer_id, jedec_id);
    if (res != 0) {
        return W25QXX_Err;
    }

    // Get device geometry (size info)
    res = w25qxx_get_device_geometry(w25qxx, &flash_bytes, &block_count);
    if (res != 0) {
        return W25QXX_Err;
    }

    // Populate the chip info structure
    memset(&w25qxx->chip_info, 0, sizeof(w25qxx->chip_info));

    w25qxx->chip_info.manufacturer_id   = manufacturer_id;
    w25qxx->chip_info.jedec_id          = (jedec_id[0] << 8) | jedec_id[1];
    w25qxx->chip_info.block_size        = W25Q_BLOCK_SIZE_BYTES;
    w25qxx->chip_info.block_count       = block_count;
    w25qxx->chip_info.sector_size       = W25Q_SECTOR_SIZE_BYTES;
    w25qxx->chip_info.sectors_in_block  = W25Q_BLOCK_SIZE_BYTES / W25Q_SECTOR_SIZE_BYTES; /* 16 */
    w25qxx->chip_info.page_size         = W25Q_PAGE_SIZE_BYTES;
    w25qxx->chip_info.pages_in_sector   = W25Q_SECTOR_SIZE_BYTES / W25Q_PAGE_SIZE_BYTES;   /* 16 */

    return W25QXX_Ok;
}

static inline uint32_t next_sector_boundary(uint32_t addr)
{
    return (addr + W25Q_SECTOR_SIZE_BYTES) & ~(W25Q_SECTOR_SIZE_BYTES - 1U);
}

/* --------------------End local helpers ------------------------ */

W25QXX_err_t W25QXX_init(W25QXX_handle_t *w25qxx, w25qxx_type_t type)
{
    DRIVER_W25QXX_LINK_INIT(&w25qxx->w25qxx_hdl, w25qxx_handle_t);
    DRIVER_W25QXX_LINK_SPI_QSPI_INIT(&w25qxx->w25qxx_hdl, w25qxx_interface_spi_qspi_init);
    DRIVER_W25QXX_LINK_SPI_QSPI_DEINIT(&w25qxx->w25qxx_hdl, w25qxx_interface_spi_qspi_deinit);
    DRIVER_W25QXX_LINK_SPI_QSPI_WRITE_READ(&w25qxx->w25qxx_hdl, w25qxx_interface_spi_qspi_write_read);
    DRIVER_W25QXX_LINK_DELAY_MS(&w25qxx->w25qxx_hdl, w25qxx_interface_delay_ms);
    DRIVER_W25QXX_LINK_DELAY_US(&w25qxx->w25qxx_hdl, w25qxx_interface_delay_us);
    DRIVER_W25QXX_LINK_DEBUG_PRINT(&w25qxx->w25qxx_hdl, w25qxx_interface_debug_print);

    uint8_t res = w25qxx_set_type(&w25qxx->w25qxx_hdl, type);
    if (res != 0) {
        w25qxx_interface_debug_print("w25qxx: set type failed.\n");

        return W25QXX_Err;
    }

    /* set chip interface */
    res = w25qxx_set_interface(&w25qxx->w25qxx_hdl, W25QXX_INTERFACE_SPI);
    if (res != 0) {
        w25qxx_interface_debug_print("w25qxx: set interface failed.\n");

        return W25QXX_Err;
    }

    /* set dual quad spi */
    res = w25qxx_set_dual_quad_spi(&w25qxx->w25qxx_hdl, W25QXX_BOOL_FALSE);
    if (res != 0) {
        w25qxx_interface_debug_print("w25qxx: set dual quad spi failed.\n");
        (void)w25qxx_deinit(w25qxx);
        return W25QXX_Err;
    }    /* chip init */
    res = w25qxx_init(&w25qxx->w25qxx_hdl);
    if (res != 0) {
        w25qxx_interface_debug_print("w25qxx: init failed.\n");
        return W25QXX_Err;
    } else {
        if (type >= W25Q256) {
            res = w25qxx_set_address_mode(&w25qxx->w25qxx_hdl, W25QXX_ADDRESS_MODE_4_BYTE);
            if (res != 0) {
                w25qxx_interface_debug_print("w25qxx: set address mode failed.\n");
                (void)w25qxx_deinit(&w25qxx->w25qxx_hdl);
                return W25QXX_Err;
            }
        }

        // Populate chip information after successful initialization
        res = w25qxx_populate_info(w25qxx);
        if (res != 0) {
            w25qxx_interface_debug_print("w25qxx: populate info failed.\n");
            (void)w25qxx_deinit(&w25qxx->w25qxx_hdl);
            return W25QXX_Err;
        }

        return W25QXX_Ok;
    }
}

W25QXX_err_t W25QXX_deinit(W25QXX_handle_t *w25qxx)
{
    if (w25qxx_deinit(&w25qxx->w25qxx_hdl) != 0) {
        return W25QXX_Err;
    }
    return W25QXX_Ok;
}

W25QXX_err_t W25QXX_read(W25QXX_handle_t *w25qxx, uint32_t address, uint8_t *buf, uint32_t len)
{
    if (w25qxx_read(&w25qxx->w25qxx_hdl, address, buf, len) != 0) {
        return W25QXX_Err;
    }
    return W25QXX_Ok;
}

W25QXX_err_t W25QXX_write(W25QXX_handle_t *w25qxx, uint32_t address, uint8_t *buf, uint32_t len)
{
    if (w25qxx_write(&w25qxx->w25qxx_hdl, address, buf, len) != 0) {
        return W25QXX_Err;
    }
    return W25QXX_Ok;
}

W25QXX_err_t W25QXX_erase(W25QXX_handle_t *w25qxx, uint32_t address, uint32_t len)
{
    W25QXX_err_t rc;

    uint32_t flash_bytes = w25qxx->chip_info.block_size * w25qxx->chip_info.block_count;

    if (len == 0 || address >= flash_bytes || (flash_bytes - address) <  len) {
        return W25QXX_Err;
    }

    while (len) {
        uint32_t sector_addr = address & ~(W25Q_SECTOR_SIZE_BYTES - 1U);

        rc = w25qxx_sector_erase_4k(&w25qxx->w25qxx_hdl, sector_addr);
        if (rc) return W25QXX_Err;                      /* abort on first error */

        /* advance to next portion */
        uint32_t bytes_erased = next_sector_boundary(address) - address;
        if (bytes_erased > len) bytes_erased = len;     /* last partial sector */

        address += bytes_erased;
        len     -= bytes_erased;
    }

    return W25QXX_Ok;
}


W25QXX_err_t W25QXX_chip_erase(W25QXX_handle_t *w25qxx)
{
    if (w25qxx_chip_erase(&w25qxx->w25qxx_hdl) != 0) {
        return W25QXX_Err;
    }
    return W25QXX_Ok;
}