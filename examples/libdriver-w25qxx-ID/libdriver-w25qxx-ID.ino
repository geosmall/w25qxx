#include <SPI.h>
#include "W25QXX.h"

w25qxx_interface_t interface = W25QXX_INTERFACE_SPI;
w25qxx_type_t chip_type = W25Q128;

#define DBG(...)    Serial.printf(__VA_ARGS__)
#define BLINK_FAST 50
#define BLINK_SLOW 1000

void W25QXX_Error_Handler()
{
    asm("BKPT #0\n"); // break into the debugger
}

// #define DBG(...)    Serial.printf(__VA_ARGS__)

#if defined(ARDUINO_BLACKPILL_F411CE)
//              MOSI  MISO  SCLK
SPIClass SPIbus(PA7,  PA6,  PA5);
#define CS_PIN PA4
#else
//              MOSI  MISO  SCLK
SPIClass SPIbus(PC12, PC11, PC10);
#define CS_PIN PD2
#endif

/**
 * @brief global var definition
 */
uint8_t g_buf[256];                        /**< uart buffer */
volatile uint16_t g_len;                   /**< uart buffer length */

GPIO_TypeDef *LED_GPIO_Port = digitalPinToPort(LED_BUILTIN);
uint16_t LED_Pin = digitalPinToBitMask(LED_BUILTIN);

GPIO_TypeDef *SPI_CS_GPIO_Port;
uint16_t SPI_CS_Pin;

SPI_HandleTypeDef *hspi = nullptr;

static w25qxx_handle_t gs_handle;        /**< w25qxx handle */

// the setup routine runs once when you press reset:
void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(100); // wait until Serial/monitor is opened

    Serial.println("SPI Flash test...");

    pinMode(LED_BUILTIN, OUTPUT);

    LED_GPIO_Port = digitalPinToPort(LED_BUILTIN);
    LED_Pin = digitalPinToBitMask(LED_BUILTIN);

    // ensure the CS pin is pulled HIGH
    pinMode(CS_PIN, OUTPUT); digitalWrite(CS_PIN, HIGH);

    SPI_CS_GPIO_Port = digitalPinToPort(CS_PIN);
    SPI_CS_Pin = digitalPinToBitMask(CS_PIN);

    delay(10); // Wait a bit to make sure w25qxx chip is ready

    uint8_t res;
    uint8_t manufacturer;
    uint8_t device_id[2];

    /* advance init */
    res = w25qxx_example_init(chip_type, interface, W25QXX_BOOL_FALSE);
    if (res != 0) {
        W25QXX_Error_Handler();
    }

    res = w25qxx_example_get_jedec_id((uint8_t *)manufacturer, device_id);
    if (res != 0) {
        W25QXX_Error_Handler();
    } else {
        w25qxx_interface_debug_print("w25qxx: manufacturer is 0x%02X JEDEC id is 0x%04X.\n",
                                     manufacturer, (device_id[0] << 8) | device_id[1]);
    }

    uint32_t size_bytes, blocks;
    if (w25qxx_probe_geometry_sfdp(&size_bytes, &blocks) == 0) {
        w25qxx_interface_debug_print("Per SFDP: Flash size: %lu bytes, %lu blocks\r\n",
                                     (unsigned long)size_bytes,
                                     (unsigned long)blocks);
    } else {
        w25qxx_interface_debug_print("SFDP probe failed\r\n");
    }

    W25QXX_info_t info;
    w25qxx_populate_info(&info);

    if (res == W25QXX_Ok) {
        DBG("W25QXX successfully initialized\n");
        DBG("Manufacturer       = 0x%2x\n", info.manufacturer_id);
        DBG("JEDEC Device       = 0x%4x\n", info.jedec_id);
        DBG("Block size         = 0x%04lx (%lu)\n", info.block_size, info.block_size);
        DBG("Block count        = 0x%04lx (%lu)\n", info.block_count, info.block_count);
        DBG("Sector size        = 0x%04lx (%lu)\n", info.sector_size, info.sector_size);
        DBG("Sectors per block  = 0x%04lx (%lu)\n", info.sectors_in_block, info.sectors_in_block);
        DBG("Page size          = 0x%04lx (%lu)\n", info.page_size, info.page_size);
        DBG("Pages per sector   = 0x%04lx (%lu)\n", info.pages_in_sector, info.pages_in_sector);
        DBG("Total size (in kB) = 0x%04lx (%lu)\n", (info.block_count * info.block_size) / 1024, (info.block_count * info.block_size) / 1024);
    } else {
        DBG("Unable to initialize w25qxx\n");
        Error_Handler();
    }

    w25qxx_example_deinit();
}

// the loop routine runs over and over again forever:
void loop()
{
    delay(100);
}

/**
 * @brief     advance example init
 * @param[in] type chip type
 * @param[in] interface chip interface
 * @param[in] dual_quad_spi_enable bool value
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t w25qxx_example_init(w25qxx_type_t type, w25qxx_interface_t interface, w25qxx_bool_t dual_quad_spi_enable)
{
    uint8_t res;

    /* link interface function */
    DRIVER_W25QXX_LINK_INIT(&gs_handle, w25qxx_handle_t);
    DRIVER_W25QXX_LINK_SPI_QSPI_INIT(&gs_handle, w25qxx_interface_spi_qspi_init);
    DRIVER_W25QXX_LINK_SPI_QSPI_DEINIT(&gs_handle, w25qxx_interface_spi_qspi_deinit);
    DRIVER_W25QXX_LINK_SPI_QSPI_WRITE_READ(&gs_handle, w25qxx_interface_spi_qspi_write_read);
    DRIVER_W25QXX_LINK_DELAY_MS(&gs_handle, w25qxx_interface_delay_ms);
    DRIVER_W25QXX_LINK_DELAY_US(&gs_handle, w25qxx_interface_delay_us);
    DRIVER_W25QXX_LINK_DEBUG_PRINT(&gs_handle, w25qxx_interface_debug_print);

    /* set chip type */
    res = w25qxx_set_type(&gs_handle, type);
    if (res != 0) {
        w25qxx_interface_debug_print("w25qxx: set type failed.\n");

        return 1;
    }

    /* set chip interface */
    res = w25qxx_set_interface(&gs_handle, interface);
    if (res != 0) {
        w25qxx_interface_debug_print("w25qxx: set interface failed.\n");

        return 1;
    }

    /* set dual quad spi */
    res = w25qxx_set_dual_quad_spi(&gs_handle, dual_quad_spi_enable);
    if (res != 0) {
        w25qxx_interface_debug_print("w25qxx: set dual quad spi failed.\n");
        (void)w25qxx_deinit(&gs_handle);

        return 1;
    }

    /* chip init */
    res = w25qxx_init(&gs_handle);
    if (res != 0) {
        w25qxx_interface_debug_print("w25qxx: init failed.\n");

        return 1;
    } else {
        if (type >= W25Q256) {
            res = w25qxx_set_address_mode(&gs_handle, W25QXX_ADDRESS_MODE_4_BYTE);
            if (res != 0) {
                w25qxx_interface_debug_print("w25qxx: set address mode failed.\n");
                (void)w25qxx_deinit(&gs_handle);

                return 1;
            }
        }

        return 0;
    }
}

/**
 * @brief  advance example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t w25qxx_example_deinit(void)
{
    if (w25qxx_deinit(&gs_handle) != 0) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief      advance example get the jedec id information
 * @param[out] *manufacturer pointer to a manufacturer buffer
 * @param[out] *device_id pointer to a device id buffer
 * @return     status code
 *             - 0 success
 *             - 1 get jedec id failed
 * @note       none
 */
uint8_t w25qxx_example_get_jedec_id(uint8_t *manufacturer, uint8_t device_id[2])
{
    if (w25qxx_get_jedec_id(&gs_handle, manufacturer, device_id) != 0) {
        return 1;
    } else {
        return 0;
    }
}

static uint32_t le32(const uint8_t *p)   /* helper: 4-byte little-endian */
{
    return ((uint32_t)p[3] << 24) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[1] <<  8) | (uint32_t)p[0];
}
/**
 * @brief  Probes the flash geometry using SFDP.
 * @param  *flash_bytes  Pointer to a variable to receive the flash size in bytes.
 * @param  *block_cnt    Pointer to a variable to receive the number of 64 KiB blocks.
 * @retval 0             Success
 * @retval 1             SFDP signature not found
 * @retval 2             Basic Flash Parameter table not found
 * @note   Uses w25qxx_get_sfdp() to read the 256-byte SFDP data.
 *         (JESD216 spec §6.4, 9 DWORDs).  Works with all Winbond W25Q parts
 */
uint8_t w25qxx_probe_geometry_sfdp(uint32_t *flash_bytes, uint32_t *block_cnt)
{
    uint8_t buf[256];
    uint8_t res = w25qxx_get_sfdp(&gs_handle, buf); /* 0x5A read */
    if (res) return res;

    /* 1. Check the “SFDP” signature (0x53 46 44 50) */
    if (memcmp(buf, "SFDP", 4) != 0) {
        w25qxx_interface_debug_print("SFDP: bad signature\r\n");
        return 1;
    }

    /* 2. Grab the first parameter header (always Basic Flash) */
    const uint8_t *hdr = &buf[8];                  /* header #0, §6.3 */

    /* Offset 4-6 = 24-bit Parameter-Table Pointer (little-endian) */
    uint32_t ptp = (uint32_t)hdr[4]        |       /* bits  7-0  */
                   ((uint32_t)hdr[5] <<  8) |      /* bits 15-8  */
                   ((uint32_t)hdr[6] << 16);       /* bits 23-16 */

    if (ptp + 8 > sizeof buf) {                    /* need at least DWORD1 */
        w25qxx_interface_debug_print("SFDP: Basic table outside 256-B window\r\n");
        return 2;
    }

    /* 3. DWORD1 (offset +4 inside Basic table) holds density info */
    uint32_t dword1 = le32(&buf[ptp + 4]);

    uint64_t total_bits;
    if (dword1 & 0x80000000UL)                     /* “big-density” encoding */
        total_bits = 1ULL << (dword1 & 0x7FFFFFFFUL);
    else
        total_bits = (uint64_t)dword1 + 1ULL;      /* bits-1 → bits */

    uint64_t bytes = total_bits >> 3;              /* /8 → bytes */
    *flash_bytes = (uint32_t)bytes;                /* Winbond ≤ 256 MBytes */
    *block_cnt   = (uint32_t)(bytes / 65536UL);    /* 64 KiB blocks */

    return 0;
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

    return 0UL;                                /* unknown */
}

/**
 * @brief  Fill a W25QXX_handle_t struct with live data from the flash.
 * @param  info     Pointer to the struct to be filled.
 * @retval 0        Success
 * @retval non-zero Driver-level error (SPI fault, unsupported device, …)
 */
uint8_t w25qxx_populate_info(W25QXX_info_t *info)
{
    uint8_t  m_id;
    uint8_t  d_id;

    /* ----------- manufacturer & device ID ------------- */
    uint8_t res = w25qxx_get_manufacturer_device_id(&gs_handle, &m_id, &d_id);
    if (res) return res;

    uint32_t flash_bytes  = 0;
    uint32_t block_count  = 0;

    uint8_t jedec_id[2];

    res = w25qxx_example_get_jedec_id(&m_id, jedec_id);
    if (res) return res;

    res = w25qxx_probe_geometry_sfdp(&flash_bytes, &block_count);

    if (res) {                                           /* SFDP failed? */
        block_count = fallback_block_count(d_id);        /* use ID-based map */
        if (block_count == 0) return res;                /* unknown part */
        flash_bytes = block_count * W25Q_BLOCK_SIZE_BYTES;
    }

    /* ---------- populate the caller’s struct ---------- */
    memset(info, 0, sizeof(*info));

    info->manufacturer_id   = m_id;
    info->jedec_id          = (jedec_id[0] << 8) | jedec_id[1];;
    info->block_size        = W25Q_BLOCK_SIZE_BYTES;
    info->block_count       = block_count;
    info->sector_size       = W25Q_SECTOR_SIZE_BYTES;
    info->sectors_in_block  = W25Q_BLOCK_SIZE_BYTES / W25Q_SECTOR_SIZE_BYTES; /* 16 */
    info->page_size         = W25Q_PAGE_SIZE_BYTES;
    info->pages_in_sector   = W25Q_SECTOR_SIZE_BYTES / W25Q_PAGE_SIZE_BYTES;   /* 16 */

    return 0;
}