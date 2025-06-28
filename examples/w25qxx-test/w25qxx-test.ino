#include <SPI.h>
#include "W25QXX.h"

w25qxx_interface_t interface = W25QXX_INTERFACE_SPI;
w25qxx_type_t chip_type = W25Q128;

#define DBG(...)    Serial.printf(__VA_ARGS__)
#define BLINK_FAST 50
#define BLINK_SLOW 1000

#define PAGE_SIZE 4096

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

static W25QXX_dev_hdl_t W25QXX_hdl;        /**< W25QXX handle */

// the setup routine runs once when you press reset:
void setup()
{
    CRC_HandleTypeDef hcrc;

    Serial.begin(115200);
    while (!Serial) delay(100); // wait until Serial/monitor is opened

    Serial.println("SPI Flash test...");

    pinMode(LED_BUILTIN, OUTPUT);

    LED_GPIO_Port = digitalPinToPort(LED_BUILTIN);
    LED_Pin = digitalPinToBitMask(LED_BUILTIN);

    // ensure the CS pin is pulled HIGH
    pinMode(CS_PIN, OUTPUT); digitalWrite(CS_PIN, HIGH);

    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        Error_Handler();
    }

    delay(10); // Wait a bit to make sure w25qxx chip is ready

    W25QXX_err_t res;

    /* W25QXX init */
    res = W25QXX_init(&W25QXX_hdl, chip_type);
    if (res != W25QXX_Ok) {
        W25QXX_Error_Handler();
    }

    // W25QXX_info_t info = W25QXX_hdl.chip_info;
    auto& info = W25QXX_hdl.chip_info;

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

    HAL_Delay(2000);

    uint8_t buf[PAGE_SIZE]; // Buffer the size of a page

    for (uint8_t run = 0; run <= 2; ++run) {

        DBG("\n-------------\nRun %d\n", run);

        DBG("Reading first page");

        res = W25QXX_read(&W25QXX_hdl, 0, (uint8_t *) &buf, sizeof(buf));
        if (res == W25QXX_Ok) {
            hex_dump("First page at start", 0, (uint8_t *) &buf, sizeof(buf));
        } else {
            DBG("Unable to read w25qxx\n");
        }

        DBG("Erasing first page");
        if (W25QXX_erase(&W25QXX_hdl, 0, sizeof(buf)) == W25QXX_Ok) {
            DBG("Reading first page\n");
            if (W25QXX_read(&W25QXX_hdl, 0, (uint8_t *) &buf, sizeof(buf)) == W25QXX_Ok) {
                hex_dump("After erase", 0, (uint8_t *) &buf, sizeof(buf));
            }
        }

        // Create a well known pattern
        fill_buffer(run, buf, sizeof(buf));

        // Write it to device
        DBG("Writing first page\n");
        if (W25QXX_write(&W25QXX_hdl, 0, (uint8_t *) &buf, sizeof(buf)) == W25QXX_Ok) {
            // now read it back
            DBG("Reading first page\n");
            if (W25QXX_read(&W25QXX_hdl, 0, (uint8_t *) &buf, sizeof(buf)) == W25QXX_Ok) {
                //DBG("  - sum = %lu", get_sum(buf, 256));
                hex_dump("After write", 0, (uint8_t *) &buf, sizeof(buf));
            }
        }
    }

    // Perform a stress test
    uint32_t start;
    uint32_t sectors = info.block_count * info.sectors_in_block; // Entire chip

    DBG("Stress testing w25qxx device: sectors = %lu\n", sectors);

    DBG("Doing chip erase\n");
    start = HAL_GetTick();
    W25QXX_chip_erase(&W25QXX_hdl);
    DBG("Done erasing - took %lu ms\n", HAL_GetTick() - start);

    fill_buffer(0, buf, sizeof(buf));

    DBG("Writing all zeroes %lu sectors\n", sectors);
    start = HAL_GetTick();
    for (uint32_t i = 0; i < sectors; ++i) {
        W25QXX_write(&W25QXX_hdl, i * info.sector_size, buf, sizeof(buf));
    }
    DBG("Done writing - took %lu ms\n", HAL_GetTick() - start);

    DBG("Reading %lu sectors\n", sectors);
    start = HAL_GetTick();
    for (uint32_t i = 0; i < sectors; ++i) {
        W25QXX_read(&W25QXX_hdl, i * info.sector_size, buf, sizeof(buf));
    }
    DBG("Done reading - took %lu ms\n", HAL_GetTick() - start);

    DBG("Validating buffer .... ");
    if (check_buffer(0, buf, sizeof(buf))) {
        DBG("OK\n");
    } else {
        DBG("Not OK\n");
    }

    DBG("Doing chip erase\n");
    start = HAL_GetTick();
    W25QXX_chip_erase(&W25QXX_hdl);
    DBG("Done erasing - took %lu ms\n", HAL_GetTick() - start);

    fill_buffer(1, buf, sizeof(buf));

    DBG("Writing 10101010 %lu sectors\n", sectors);
    start = HAL_GetTick();
    for (uint32_t i = 0; i < sectors; ++i) {
        W25QXX_write(&W25QXX_hdl, i * info.sector_size, buf, sizeof(buf));
    }
    DBG("Done writing - took %lu ms\n", HAL_GetTick() - start);

    DBG("Reading %lu sectors\n", sectors);
    start = HAL_GetTick();
    for (uint32_t i = 0; i < sectors; ++i) {
        W25QXX_read(&W25QXX_hdl, i * info.sector_size, buf, sizeof(buf));
    }
    DBG("Done reading - took %lu ms\n", HAL_GetTick() - start);

    DBG("Validating buffer ... ");
    if (check_buffer(1, buf, sizeof(buf))) {
        DBG("OK\n");
    } else {
        DBG("Not OK\n");
    }

    DBG("Erasing %lu sectors sequentially\n", sectors);
    start = HAL_GetTick();
    for (uint32_t i = 0; i < sectors; ++i) {
        W25QXX_erase(&W25QXX_hdl, i * info.sector_size, sizeof(buf));
        if ((i > 0) && (i % 100 == 0)) {
            DBG("Done %4lu sectors - total time = %3lu s\n", i, (HAL_GetTick() - start) / 1000);
        }
    }
    DBG("Done erasing - took %lu ms\n", HAL_GetTick() - start);

    uint32_t now = 0, last_blink = 0, last_test = 0, offset_address = 0;

    while (1) {

        now = HAL_GetTick();

        if (now - last_blink >= 500) {

            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

            last_blink = now;
        }

        if (now - last_test >= 1000) {

            DBG("---------------\nReading page at address     : 0x%08lx\n", offset_address);

            res = W25QXX_read(&W25QXX_hdl, offset_address, (uint8_t *) &buf, sizeof(buf));
            if (res == W25QXX_Ok) {
                //dump_hex("First page at start", offset_address, (uint8_t*) &buf, sizeof(buf));
                DBG("Reading old value           : 0x%08lx\n", HAL_CRC_Calculate(&hcrc, (uint32_t *)&buf, sizeof(buf) / 4));
            } else {
                DBG("Unable to read w25qxx\n");
            }

            if (W25QXX_erase(&W25QXX_hdl, offset_address, sizeof(buf)) == W25QXX_Ok) {
                if (W25QXX_read(&W25QXX_hdl, offset_address, (uint8_t *) &buf, sizeof(buf)) == W25QXX_Ok) {
                    DBG("After erase                 : 0x%08lx\n", HAL_CRC_Calculate(&hcrc, (uint32_t *)&buf, sizeof(buf) / 4));
                }
            }

            // Create a well known pattern
            fill_buffer(2, buf, sizeof(buf));

            // Write it to device
            DBG("Writing page value          : 0x%08lx\n", HAL_CRC_Calculate(&hcrc, (uint32_t *)&buf, sizeof(buf) / 4));
            if (W25QXX_write(&W25QXX_hdl, offset_address, (uint8_t *) &buf, sizeof(buf)) == W25QXX_Ok) {
                // now read it back
                //DBG("Reading page");
                if (W25QXX_read(&W25QXX_hdl, offset_address, (uint8_t *) &buf, sizeof(buf)) == W25QXX_Ok) {
                    DBG("Reading back                : 0x%08lx\n", HAL_CRC_Calculate(&hcrc, (uint32_t *)&buf, sizeof(buf) / 4));
                }
            }

            DBG("Test time                   : %lu ms\n", HAL_GetTick() - now);

            offset_address += PAGE_SIZE / 4;

            if (offset_address + PAGE_SIZE > info.block_count * info.block_size)
                offset_address = 0;

            last_test = now;
        }
    }

    W25QXX_deinit(&W25QXX_hdl);
}

// the loop routine runs over and over again forever:
void loop()
{
    delay(100);
}
