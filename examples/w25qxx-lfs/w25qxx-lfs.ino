#include <SPI.h>
#include "W25QXX.h"
#include "fs.h"

w25qxx_interface_t interface = W25QXX_INTERFACE_SPI;
w25qxx_type_t chip_type = W25Q128;

/**
    Add -DLFS_YES_TRACE to build_opt.h for trace
    Add -DLFS_NO_MALLOC to build_opt.h for static LFS allocations
*/

/** Uncomment to  erase W25QXX chip */
#define ERASE_CHIP

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

static W25QXX_dev_hdl_t W25QXX_hdl;        /**< W25QXX handle struct */

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
    auto &info = W25QXX_hdl.chip_info;

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

    DBG("\n");


#if defined (ERASE_CHIP)
    Serial.println("Erasing...");
    res = W25QXX_chip_erase(&W25QXX_hdl);
    if (res != W25QXX_Ok) {
        W25QXX_Error_Handler();
    }    Serial.println("Done erasing chip");
#endif

    FS_Init(&W25QXX_hdl);

    FS_PrintStatus();

    FS_RunBenchmark();

    W25QXX_deinit(&W25QXX_hdl);
}

// the loop routine runs over and over again forever:
void loop()
{
    delay(100);
}

extern "C" void _putchar(char character)
{
    Serial.write(character);
}
