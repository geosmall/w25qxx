#include <SPI.h>
#include "w25qxx.h"

w25qxx_interface_t interface = W25QXX_INTERFACE_SPI;
w25qxx_type_t chip_type = W25Q128;
#define PAGE_SIZE 4096

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