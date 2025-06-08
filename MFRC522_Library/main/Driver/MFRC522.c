#include "MFRC522.h"
#include "esp_err.h"
#include "driver/gpio.h"

Mfrc522_t *device_m = NULL;
static char *TAG = "MFRC522_DRIVER";

/// @brief This initialized the driver of the device.
/// @param sda SPI chip select pin (CS/SS/SSEL)
/// @param reset Not reset and power-down pin.
/// @return ESP_OK Success - ESP_ERR_INVALID_ARG GPIO number error
esp_err_t Initialize(spi_host_device_t host, gpio_num_t sda_pin, gpio_num_t reset_pin, gpio_num_t mosi, gpio_num_t miso, gpio_num_t sclk)
{

    device_m = (Mfrc522_t *)malloc(sizeof(Mfrc522_t));
    if (device_m == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate space for the device struct exiting..\n");
        return ESP_FAIL;
    }

    device_m->reset_pin = reset_pin;
    device_m->sda_pin = sda_pin;
    device_m->spi_host = host;
    // SPI Bus config (on stack, no need to malloc)
    spi_bus_config_t busConfiguration = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sclk,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1};

    ESP_ERROR_CHECK(spi_bus_initialize(device_m->spi_host, &busConfiguration, SPI_DMA_CH_AUTO));

    // SPI Device config
    spi_device_interface_config_t spiDeviceConfiguration_st =
        {
            .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
            .mode = 0,
            .spics_io_num = -1, // CS handled manually
            .queue_size = 1,
            .flags = 0};
    ESP_ERROR_CHECK(spi_bus_add_device(device_m->spi_host, &spiDeviceConfiguration_st, &device_m->spi_device));

    // Configure SDA pin
    gpio_config_t sdaConfiguartion_st =
        {
            .pin_bit_mask = 1ULL << device_m->sda_pin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&sdaConfiguartion_st);
    gpio_set_level(device_m->sda_pin, 1); // Deselect (HIGH)

    // Configure Reset pin
    gpio_config_t resetConfiguration_st = {
        .pin_bit_mask = 1ULL << device_m->reset_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&resetConfiguration_st);
    gpio_set_level(device_m->reset_pin, 1); // Bring out of reset (HIGH)

    return ESP_OK;
}

/**************************************************************************/
/*!

  @brief   Writes value to a register.

  @param   addr  The address a register.
  @param   val   The value to write to a register.

 */
/**************************************************************************/
esp_err_t WriteToRegister(uint8_t addr, uint8_t val)
{
    if (device_m == NULL)
    {
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(gpio_set_level(device_m->sda_pin, 0));
    // Address format: 0XXXXXX0
    uint8_t tx_data[2];
    tx_data[0] = (addr << 1) & 0x7E; // Format: 0XXXXXX0
    tx_data[1] = val;

    spi_transaction_t trans =
        {
            .length = 8 * sizeof(tx_data),
            .tx_buffer = tx_data,
            .rx_buffer = NULL,
        };
    gpio_set_level(device_m->sda_pin, 0); // CS LOW
    esp_err_t ret = spi_device_transmit(device_m->spi_device, &trans);
    gpio_set_level(device_m->sda_pin, 1); // CS HIGH
    return ret;
}

/**************************************************************************/
/*!

  @brief   Reads the value at a register.

  @param   addr  The address a register.

  @returns The byte at the register.

 */
/**************************************************************************/
uint8_t ReadFromRegister(uint8_t addr)
{
    if (device_m == NULL)
        return 0;

    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = ((addr << 1) & 0x7E) | 0x80; // Read format: 1XXXXXX0
    tx_data[1] = 0x00;                        // Dummy byte to clock in response

    spi_transaction_t transcation =
        {
            .length = 8 * sizeof(tx_data), // Total bits
            .tx_buffer = tx_data,
            .rx_buffer = rx_data};

    // CS LOW
    gpio_set_level(device_m->sda_pin, 0);

    gpio_set_level(device_m->sda_pin, 0); // CS LOW
    spi_device_transmit(device_m->spi_device, &transcation);
    gpio_set_level(device_m->sda_pin, 1); // CS HIGH
    return rx_data[1];                    // Second byte is the response
}

/**************************************************************************/
/*!

  @brief   Adds a bitmask to a register.

  @param   addr   The address a register.
  @param   mask  The mask to update the register with.

 */
/**************************************************************************/
void SetBitMask(uint8_t addr, uint8_t mask)
{
    uint8_t current;
    current = ReadFromRegister(addr);
    if (current == 0)
    {
        ESP_LOGE(TAG, "Failed to read from register\n");
    }
    ESP_ERROR_CHECK(WriteToRegister(addr, current | mask));
}
