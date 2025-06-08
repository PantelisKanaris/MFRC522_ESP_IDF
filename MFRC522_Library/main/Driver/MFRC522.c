#include "MFRC522.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

  @returns The uint8_t at the register.

 */
/**************************************************************************/
uint8_t ReadFromRegister(uint8_t addr)
{
    if (device_m == NULL)
        return 0;

    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = ((addr << 1) & 0x7E) | 0x80; // Read format: 1XXXXXX0
    tx_data[1] = 0x00;                        // Dummy uint8_t to clock in response

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
    return rx_data[1];                    // Second uint8_t is the response
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

/**************************************************************************/
/*!

  @brief   Removes a bitmask from the register.

  @param   reg   The address a register.
  @param   mask  The mask to update the register with.

 */
/**************************************************************************/
void ClearBitMask(uint8_t addr, uint8_t mask)
{
    uint8_t current;
    current = ReadFromRegister(addr);
    if (current == 0)
    {
        ESP_LOGE(TAG, "Failed to read from register\n");
    }
    ESP_ERROR_CHECK(WriteToRegister(addr, current & (~mask)));
}

/// @brief Softs resets the device.
void Reset()
{
    WriteToRegister(CommandReg, MFRC522_SOFTRESET);
}

/**************************************************************************/
/*!

  @brief   Does the setup for the MFRC522.

 */
/**************************************************************************/
void Begin()
{
    gpio_set_level(device_m->sda_pin, 1);

    Reset();

    // Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    WriteToRegister(TModeReg, 0x8D);      // Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    WriteToRegister(TPrescalerReg, 0x3E); // TModeReg[3..0] + TPrescalerReg
    WriteToRegister(TReloadRegL, 30);
    WriteToRegister(TReloadRegH, 0);

    WriteToRegister(TxAutoReg, 0x40); // 100%ASK
    WriteToRegister(ModeReg, 0x3D);   // CRC initial value 0x6363

    setBitMask(TxControlReg, 0x03); // Turn antenna on.
}

/**************************************************************************/
/*!

  @brief   Checks the firmware version of the chip.

  @returns The firmware version of the MFRC522 chip.

 */
/**************************************************************************/
uint8_t GetFirmwareVersion()
{
    uint8_t response;
    response = ReadFromRegister(VersionReg);
    return response;
}

/**************************************************************************/
/*!

  @brief   Runs the digital self test.

  @returns True if the self test passes, false otherwise.

 */
/**************************************************************************/
bool DigitalSelfTestPass()
{
    int i;
    uint8_t n;

    uint8_t selfTestResultV1[] = {0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
                                  0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
                                  0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
                                  0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
                                  0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
                                  0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
                                  0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
                                  0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79};
    uint8_t selfTestResultV2[] = {0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
                                  0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
                                  0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
                                  0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
                                  0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
                                  0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
                                  0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
                                  0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F};
    uint8_t *selfTestResult;
    switch (GetFirmwareVersion())
    {
    case 0x91:
        selfTestResult = selfTestResultV1;
        break;
    case 0x92:
        selfTestResult = selfTestResultV2;
        break;
    default:
        return false;
    }

    Reset();
    WriteToRegister(FIFODataReg, 0x00);
    WriteToRegister(CommandReg, MFRC522_MEM);
    WriteToRegister(AutoTestReg, 0x09);
    WriteToRegister(FIFODataReg, 0x00);
    WriteToRegister(CommandReg, MFRC522_CALCCRC);

    // Wait for the self test to complete.
    i = 0xFF;
    do
    {
        n = ReadFromRegister(DivIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x04));

    for (i = 0; i < 64; i++)
    {
        if (ReadFromRegister(FIFODataReg) != selfTestResult[i])
        {
            ESP_LOGI(TAG, "i:%d\n", i);
            return false;
        }
    }
    return true;
}

/**************************************************************************/
/*!

  @brief   Sends a command to a tag.

  @param   cmd     The command to the MFRC522 to send a command to the tag.
  @param   data    The data that is needed to complete the command.
  @param   dlen    The length of the data.
  @param   result  The result returned by the tag.
  @param   rlen    The number of valid bits in the resulting value.

  @returns Returns the status of the calculation.
           MI_ERR        if something went wrong,
           MI_NOTAGERR   if there was no tag to send the command to.
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int CommandTag(uint8_t cmd, uint8_t *data, int dlen, uint8_t *result, int *rlen)
{
    int status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits, n;
    int i;

    switch (cmd)
    {
    case MFRC522_AUTHENT:
        irqEn = 0x12;
        waitIRq = 0x10;
        break;
    case MFRC522_TRANSCEIVE:
        irqEn = 0x77;
        waitIRq = 0x30;
        break;
    default:
        break;
    }

    WriteToRegister(CommIEnReg, irqEn | 0x80); // interrupt request
    ClearBitMask(CommIrqReg, 0x80);            // Clear all interrupt requests bits.
    SetBitMask(FIFOLevelReg, 0x80);            // FlushBuffer=1, FIFO initialization.

    WriteToRegister(CommandReg, MFRC522_IDLE); // No action, cancel the current command.

    // Write to FIFO
    for (i = 0; i < dlen; i++)
    {
        WriteToRegister(FIFODataReg, data[i]);
    }

    // Execute the command.
    WriteToRegister(CommandReg, cmd);
    if (cmd == MFRC522_TRANSCEIVE)
    {
        SetBitMask(BitFramingReg, 0x80); // StartSend=1, transmission of data starts
    }

    // Waiting for the command to complete so we can receive data.
    i = 25; // Max wait time is 25ms.
    do
    {
        vTaskDelay(pdMS_TO_TICKS(1)); // 1 millisecond.
        // CommIRqReg[7..0]
        // Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = ReadFromRegister(CommIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    ClearBitMask(BitFramingReg, 0x80); // StartSend=0

    if (i != 0)
    { // Request did not time out.
        if (!(ReadFromRegister(ErrorReg) & 0x1D))
        { // BufferOvfl Collerr CRCErr ProtocolErr
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
                status = MI_NOTAGERR;
            }

            if (cmd == MFRC522_TRANSCEIVE)
            {
                n = ReadFromRegister(FIFOLevelReg);
                lastBits = ReadFromRegister(ControlReg) & 0x07;
                if (lastBits)
                {
                    *rlen = (n - 1) * 8 + lastBits;
                }
                else
                {
                    *rlen = n * 8;
                }

                if (n == 0)
                {
                    n = 1;
                }

                if (n > MAX_LEN)
                {
                    n = MAX_LEN;
                }

                // Reading the recieved data from FIFO.
                for (i = 0; i < n; i++)
                {
                    result[i] = ReadFromRegister(FIFODataReg);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }
    return status;
}

/**************************************************************************/
/*!

  @brief   Checks to see if there is a tag in the vicinity.

  @param   mode  The mode we are requsting in.
  @param   type  If we find a tag, this will be the type of that tag.
                 0x4400 = Mifare_UltraLight
                 0x0400 = Mifare_One(S50)
                 0x0200 = Mifare_One(S70)
                 0x0800 = Mifare_Pro(X)
                 0x4403 = Mifare_DESFire

  @returns Returns the status of the request.
           MI_ERR        if something went wrong,
           MI_NOTAGERR   if there was no tag to send the command to.
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int RequestTag(uint8_t mode, uint8_t *data)
{
    int status, len;
    WriteToRegister(BitFramingReg, 0x07); // TxLastBists = BitFramingReg[2..0]

    data[0] = mode;
    status = CommandTag(MFRC522_TRANSCEIVE, data, 1, data, &len);

    if ((status != MI_OK) || (len != 0x10))
    {
        status = MI_ERR;
    }

    return status;
}

/**************************************************************************/
/*!

  @brief   Handles collisions that might occur if there are multiple
           tags available.

  @param   serial  The serial nb of the tag.

  @returns Returns the status of the collision detection.
           MI_ERR        if something went wrong,
           MI_NOTAGERR   if there was no tag to send the command to.
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int AntiCollision(uint8_t *serial)
{
    int status, i, len;
    uint8_t check = 0x00;

    WriteToRegister(BitFramingReg, 0x00); // TxLastBits = BitFramingReg[2..0]

    serial[0] = MF1_ANTICOLL;
    serial[1] = 0x20;
    status = CommandTag(MFRC522_TRANSCEIVE, serial, 2, serial, &len);
    len = len / 8; // len is in bits, and we want each uint8_t.
    if (status == MI_OK)
    {
        // The checksum of the tag is the ^ of all the values.
        for (i = 0; i < len - 1; i++)
        {
            check ^= serial[i];
        }
        // The checksum should be the same as the one provided from the
        // tag (serial[4]).
        if (check != serial[i])
        {
            status = MI_ERR;
        }
    }
    return status;
}

/**************************************************************************/
/*!

  @brief   Calculates the CRC value for some data that should be sent to
           a tag.

  @param   data    The data to calculate the value for.
  @param   len     The length of the data.
  @param   result  The result of the CRC calculation.

 */
/**************************************************************************/
void CalculateCRC(uint8_t *data, int len, uint8_t *result)
{
    int i;
    uint8_t n;

    ClearBitMask(DivIrqReg, 0x04);  // CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80); // Clear the FIFO pointer

    // Writing data to the FIFO.
    for (i = 0; i < len; i++)
    {
        WriteToRegister(FIFODataReg, data[i]);
    }
    WriteToRegister(CommandReg, MFRC522_CALCCRC);

    // Wait for the CRC calculation to complete.
    i = 0xFF;
    do
    {
        n = ReadFromRegister(DivIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x04)); // CRCIrq = 1

    // Read the result from the CRC calculation.
    result[0] = ReadFromRegister(CRCResultRegL);
    result[1] = ReadFromRegister(CRCResultRegM);
}

/**************************************************************************/
/*!

  @brief   Selects a tag for processing.

  @param   serial  The serial number of the tag that is to be selected.

  @returns The SAK response from the tag.

 */
/**************************************************************************/
uint8_t SelectTag(uint8_t *serial)
{
    int i, status, len;
    uint8_t sak;
    uint8_t buffer[9];

    buffer[0] = MF1_SELECTTAG;
    buffer[1] = 0x70;
    for (i = 0; i < 5; i++)
    {
        buffer[i + 2] = serial[i];
    }
    CalculateCRC(buffer, 7, &buffer[7]);

    status = CommandTag(MFRC522_TRANSCEIVE, buffer, 9, buffer, &len);

    if ((status == MI_OK) && (len == 0x18))
    {
        sak = buffer[0];
    }
    else
    {
        sak = 0;
    }

    return sak;
}

/**************************************************************************/
/*!

  @brief   Handles the authentication between the tag and the reader.

  @param   mode    What authentication key to use.
  @param   block   The block that we want to read.
  @param   key     The authentication key.
  @param   serial  The serial of the tag.

  @returns Returns the status of the collision detection.
           MI_ERR        if something went wrong,
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int Authenticate(uint8_t mode, uint8_t block, uint8_t *key, uint8_t *serial)
{
    int i, status, len;
    uint8_t buffer[12];

    // Verify the command block address + sector + password + tag serial number
    buffer[0] = mode;  // 0th uint8_t is the mode
    buffer[1] = block; // 1st uint8_t is the block to address.
    for (i = 0; i < 6; i++)
    { // 2nd to 7th uint8_t is the authentication key.
        buffer[i + 2] = key[i];
    }
    for (i = 0; i < 4; i++)
    { // 8th to 11th uint8_t is the serial of the tag.
        buffer[i + 8] = serial[i];
    }

    status = CommandTag(MFRC522_AUTHENT, buffer, 12, buffer, &len);

    if ((status != MI_OK) || (!(ReadFromRegister(Status2Reg) & 0x08)))
    {
        status = MI_ERR;
    }

    return status;
}

/**************************************************************************/
/*!

  @brief   Tries to read from the current (authenticated) tag.

  @param   block   The block that we want to read.
  @param   result  The resulting value returned from the tag.

  @returns Returns the status of the collision detection.
           MI_ERR        if something went wrong,
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int ReadFromTag(uint8_t block, uint8_t *result)
{
    int status, len;

    result[0] = MF1_READ;
    result[1] = block;
    CalculateCRC(result, 2, &result[2]);
    status = CommandTag(MFRC522_TRANSCEIVE, result, 4, result, &len);

    if ((status != MI_OK) || (len != 0x90))
    {
        status = MI_ERR;
    }

    return status;
}

/**************************************************************************/
/*!

  @brief   Tries to write to a block on the current tag.

  @param   block  The block that we want to write to.
  @param   data   The data that we shoudl write to the block.

  @returns Returns the status of the collision detection.
           MI_ERR        if something went wrong,
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int WriteToTag(uint8_t block, uint8_t *data)
{
    int status, i, len;
    uint8_t buffer[18];

    buffer[0] = MF1_WRITE;
    buffer[1] = block;
    CalculateCRC(buffer, 2, &buffer[2]);
    status = CommandTag(MFRC522_TRANSCEIVE, buffer, 4, buffer, &len);

    if ((status != MI_OK) || (len != 4) || ((buffer[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    if (status == MI_OK)
    {
        for (i = 0; i < 16; i++)
        {
            buffer[i] = data[i];
        }
        CalculateCRC(buffer, 16, &buffer[16]);
        status = CommandTag(MFRC522_TRANSCEIVE, buffer, 18, buffer, &len);

        if ((status != MI_OK) || (len != 4) || ((buffer[0] & 0x0F) != 0x0A))
        {
            status = MI_ERR;
        }
    }

    return status;
}

/**************************************************************************/
/*!

  @brief   Sends a halt command to the current tag.

  @returns Returns the result of the halt.
           MI_ERR        If the command didn't complete properly.
           MI_OK         If the command completed.
 */
/**************************************************************************/
int HaltTag() {
  int status, len;
  uint8_t buffer[4];

  buffer[0] = MF1_HALT;
  buffer[1] = 0;
  CalculateCRC(buffer, 2, &buffer[2]);
  status = CommandTag(MFRC522_TRANSCEIVE, buffer, 4, buffer, &len);
  ClearBitMask(Status2Reg, 0x08);  // turn off encryption
  return status;
}