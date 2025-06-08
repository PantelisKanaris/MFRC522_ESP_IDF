#include <stdio.h>
#include <driver/gpio.h>
#include "Driver/MFRC522.h"
#include "esp_log.h"
#define SDA_PIN GPIO_NUM_10
#define RST_PIN GPIO_NUM_5
#define MOSI_PIN GPIO_NUM_23
#define MISO_PIN GPIO_NUM_19
#define SCLK_PIN GPIO_NUM_18

static char *TAG = "RFID_MAIN";

uint8_t keyA[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t keyB[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void app_main(void)
{
    esp_err_t ret = Initialize(SPI2_HOST, SDA_PIN, RST_PIN, MOSI_PIN, MISO_PIN, SCLK_PIN);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Initialization failed.");
        return;
    }
    Begin();

    uint8_t version = GetFirmwareVersion();
    if (version == 0)
    {
        ESP_LOGE(TAG, "MFRC522 not found.");
        return;
    }

    ESP_LOGI(TAG, "Found MFRC522 chip, firmware version: 0x%02X", version);

    uint8_t status;
    uint8_t data[MAX_LEN];
    uint8_t serial[5];
    while (1)
    {
        status = RequestTag(MF1_REQIDL, data);

        if (status == MI_OK)
        {
            ESP_LOGI(TAG, "Tag detected. Type: 0x%02X 0x%02X", data[0], data[1]);

            status = AntiCollision(serial);
            if (status != MI_OK)
            {
                ESP_LOGW(TAG, "Anti-collision failed.");
                continue;
            }

            ESP_LOGI(TAG, "Serial Number: %02X %02X %02X %02X", serial[0], serial[1], serial[2], serial[3]);

            SelectTag(serial);

            for (int i = 0; i < 64; i++)
            {
                status = Authenticate(MF1_AUTHENT1A, i, keyA, serial);

                if (status == MI_OK)
                {
                    ESP_LOGI(TAG, "Authenticated block %d with Key A", i);

                    if (ReadFromTag(i, data) == MI_OK)
                    {
                        char buf[128] = {0};
                        char *ptr = buf;
                        for (int j = 0; j < 16; j++)
                        {
                            ptr += sprintf(ptr, "%02X ", data[j]);
                        }
                        ESP_LOGI(TAG, "Data: %s", buf);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Read failed at block %d", i);
                    }
                }
                else
                {
                    status = Authenticate(MF1_AUTHENT1B, i, keyB, serial);
                    if (status == MI_OK)
                    {
                        ESP_LOGI(TAG, "Authenticated block %d with Key B", i);

                        if (ReadFromTag(i, data) == MI_OK)
                        {
                            char buf[128] = {0};
                            char *ptr = buf;
                            for (int j = 0; j < 16; j++)
                            {
                                ptr += sprintf(ptr, "%02X ", data[j]);
                            }
                            ESP_LOGI(TAG, "Data: %s", buf);
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Read failed at block %d", i);
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Access denied at block %d", i);
                    }
                }
            }

            HaltTag();
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
