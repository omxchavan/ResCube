#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "bmp180.h"
#include "ssd1306.h"
#include "esp_log.h"
#include <esp_intr_alloc.h>
#include "lora.h"
#include "driver/uart.h"
#include "stdint.h"

#define BUF_SIZE (1024)
#define REFERENCE_PRESSURE 101325
#define TAG "BMP180_APP"

char latitude[12];  
char longitude[12]; 
unsigned int pressure;
float altitude;
float tempratture;

SemaphoreHandle_t xMutex;

TaskHandle_t handle1 = NULL;
TaskHandle_t handle2 = NULL;
TaskHandle_t handle3 = NULL;
TaskHandle_t handle4 = NULL;
TaskHandle_t handle5 = NULL;
TaskHandle_t handle6 = NULL;
TaskHandle_t handle7 = NULL;

void gps()
{
	 ESP_LOGI(TAG, "gps task is running now");
	 uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

   
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    int len;
    char* token;
    
    while (1) {
        len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = 0; 
           token = strtok((char*)data, ",");
            while (token != NULL) {
                if (strcmp(token, "$GPGGA") == 0) {
                    for (int i = 0; i < 2; ++i) {
                        token = strtok(NULL, ",");
                        if (token == NULL) break;
                        if (i == 1) {
                            snprintf(latitude, sizeof(latitude), "%s", token);
                        } else if (i == 2) {
                            snprintf(longitude, sizeof(longitude), "%s", token);
                        }
                    }
                    break;
                }
                token = strtok(NULL, ",");
            }
        }
        
    
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Update display every second
    }
    free(data);
}
void gps_oled()

{
	ESP_LOGI(TAG, "gps oled task is running now");
	SSD1306_t dev;
    i2c_master_init(&dev, 21, 22, -1);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);


}

void stopwatch_oled(void *arg)
{
	ESP_LOGI(TAG, "stopwatch_oled task is running now");
    int i;
    i = 0;
    char time[30];
    SSD1306_t dev;
    i2c_master_init(&dev, 21, 22, -1);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);

    while (1)
    {
        if (xSemaphoreTake(xMutex, portMAX_DELAY))
        {
            i++;
            ssd1306_display_text_x3(&dev, 0, "Stop", 8, false);
            ssd1306_display_text_x3(&dev, 2, "Watch", 8, false);
            sprintf(time, "%i", i);
            ssd1306_display_text(&dev, 6, time, 7, false);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ssd1306_clear_screen(&dev, false);

            xSemaphoreGive(xMutex);
        }
    }
}

void oled_displaylora(void *arg)
{
	ESP_LOGI(TAG, "Oled lora task is running now");
    if (xSemaphoreTake(xMutex, portMAX_DELAY))
    {
        SSD1306_t dev;
        i2c_master_init(&dev, 21, 22, 27);
        ssd1306_init(&dev, 128, 64);
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);

        xSemaphoreGive(xMutex);
    }
}

void lorarf(void *arg) 
{    
	 ESP_LOGI(TAG, "LORArf task is running now");
	 for(;;) {
      vTaskDelay(pdMS_TO_TICKS(5000));
      lora_send_packet((uint8_t*)"Hello", 5);
      printf("packet sent...\n");
   }
}

void morse_task(void *arg)
{
	ESP_LOGI(TAG, "sos task is running now");
   while(1) 
    {
	gpio_set_level(34, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(34, 0);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(34, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(34, 1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(34, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(34, 0);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(34, 1);
    vTaskDelay(1000/portTICK_PERIOD_MS); // -
    gpio_set_level(34, 1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(34, 1);
    vTaskDelay(1000/portTICK_PERIOD_MS); // -
    gpio_set_level(34, 0);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(34, 1);
    vTaskDelay(1000/portTICK_PERIOD_MS); // -
    gpio_set_level(34, 1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(34, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(34, 0);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(34, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(34, 1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(34, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(34, 1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    }
    
  }

void morse_taskoled(void *arg)
{
	ESP_LOGI(TAG, "morse oled  task is running now");
    if (xSemaphoreTake(xMutex, portMAX_DELAY))
    {
        SSD1306_t dev;
        i2c_master_init(&dev, 21, 22, 27);
        ssd1306_init(&dev, 128, 64);
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);

        char code[15];
        strcpy(code, "Flashing sos");
        ssd1306_display_text(&dev, 2, code, 15, false);

        xSemaphoreGive(xMutex);
    }
}

void time_task_with_oled();

/*void oled_displaybme(void *arg)
{
    if (xSemaphoreTake(xMutex, portMAX_DELAY))
    {
        SSD1306_t dev;
        i2c_master_init(&dev, 21, 22, 27);
        ssd1306_init(&dev, 128, 64);
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);

        char alt[12];
        sprintf(alt, "%.2f", altitude);
        ssd1306_display_text(&dev, 0, alt, 8, false);

        char temp[12];
        sprintf(temp, "%.2f", tempratture);
        ssd1306_display_text(&dev, 2, temp, 8, false);

        char press[12];
        sprintf(press, "%u", pressure);
        ssd1306_display_text(&dev, 4, press, 16, false);

        xSemaphoreGive(xMutex);
    }
}
*/

/* void bmp_task(void *arg)
{
    while (1)
    {
        esp_err_t err;

        if (xSemaphoreTake(xMutex, portMAX_DELAY))
        {
            err = bmp180_read_pressure(&pressure);
            if (err != ESP_OK)
            {
                ESP_LOGI(TAG, "Reading of pressure BMP sensor failed");
            }
            err = bmp180_read_altitude(REFERENCE_PRESSURE, &altitude);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Reading of altitude from BMP Sensor failed");
            }
            err = bmp180_read_temperature(&tempratture);
            if (err != ESP_OK)
            {
                ESP_LOGI(TAG, "Reading of BMP Temperature failed");
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);

            xSemaphoreGive(xMutex);
        }
    }
}*/

void app_main()
{
    esp_err_t err;
    gpio_set_direction(14, GPIO_MODE_INPUT);
    gpio_set_direction(35, GPIO_MODE_INPUT);
    gpio_set_direction(25, GPIO_MODE_INPUT);
    gpio_set_direction(39, GPIO_MODE_INPUT);
    gpio_set_direction(34, GPIO_MODE_OUTPUT);
    lora_init();
    lora_set_frequency(915e6);
    lora_enable_crc();
    xMutex = xSemaphoreCreateMutex();
    xSemaphoreGive(xMutex);

    int gpio_14 = gpio_get_level(14);
    int gpio_32 = gpio_get_level(32);
    int gpio_35 = gpio_get_level(35);
    int gpio_39 = gpio_get_level(39);

    vTaskDelay(1000 / portTICK_PERIOD_MS); 

    printf("GPIO 14: %d\n", gpio_14);
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
    printf("GPIO 25: %d\n", gpio_32);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("GPIO 35: %d\n", gpio_35);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("GPIO 39: %d\n", gpio_39);

    vTaskDelay(2000 / portTICK_PERIOD_MS); // Additional delay after GPIO prints

    /*if (gpio_14 == 1)
    {
        err = bmp180_init(21, 22);
        if (err == ESP_OK)
        {
            xTaskCreate(&bmp_task, "bmp task", 1024 * 4, NULL, 5, &handle1);
        }
        else
        {
            ESP_LOGE(TAG, "BMP init failed");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        xTaskCreatePinnedToCore(oled_displaybme, "oled display bme", 2048 * 2, NULL, 5, &handle2, 0);
        vTaskDelete(handle2);
        vTaskDelete(handle3);
        vTaskDelete(handle4);
        vTaskDelete(handle5);
        vTaskDelete(handle6);
        vTaskDelete(handle7);

        handle2 = NULL;
        handle3 = NULL;
        handle4 = NULL;
        handle5 = NULL;
        handle6 = NULL;
        handle7 = NULL;
    }
    */
     if (gpio_32 == 1)
    {
        printf("Starting stopwatch OLED task\n");
        xTaskCreate(&stopwatch_oled, "stop watch", 4092, NULL, 5, &handle3);
        vTaskDelete(handle1);
        vTaskDelete(handle2);
        vTaskDelete(handle4);
        vTaskDelete(handle3);
        vTaskDelete(handle6);
        vTaskDelete(handle7);

        handle1 = NULL;
        handle2 = NULL;
        handle4 = NULL;
        handle3 = NULL;
        handle6 = NULL;
        handle7 = NULL;
    }
    
     else if (gpio_35 == 1)
    {
        xTaskCreate(&lorarf, "lora rf", 4092, NULL, 5, &handle4);
        xTaskCreatePinnedToCore(&oled_displaylora, "lora display on oled", 4092, NULL, 5, &handle5, 1);
        vTaskDelete(handle2);
        vTaskDelete(handle3);
        vTaskDelete(handle1);
        vTaskDelete(handle6);
        vTaskDelete(handle7);

        handle2 = NULL;
        handle3 = NULL;
        handle1 = NULL;
        handle6 = NULL;
        handle7 = NULL;
    }
    else if (gpio_39 == 1)
    {
        xTaskCreate(&morse_task, "sos flash", 4092, NULL, 5, &handle6);
        xTaskCreate(&morse_taskoled, "oled display for sos", 2048 * 2, NULL, 5, &handle7);
        vTaskDelete(handle2);
        vTaskDelete(handle3);
        vTaskDelete(handle4);
        vTaskDelete(handle5);
        vTaskDelete(handle1);

        handle2 = NULL;
        handle3 = NULL;
        handle4 = NULL;
        handle5 = NULL;
        handle1 = NULL;
    }
}
