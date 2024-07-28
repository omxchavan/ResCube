#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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
#include "ultrasonic.h"
#include "time.h"
#include "esp_timer.h"
#include "driver/timer.h"
#include <string.h>
#include <inttypes.h>

#define BUF_SIZE (1024)
#define REFERENCE_PRESSURE 101325
#define TAG "BMP180_APP"
#define SOUND_SPEED_MPS 343.0


#define TRIG_PIN 4
#define ECHO_PIN 2



#define MAX_DISTANCE_CM 500 

#define GPS_UART_PORT_NUM      0
#define GPS_UART_BAUD_RATE     9600
#define GPS_UART_TX_PIN        1
#define GPS_UART_RX_PIN        3




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
TaskHandle_t inoled = NULL;

void init_timer() {
    timer_config_t config = {
        .divider = 16,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = TIMER_AUTORELOAD_DIS,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
}

uint32_t measure_distance() {
    gpio_set_level(TRIG_PIN, 0);
    ets_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    ets_delay_us(20);
    gpio_set_level(TRIG_PIN, 0);

    while (gpio_get_level(ECHO_PIN) == 0) {
    }

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, TIMER_0);

    while (gpio_get_level(ECHO_PIN) == 1) {
        
    }

    timer_pause(TIMER_GROUP_0, TIMER_0);
    uint64_t echo_time;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &echo_time);

    float distance = (echo_time * 0.343) / 2;
    return (uint32_t)distance;
}




void short_range(void *arg)
{
	if(xSemaphoreTake(xMutex, portMAX_DELAY))
	{
	SSD1306_t dev;
    i2c_master_init(&dev, 21, 22, -1);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    

    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    
    init_timer();

    while (1) {
        uint32_t distance = measure_distance();
        distance = distance/10;
        char dist_str[15];
        snprintf(dist_str, sizeof(dist_str), "%" PRIu32 " cm", distance);
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text_x3(&dev, 0, "Range", 5, false);
        ssd1306_display_text(&dev, 4, "Finder", 6, false);
        ssd1306_display_text(&dev, 5, dist_str, strlen(dist_str), false);
        ESP_LOGI(TAG, "Distance: %" PRIu32 " cm", distance);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

}

}

void initmenu(void *arg) //not in use
{
	SSD1306_t dev;
        i2c_master_init(&dev, 21, 22, 27);
        ssd1306_init(&dev, 128, 64);
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);
        ssd1306_display_text_x3(&dev, 0, "Res", 3, false);
        ssd1306_display_text_x3(&dev, 4, "Cube", 4, false);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ssd1306_display_text(&dev, 6, "Initialising...", 15, false);
        vTaskDelay(2000/portTICK_PERIOD_MS);
        
        while(1)
        {
        ssd1306_contrast(&dev, 0xff);
        ssd1306_display_text_x3(&dev, 0, "Res", 3, false);
        ssd1306_display_text_x3(&dev, 4, "Cube", 4, false);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ssd1306_display_text(&dev, 6, "Initialising...", 15, false);
        vTaskDelay(2000/portTICK_PERIOD_MS);
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text_x3(&dev, 0, "Menu", 4, false);
        ssd1306_display_text_x3(&dev, 3, "35 for ", 5, false);
        ssd1306_display_text_x3(&dev, 4, "36 for ", 5, false);
        ssd1306_display_text_x3(&dev, 5, "33 for", 5, false);
        ssd1306_display_text_x3(&dev, 6, "Rst for new", 3, false);
        }
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

void oled_displaylora()
{
	ESP_LOGI(TAG, "Oled lora + gps oled is running now");
	
    
   {
	    uint8_t data[BUF_SIZE];
	    int length = uart_read_bytes(GPS_UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        SSD1306_t dev;
        i2c_master_init(&dev, 21, 22, 27);
        ssd1306_clear_screen(&dev, false);
        ssd1306_init(&dev, 128, 64);
        ssd1306_contrast(&dev, 0xff);
        
        while (1) {
      
        int length = uart_read_bytes(GPS_UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (length > 0) {
            data[length] = '\0'; 
            ESP_LOGI(TAG, "Received: %s", data);

           
            ssd1306_clear_screen(&dev, false);
            ssd1306_display_text(&dev, 0, "Sending" ,7 , false);
            ssd1306_display_text_x3(&dev, 1, "SOS", 3, false);
            ssd1306_display_text(&dev, 5, "TXFreq:915",10, false);
            
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
        }
       } 
       }
       

void parse_nmea(const char *nmea, char *latitude, char *longitude) {
    const char *start;
    char lat[12], lon[12];

  
    start = strstr(nmea, "$GPGGA");
    if (start != NULL) {
        sscanf(start, "$GPGGA,%*f,%11[^,],%*c,%11[^,],%*c", lat, lon);
        sprintf(latitude, "%s", lat);
        sprintf(longitude, "%s", lon);
    }
}

void lorarf(void *arg) 
{    
 
const uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(GPS_UART_PORT_NUM, &uart_config);
    uart_set_pin(GPS_UART_PORT_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uint8_t data[BUF_SIZE];
    char latitude[12] = {0};
    char longitude[12] = {0};

    while (1) {
        int length = uart_read_bytes(GPS_UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (length > 0) {
            data[length] = '\0'; 
            ESP_LOGI(TAG, "Received: %s", data);

          
            parse_nmea((char *)data, latitude, longitude);

            
            ESP_LOGI(TAG, "Latitude: %s, Longitude: %s", latitude, longitude);

            // final payload
            char lora_payload[50];
            snprintf(lora_payload, sizeof(lora_payload), "Lat: %s, Lon: %s", latitude, longitude);
            lora_send_packet((uint8_t*)lora_payload, strlen(lora_payload));
            printf("Packet sent...\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}
void morse_task(void *arg)
{
	ESP_LOGI(TAG, "sos task is running now");
   while(1) 
    {
	gpio_set_level(17, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(17, 0);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(17, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(17, 1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(17, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(17, 0);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(17, 1);
    vTaskDelay(1000/portTICK_PERIOD_MS); // -
    gpio_set_level(17, 1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(17, 1);
    vTaskDelay(1000/portTICK_PERIOD_MS); // -
    gpio_set_level(17, 0);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(17, 1);
    vTaskDelay(1000/portTICK_PERIOD_MS); // -
    gpio_set_level(17, 1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(17, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(17, 0);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(17, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(17, 0);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(17, 1);
    vTaskDelay(300/portTICK_PERIOD_MS); // .
    gpio_set_level(17, 0);
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
while(1)
{
        char code[15];
        strcpy(code, "Flashing sos");
        ssd1306_display_text_x3(&dev, 0, "SOS", 3, false);
        ssd1306_display_text(&dev, 4, code, 15, false);
        ssd1306_display_text(&dev, 5, "...---...", 9, false);
        xSemaphoreGive(xMutex);
       }
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
   const uart_config_t uart_log_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_log_config);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    //esp_err_t err;
    gpio_set_direction(35, GPIO_MODE_INPUT);
    gpio_set_direction(36, GPIO_MODE_INPUT);
    gpio_set_direction(34, GPIO_MODE_INPUT);
    gpio_set_direction(39, GPIO_MODE_INPUT);
    gpio_set_direction(17, GPIO_MODE_OUTPUT);
    lora_init();
    lora_set_frequency(915e6); // optimum signal freq ISM 435-510
    lora_enable_crc();
    xMutex = xSemaphoreCreateMutex();
    xSemaphoreGive(xMutex);
    
    
    
   while(1)
{ 

    
    int gpio_36 = gpio_get_level(36);
    int gpio_35 = gpio_get_level(35);
    int gpio_34 = gpio_get_level(34);
    int gpio_39 = gpio_get_level(39);

    vTaskDelay(200 / portTICK_PERIOD_MS); 

    
    printf("GPIO 36: %d\n", gpio_36);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    printf("GPIO 35: %d\n", gpio_35);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    printf("GPIO 34: %d\n", gpio_34);
    vTaskDelay(500 / portTICK_PERIOD_MS); 
    printf("GPIO 39: %d\n", gpio_39);
    vTaskDelay(500 / portTICK_PERIOD_MS); 



    if (gpio_39 == 1)
    {
       xTaskCreate(&short_range, "short range", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
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
    
     else if (gpio_36 == 1)
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
        xTaskCreate(&lorarf, "lora rf", 4092*2, NULL, 5, &handle4);
        xTaskCreatePinnedToCore(&oled_displaylora, "lora display on oled", 4092*2, NULL, 5, &handle5, 1);
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
    else if (gpio_34 == 1)
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
     vTaskDelay(pdMS_TO_TICKS(100));
}
}
