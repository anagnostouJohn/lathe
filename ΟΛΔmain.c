

#include <stdio.h>
#include <driver/pulse_cnt.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "esp_sleep.h"

static const char *TAG = "example";

#define EXAMPLE_PCNT_HIGH_LIMIT 2400
#define EXAMPLE_PCNT_LOW_LIMIT  -2400

#define EXAMPLE_EC11_GPIO_A 35
#define EXAMPLE_EC11_GPIO_B 2

// // #define LED_PIN GPIO_NUM_4
#define DirPin GPIO_NUM_4
#define PulcePin GPIO_NUM_5

// #define EXAMPLE_CHAN_GPIO_A 2
// #define EXAMPLE_CHAN_GPIO_B 42

// #define EXAMPLE_PCNT_HIGH_LIMIT 100
// #define EXAMPLE_PCNT_LOW_LIMIT  -100


int direction = 0;
int PrevValue = 0;
int current_value;
SemaphoreHandle_t xBinarySemaphoreClock;
// SemaphoreHandle_t xBinarySemaphoreCounterClock;
pcnt_unit_handle_t pcnt_unit = NULL;// INFO  I TOOK IT FROM BELOW
static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;

    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    if (edata->watch_point_value==0){
        PrevValue=0;
    }
      if (edata->watch_point_value < PrevValue) {
        direction = 0;
      } else if (edata->watch_point_value > PrevValue){
        direction =1;
      } else if (edata->watch_point_value == PrevValue){
        ;;
      }
     
    xSemaphoreGiveFromISR(xBinarySemaphoreClock, &high_task_wakeup);
         PrevValue = edata->watch_point_value;
    // } else if (edata->watch_point_value == -1200) {
    //     xSemaphoreGiveFromISR(xBinarySemaphoreCounterClock, &high_task_wakeup);

    // }

    
    portYIELD_FROM_ISR(high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

void Task1(void *pvParameters) {
    // Configure the LED pin as output

    esp_rom_gpio_pad_select_gpio(DirPin);
    gpio_set_direction(DirPin, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(PulcePin);
    gpio_set_direction(PulcePin, GPIO_MODE_OUTPUT);

    while (1) {
        
        if(xSemaphoreTake(xBinarySemaphoreClock, portMAX_DELAY)==pdTRUE){
            
            ESP_LOGI("SEM TAKEN", "AAAAAAAAAAAAAAAA >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %d", direction);
            gpio_set_level(DirPin,direction);
            gpio_set_level(PulcePin, 1);
            esp_rom_delay_us(100);
        //     // vTaskDelay(2 / portTICK_PERIOD_MS);  // Delay 500 ms

        // // Turn the LED off
            gpio_set_level(PulcePin, 0);
        //     vTaskDelay(15 / portTICK_PERIOD_MS);  // Delay 500 ms
        }
        
    //  if (xSemaphoreTake(xBinarySemaphoreCounterClock, portMAX_DELAY)==pdTRUE){
    //         ESP_LOGI("SEM TAKEN", "BBBBBBBBBBBBBBBBBBBB");
    //     }
        
        //  vTaskDelay(10 / portTICK_PERIOD_MS);
    }
   
}

void TaskRottary(void *pvParameters)
{
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
   
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
        .level_gpio_num = EXAMPLE_EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
        .level_gpio_num = EXAMPLE_EC11_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {-2400, -2388, -2376, 0, 2400};
    // int watch_points[] = {12, 24,  48, 60};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(20, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

    // Report counter value
    int pulse_count = 0;
    int event_count = 0;
    while (1) {
        if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
            ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
        } else {
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
            ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
        }
    }
}






void app_main() {

   
    xBinarySemaphoreClock = xSemaphoreCreateBinary();
    // xBinarySemaphoreCounterClock = xSemaphoreCreateBinary();
    if (xBinarySemaphoreClock == NULL) {
        ESP_LOGE("app_main", "Failed to create semaphore");
        return; // Abort if semaphore creation fails
    }
    

    // Create the task and pin it to core 1
    xTaskCreatePinnedToCore(
        Task1,            // Function that implements the task
        "BlinkTask",      // Name of the task
        4096,             // Stack size
        NULL,             // Task input parameter
        1,                // Priority of the task
        NULL,             // Task handle
        1                 // Core where the task should run (1 for core 1, 0 for core 0)
    );

    xTaskCreatePinnedToCore(  
        TaskRottary,
        "RotaryEncoder",
        4096,
        NULL,
        1,
        NULL,
        0
    );
}

