

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
#include "esp_task_wdt.h"
#include <stdio.h>
#include "esp_pm.h"


static const char *TAG = "Lathe";


#define EXAMPLE_EC11_GPIO_A 35
#define EXAMPLE_EC11_GPIO_B 2

#define DirPin GPIO_NUM_4
#define PulcePin GPIO_NUM_5

#define EXAMPLE_PCNT_HIGH_LIMIT 2400//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define EXAMPLE_PCNT_LOW_LIMIT  -2400//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

#define LeadScrewPitch 200  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define DesirePitch 100     //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

#define SpindlePulses  4000 //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define StepperSteps   200  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// int direction = 0;
int PrevValue = 0;
int current_value;
SemaphoreHandle_t xBinarySemaphoreClock;
SemaphoreHandle_t xBinarySemaphoreCounterClock;

typedef struct {
    int *arr;      // pointer to the dynamically allocated array
    int length;    // how many elements are in arr
} block_data_t;

void TaskStepperClock(void *pvParameters) {
    esp_rom_gpio_pad_select_gpio(DirPin);
    gpio_set_direction(DirPin, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(PulcePin);
    gpio_set_direction(PulcePin, GPIO_MODE_OUTPUT);
    while (1) {
        if(xSemaphoreTake(xBinarySemaphoreClock, portMAX_DELAY)==pdTRUE){     
            gpio_set_level(DirPin,1);      
            gpio_set_level(PulcePin, 1);
            vTaskDelay(pdMS_TO_TICKS(1));
            gpio_set_level(PulcePin, 0);
        }
    }
}

void TaskStepperCounterClock(void *pvParameters) {
    esp_rom_gpio_pad_select_gpio(DirPin);
    gpio_set_direction(DirPin, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(PulcePin);
    gpio_set_direction(PulcePin, GPIO_MODE_OUTPUT);
    while (1) {
        if(xSemaphoreTake(xBinarySemaphoreCounterClock, portMAX_DELAY)==pdTRUE){     
            gpio_set_level(DirPin,0);      
            gpio_set_level(PulcePin, 1);
            vTaskDelay(pdMS_TO_TICKS(1));
            gpio_set_level(PulcePin, 0);
        }
    }
}

void TaskRottary(void *pvParameters)
{
    esp_task_wdt_add(NULL);
        block_data_t *p = (block_data_t *)pvParameters;
    int *numbers = p->arr;       // pointer to allocated array
    int sizeOfList = p->length;  // length


        ESP_LOGI("RottaryEnc", "sizeOfList = %d", sizeOfList);
    for (int k = 0; k < sizeOfList; k++) {
        ESP_LOGI("RottaryEnc", "Element %d = %d", k, numbers[k]);
    }

    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_handle_t pcnt_unit = NULL;// INFO  I TOOK IT FROM BELO
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
   
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

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
    int pulse_count = 0;
    for (;;) {

            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
                    if (pulse_count % numbers[0] == 0 && pulse_count>0) {
                            xSemaphoreGive(xBinarySemaphoreClock);
                            pcnt_unit_clear_count(pcnt_unit);
                            int x = numbers[0];
                            for(int i=0; i<sizeOfList - 1; i++){
                                numbers[i]=numbers[i+1];
                            }
                            numbers[sizeOfList-1] = x;

                        } else if (pulse_count % numbers[0] == 0 && pulse_count < 0){
                            xSemaphoreGive(xBinarySemaphoreCounterClock);
                            pcnt_unit_clear_count(pcnt_unit);
                            int x = numbers[0];
                            for(int i=0; i<sizeOfList - 1; i++){
                                numbers[i]=numbers[i+1];
                            }
                            numbers[sizeOfList-1] = x;
                        } else{;;}
                         esp_task_wdt_reset();
                    }
}




static int gcd(int a, int b)
{
    if (b == 0) return a;
    return gcd(b, a % b);
}


void app_main() {
   // Wait for system to stabilize
   esp_task_wdt_config_t wd_config = {
    .timeout_ms =5000,
    .idle_core_mask=0,
    .trigger_panic=true,
   };
     esp_err_t ret = esp_task_wdt_reconfigure(&wd_config); // Apply configuration
    if (ret != ESP_OK) {
        printf("Task Watchdog Timer reconfiguration failed: %d\n", ret);
    }

    // Check and print the APB clock frequency
    xBinarySemaphoreClock = xSemaphoreCreateBinary();
    if (xBinarySemaphoreClock == NULL) {
        ESP_LOGE("app_main", "Failed to create semaphore");
        return; // Abort if semaphore creation fails
    }
        xBinarySemaphoreCounterClock = xSemaphoreCreateBinary();
    if (xBinarySemaphoreCounterClock == NULL) {
        ESP_LOGE("app_main", "Failed to create semaphore");
        return; // Abort if semaphore creation fails
    }




    // We want to express ratio = pulses / steps in simplest form.
    // ratio = 4000 / 175

    // 1) Compute GCD


    float fRleadScrew = StepperSteps * ((float)DesirePitch / (float)LeadScrewPitch);
    int RleadScrew = (int)fRleadScrew;
    ESP_LOGE("app_main", ">>>>>>>>>>>>>>>>>>>>>!!!!!!!!!>>>>>>>>  %d ===,%d,====%d",StepperSteps , DesirePitch,LeadScrewPitch);
    ESP_LOGE("app_main", ">>>>>>>>>>>>>>>>>>>>>!!!!!!!!!>>>>>>>>  %d",RleadScrew);
   
    int divisor = gcd(SpindlePulses, RleadScrew);

    // 2) Divide both by GCD to get reduced fraction
    int numerator   = SpindlePulses / divisor;  // e.g. 4000 / 25 = 160
    int denominator = RleadScrew  / divisor;  // e.g. 175 / 25  = 7

    // 3) Print results
    printf("Ratio (pulses/steps) = %d/%d Divisor = %d\n", numerator, denominator, divisor);

    // The fraction is 160/7, meaning:
    // - For every 7 'step blocks', we have 160 'pulse blocks'.
    // - Repeated enough times => 175 steps, 4000 pulses.

    int blockSteps  = denominator; // 7
    int blockPulses = numerator;   // 160

    printf("Block size (steps) = %d, Pulses in that block = %d\n",
           blockSteps, blockPulses);

    // Keep this task alive
    
    int *blockArray = malloc(blockSteps * sizeof(int));
    if (!blockArray) {
        ESP_LOGE("app_main", "Failed to allocate memory for blockArray");
        return; // handle error gracefully
    }

    int FirstEntries = numerator / blockSteps;
    ESP_LOGE("app_main", "FirstEnttries: %d", FirstEntries);
    for (int i =0; i<blockSteps; i++){
        blockArray[i]=FirstEntries;
    }
    int Remaining = blockPulses - (FirstEntries *blockSteps);
    ESP_LOGE("app_main", "Remaining: %d", Remaining);
    if (Remaining>0){
        int counter = 0;
        for (int j=0; j<Remaining; j++){
            blockArray[counter]++;
            counter++;
            if (counter>blockSteps){
                counter =0;
            }
        }
    }

    for(int k=0; k<blockSteps; k++){
        ESP_LOGE("app_main", "Element: %d, Size: %d",k,blockArray[k]);
    }
    // vTaskDelay(1000000);

    block_data_t *params = malloc(sizeof(block_data_t));
    if (!params) {
        ESP_LOGE("app_main", "Failed to allocate memory for params");
        free(blockArray);
        return; // handle error
    }

    params->arr    = blockArray;   // store the pointer
    params->length = blockSteps;   // store the length

    // Create the task and pin it to core 1
    xTaskCreatePinnedToCore(
        TaskStepperClock,            // Function that implements the task
        "TaskStepperClock",      // Name of the task
        4096,             // Stack size
        NULL,             // Task input parameter
        1,                // Priority of the task
        NULL,             // Task handle
        tskNO_AFFINITY
        // 1                 // Core where the task should run (1 for core 1, 0 for core 0)

    );


    xTaskCreatePinnedToCore(
        TaskStepperCounterClock,            // Function that implements the task
        "TaskStepperCounterClock",      // Name of the task
        4096,             // Stack size
        NULL,             // Task input parameter
        1,                // Priority of the task
        NULL,             // Task handle
        tskNO_AFFINITY
        // 1                 // Core where the task should run (1 for core 1, 0 for core 0)

    );


    xTaskCreatePinnedToCore(  
        TaskRottary,
        "RotaryEncoder",
        4096,
        (void *)params,
        1,
        NULL,
        tskNO_AFFINITY
        // 0
    );
}