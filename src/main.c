

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


static const char *TAG = "example";

#define EXAMPLE_PCNT_HIGH_LIMIT 2400
#define EXAMPLE_PCNT_LOW_LIMIT  -2400

#define EXAMPLE_EC11_GPIO_A 35
#define EXAMPLE_EC11_GPIO_B 2

#define DirPin GPIO_NUM_4
#define PulcePin GPIO_NUM_5


// int direction = 0;
int PrevValue = 0;
int current_value;
SemaphoreHandle_t xBinarySemaphoreClock;
SemaphoreHandle_t xBinarySemaphoreCounterClock;



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
    int numbers[] = {27, 26, 27};
    // int numbers175[] = {22,23,23,24,23,23,22};
   
    
    // int numbers[] = numbers15;
    int sizeOfList = sizeof(numbers) / sizeof(numbers[0]);
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


    int pulses = 2400;
    int steps  = 150;

    // We want to express ratio = pulses / steps in simplest form.
    // ratio = 4000 / 175

    // 1) Compute GCD
    int divisor = gcd(pulses, steps);

    // 2) Divide both by GCD to get reduced fraction
    int numerator   = pulses / divisor;  // e.g. 4000 / 25 = 160
    int denominator = steps  / divisor;  // e.g. 175 / 25  = 7

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

    vTaskDelay(1000000);


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
        NULL,
        1,
        NULL,
        tskNO_AFFINITY
        // 0
    );

    //     xTaskCreatePinnedToCore(  
    //     TaskRottaryCounterClock,
    //     "RotaryEncoderCounterClock",
    //     4096,
    //     NULL,
    //     1,
    //     NULL,
    //     tskNO_AFFINITY
    //     // 0
    // );
}




// 2. Use a Single Watchpoint + Clear/Reset the Counter

// If you need an event every N pulses (e.g., 12 pulses), you can:

//     Set a single watchpoint at count = N (e.g., 12).
//     When it triggers, in the ISR:
//         Notify your main task (via a semaphore or queue).
//         Call pcnt_unit_clear_count() to reset the count to 0.

// This effectively gives you an “interrupt every 12 pulses,” but it uses only one watchpoint. You don’t need to set 100 watchpoints for 12, 24, 36, etc.

// Drawback: You won’t know the absolute count over time without doing extra tracking. You’re effectively chunking your count in blocks of 12. But if all you need is “do something every 12 pulses,” this approach works well.



/////////////////////////////////////////////////////////

// #include <stdio.h>
// #include <math.h>
// #include <stdbool.h>
// #include <stdlib.h>

// #define DESIRE 22.85  // Target average
// #define MIN_VALUE 20  // Minimum value in the list
// #define MAX_VALUE 30  // Maximum value in the list

// // Function prototypes
// void incrementList(int **list, int *size, int minValue, int maxValue);
// bool calculatePattern(int *list, int size);

// void app_main() {
//     // Initialize the list with one element set to MIN_VALUE
//     int *list = malloc(sizeof(int));  // Dynamically allocate memory for the list
//     int size = 1;  // Current size of the list
//     list[0] = MIN_VALUE;

//     while (true) {
//         // Increment the list
//         incrementList(&list, &size, MIN_VALUE, MAX_VALUE);

//         // Check if the desired average is achieved
//         if (calculatePattern(list, size)) {
//             printf("Final list: ");
//             for (int i = 0; i < size; i++) {
//                 printf("%d ", list[i]);
//             }
//             printf("\n");
//             break;
//         }
//     }

//     // Free the dynamically allocated memory
//     free(list);
// }

// // Function to increment the list
// void incrementList(int **list, int *size, int minValue, int maxValue) {
//     for (int i = 0; i < *size; i++) {
//         (*list)[i]++;

//         if ((*list)[i] > maxValue) {  // If the value exceeds maxValue
//             (*list)[i] = minValue;   // Reset to minValue

//             if (i == *size - 1) {  // If it's the last element, add a new element
//                 *size += 1;
//                 *list = realloc(*list, *size * sizeof(int));
//                 (*list)[*size - 1] = minValue;
//             }
//         } else {
//             return;  // Stop after incrementing the first incomplete element
//         }
//     }
// }

// // Function to calculate the average and compare it with the desired value
// bool calculatePattern(int *list, int size) {
//     int sum = 0;
//     for (int i = 0; i < size; i++) {
//         sum += list[i];
//     }

//     double average = (double)sum / size;            // Calculate the average
//     double truncatedAverage = trunc(average * 100) / 100;  // Truncate to two decimals

//     printf("Average: %.2f, Desired: %.2f, Size: %d\n", truncatedAverage, DESIRE, size);

//     return truncatedAverage == DESIRE;
// }




///////////////////////////////////////

    // for (;;) {

    //         ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
    //         if (pulse_count != prev) {
                    
    //                 if (pulse_count == 0) {
    //                     // At zero, which direction did we come from?
    //                     if (prev > 0) {
    //                         xSemaphoreGive(xBinarySemaphoreClock);
    //                     } else if (prev < 0) {
    //                         xSemaphoreGive(xBinarySemaphoreCounterClock);
    //                     }
    //                 }
    //                 else if (pulse_count % 12 == 0) {
    //                     // At a multiple of 12, decide direction
    //                     if (pulse_count > prev) {
    //                         xSemaphoreGive(xBinarySemaphoreClock);
    //                     } else {
    //                         xSemaphoreGive(xBinarySemaphoreCounterClock);
    //                     }
    //                 }

    //                 prev = pulse_count;  // Update the "previous" counter
    //             }
               
    //         // }
    //             esp_task_wdt_reset();
    //         }