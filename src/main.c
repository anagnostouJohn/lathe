

// #include <stdio.h>
// #include <driver/pulse_cnt.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "esp_timer.h"
// #include "esp_task_wdt.h"
// #include "esp_rom_sys.h"
// #include "esp_system.h"
// #include "esp_err.h"
// #include "sdkconfig.h"
// #include "freertos/queue.h"
// #include "esp_sleep.h"
// #include "esp_task_wdt.h"
// #include <stdio.h>
// #include "esp_pm.h"
// #include <stdio.h>
// #include "nvs_flash.h"
// #include "nvs.h"



// static const char *TAG = "Lathe";


// #define EXAMPLE_EC11_GPIO_A 35
// #define EXAMPLE_EC11_GPIO_B 2

// #define DirPin GPIO_NUM_4
// #define PulcePin GPIO_NUM_5

// #define STORAGE "Data"

// #define EXAMPLE_PCNT_HIGH_LIMIT 2400//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// #define EXAMPLE_PCNT_LOW_LIMIT  -2400//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// #define LeadScrewPitch 200  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// #define DesirePitch 100     //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// #define SpindlePulses  4000 //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// #define StepperSteps   200  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// // int direction = 0;
// int PrevValue = 0;
// int current_value;
// SemaphoreHandle_t xBinarySemaphoreClock;
// SemaphoreHandle_t xBinarySemaphoreCounterClock;

// typedef struct {
//     int *arr;      // pointer to the dynamically allocated array
//     int length;    // how many elements are in arr
// } block_data_t;

// void TaskStepperClock(void *pvParameters) {
//     esp_rom_gpio_pad_select_gpio(DirPin);
//     gpio_set_direction(DirPin, GPIO_MODE_OUTPUT);

//     esp_rom_gpio_pad_select_gpio(PulcePin);
//     gpio_set_direction(PulcePin, GPIO_MODE_OUTPUT);
//     while (1) {
//         if(xSemaphoreTake(xBinarySemaphoreClock, portMAX_DELAY)==pdTRUE){     
//             gpio_set_level(DirPin,1);      
//             gpio_set_level(PulcePin, 1);
//             vTaskDelay(pdMS_TO_TICKS(1));
//             gpio_set_level(PulcePin, 0);
//         }
//     }
// }

// void TaskStepperCounterClock(void *pvParameters) {
//     esp_rom_gpio_pad_select_gpio(DirPin);
//     gpio_set_direction(DirPin, GPIO_MODE_OUTPUT);

//     esp_rom_gpio_pad_select_gpio(PulcePin);
//     gpio_set_direction(PulcePin, GPIO_MODE_OUTPUT);
//     while (1) {
//         if(xSemaphoreTake(xBinarySemaphoreCounterClock, portMAX_DELAY)==pdTRUE){     
//             gpio_set_level(DirPin,0);      
//             gpio_set_level(PulcePin, 1);
//             vTaskDelay(pdMS_TO_TICKS(1));
//             gpio_set_level(PulcePin, 0);
//         }
//     }
// }

// void TaskRottary(void *pvParameters)
// {
//     esp_task_wdt_add(NULL);
//         block_data_t *p = (block_data_t *)pvParameters;
//     int *numbers = p->arr;       // pointer to allocated array
//     int sizeOfList = p->length;  // length


//         ESP_LOGI("RottaryEnc", "sizeOfList = %d", sizeOfList);
//     for (int k = 0; k < sizeOfList; k++) {
//         ESP_LOGI("RottaryEnc", "Element %d = %d", k, numbers[k]);
//     }

//     ESP_LOGI(TAG, "install pcnt unit");
//     pcnt_unit_handle_t pcnt_unit = NULL;// INFO  I TOOK IT FROM BELO
//     pcnt_unit_config_t unit_config = {
//         .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
//         .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
//     };
   
//     ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

//     ESP_LOGI(TAG, "install pcnt channels");
//     pcnt_chan_config_t chan_a_config = {
//         .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
//         .level_gpio_num = EXAMPLE_EC11_GPIO_B,
//     };
//     pcnt_channel_handle_t pcnt_chan_a = NULL;
//     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
//     pcnt_chan_config_t chan_b_config = {
//         .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
//         .level_gpio_num = EXAMPLE_EC11_GPIO_A,
//     };
//     pcnt_channel_handle_t pcnt_chan_b = NULL;
//     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

//     ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
//     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
//     ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
//     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
//     ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));


//     ESP_LOGI(TAG, "enable pcnt unit");
//     ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
//     ESP_LOGI(TAG, "clear pcnt unit");
//     ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
//     ESP_LOGI(TAG, "start pcnt unit");
//     ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

// #if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
//     // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
//     ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
//     ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
//     ESP_ERROR_CHECK(esp_light_sleep_start());
// #endif
//     int pulse_count = 0;
//     for (;;) {

//             ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
//                     if (pulse_count % numbers[0] == 0 && pulse_count>0) {
//                             xSemaphoreGive(xBinarySemaphoreClock);
//                             pcnt_unit_clear_count(pcnt_unit);
//                             int x = numbers[0];
//                             for(int i=0; i<sizeOfList - 1; i++){
//                                 numbers[i]=numbers[i+1];
//                             }
//                             numbers[sizeOfList-1] = x;

//                         } else if (pulse_count % numbers[0] == 0 && pulse_count < 0){
//                             xSemaphoreGive(xBinarySemaphoreCounterClock);
//                             pcnt_unit_clear_count(pcnt_unit);
//                             int x = numbers[0];
//                             for(int i=0; i<sizeOfList - 1; i++){
//                                 numbers[i]=numbers[i+1];
//                             }
//                             numbers[sizeOfList-1] = x;
//                         } else{;;}
//                          esp_task_wdt_reset();
//                     }
// }




// static int gcd(int a, int b)
// {
//     if (b == 0) return a;
//     return gcd(b, a % b);
// }



// void SetNVS(){
//     esp_err_t err = nvs_flash_init();
//     if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         err = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(err);


//     nvs_handle_t my_handle;
//     err = nvs_open(STORAGE, NVS_READWRITE, &my_handle);
//     if (err != ESP_OK) {
//         printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
//     } else {
//         printf("NVS handle opened successfully.\n");
//     }


//     int32_t counter = 42;

//     err = nvs_set_i32(my_handle, "my_counter", counter);// my_counter is the variable counter is the number
//     if (err == ESP_OK) {
//         // Commit written value.
//         err = nvs_commit(my_handle);
//         if (err != ESP_OK) {
//             printf("Error committing to NVS!\n");
//         }
//     } else {
//         printf("Error setting value in NVS!\n");
//     }

// }


// int GetNVS(){
//     esp_err_t err = nvs_flash_init();
//     if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         err = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(err);


//     nvs_handle_t my_handle;
//     err = nvs_open(STORAGE, NVS_READWRITE, &my_handle);
//     if (err != ESP_OK) {
//         printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
//     } else {
//         printf("NVS handle opened successfully.\n");
//     }


//     int32_t stored_value = 0;
//     err = nvs_get_i32(my_handle, "my_counter", &stored_value);
//     switch (err) {
//         case ESP_OK:
//             printf("The stored counter value is: %ld\n", stored_value);
//             break;
//         case ESP_ERR_NVS_NOT_FOUND:
//             printf("The value is not initialized yet!\n");
//             break;
//         default :
//             printf("Error (%s) reading!\n", esp_err_to_name(err));
//     }

//     nvs_close(my_handle);
//     return stored_value;

// }

// void app_main() {


//     SetNVS();
//     int x = GetNVS();
//     printf(" x : %d!\n",x);

//     // vTaskDelay(100000);

//    // Wait for system to stabilize
//    esp_task_wdt_config_t wd_config = {
//     .timeout_ms =5000,
//     .idle_core_mask=0,
//     .trigger_panic=true,
//    };
//      esp_err_t ret = esp_task_wdt_reconfigure(&wd_config); // Apply configuration
//     if (ret != ESP_OK) {
//         printf("Task Watchdog Timer reconfiguration failed: %d\n", ret);
//     }

//     // Check and print the APB clock frequency
//     xBinarySemaphoreClock = xSemaphoreCreateBinary();
//     if (xBinarySemaphoreClock == NULL) {
//         ESP_LOGE("app_main", "Failed to create semaphore");
//         return; // Abort if semaphore creation fails
//     }
//         xBinarySemaphoreCounterClock = xSemaphoreCreateBinary();
//     if (xBinarySemaphoreCounterClock == NULL) {
//         ESP_LOGE("app_main", "Failed to create semaphore");
//         return; // Abort if semaphore creation fails
//     }




//     // We want to express ratio = pulses / steps in simplest form.
//     // ratio = 4000 / 175

//     // 1) Compute GCD


//     float fRleadScrew = StepperSteps * ((float)DesirePitch / (float)LeadScrewPitch);
//     int RleadScrew = (int)fRleadScrew;
   
//     int divisor = gcd(SpindlePulses, RleadScrew);

//     // 2) Divide both by GCD to get reduced fraction
//     int numerator   = SpindlePulses / divisor;  // e.g. 4000 / 25 = 160
//     int denominator = RleadScrew  / divisor;  // e.g. 175 / 25  = 7

//     // 3) Print results
//     printf("Ratio (pulses/steps) = %d/%d Divisor = %d\n", numerator, denominator, divisor);

//     // The fraction is 160/7, meaning:
//     // - For every 7 'step blocks', we have 160 'pulse blocks'.
//     // - Repeated enough times => 175 steps, 4000 pulses.

//     int blockSteps  = denominator; // 7
//     int blockPulses = numerator;   // 160

//     printf("Block size (steps) = %d, Pulses in that block = %d\n",
//            blockSteps, blockPulses);

//     // Keep this task alive
    
//     int *blockArray = malloc(blockSteps * sizeof(int));
//     if (!blockArray) {
//         ESP_LOGE("app_main", "Failed to allocate memory for blockArray");
//         return; // handle error gracefully
//     }

//     int FirstEntries = numerator / blockSteps;
//     ESP_LOGE("app_main", "FirstEnttries: %d", FirstEntries);
//     for (int i =0; i<blockSteps; i++){
//         blockArray[i]=FirstEntries;
//     }
//     int Remaining = blockPulses - (FirstEntries *blockSteps);
//     ESP_LOGE("app_main", "Remaining: %d", Remaining);
//     if (Remaining>0){
//         int counter = 0;
//         for (int j=0; j<Remaining; j++){
//             blockArray[counter]++;
//             counter++;
//             if (counter>blockSteps){
//                 counter =0;
//             }
//         }
//     }

//     for(int k=0; k<blockSteps; k++){
//         ESP_LOGE("app_main", "Element: %d, Size: %d",k,blockArray[k]);
//     }
//     // vTaskDelay(1000000);

//     block_data_t *params = malloc(sizeof(block_data_t));
//     if (!params) {
//         ESP_LOGE("app_main", "Failed to allocate memory for params");
//         free(blockArray);
//         return; // handle error
//     }

//     params->arr    = blockArray;   // store the pointer
//     params->length = blockSteps;   // store the length

//     // Create the task and pin it to core 1
//     xTaskCreatePinnedToCore(
//         TaskStepperClock,            // Function that implements the task
//         "TaskStepperClock",      // Name of the task
//         4096,             // Stack size
//         NULL,             // Task input parameter
//         1,                // Priority of the task
//         NULL,             // Task handle
//         tskNO_AFFINITY
//         // 1                 // Core where the task should run (1 for core 1, 0 for core 0)

//     );


//     xTaskCreatePinnedToCore(
//         TaskStepperCounterClock,            // Function that implements the task
//         "TaskStepperCounterClock",      // Name of the task
//         4096,             // Stack size
//         NULL,             // Task input parameter
//         1,                // Priority of the task
//         NULL,             // Task handle
//         tskNO_AFFINITY
//         // 1                 // Core where the task should run (1 for core 1, 0 for core 0)

//     );


//     xTaskCreatePinnedToCore(  
//         TaskRottary,
//         "RotaryEncoder",
//         4096,
//         (void *)params,
//         1,
//         NULL,
//         tskNO_AFFINITY
//         // 0
//     );
// }



////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#define SELECTOR_GPIO_A 36
#define SELECTOR_GPIO_B 37
#define SELECTOR_GPIO_BUTTON 40

#define EXAMPLE_PCNT_HIGH_LIMIT 80
#define EXAMPLE_PCNT_LOW_LIMIT  -80

static const char *TAG = "I2C_LCD";

/* I2C configuration */
#define I2C_MASTER_SCL_IO          6   // GPIO for SCL signal
#define I2C_MASTER_SDA_IO          7   // GPIO for SDA signal
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000 // Standard 100kHz
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0

/* LCD backpack (PCF8574) I2C address.
   Common addresses are 0x27 or 0x3F. Adjust to match your module. */
#define LCD_ADDR                   0x27

/* For a 16x2 or 20x4 HD44780-based LCD in 4-bit mode */
#define LCD_COLS                   20
#define LCD_ROWS                   4

/* Commands / Flags (HD44780-style) */
#define LCD_CMD_CLEAR_DISPLAY      0x01
#define LCD_CMD_RETURN_HOME        0x02
#define LCD_CMD_ENTRY_MODE_SET     0x04
#define LCD_CMD_DISPLAY_CONTROL    0x08
#define LCD_CMD_FUNCTION_SET       0x20
#define LCD_CMD_SET_DDRAM_ADDR     0x80

#define LCD_FLAG_DISPLAY_ON        0x04
#define LCD_FLAG_DISPLAY_OFF       0x00
#define LCD_FLAG_BLINK_ON          0x01
#define LCD_FLAG_CURSOR_ON         0x02
#define LCD_FLAG_2LINE             0x08
#define LCD_FLAG_5x8DOTS           0x00
#define LCD_FLAG_4BITMODE          0x00

/* PCF8574 pin mapping for LCD backpack:
   Often these bits map to:
      D7 -> P7
      D6 -> P6
      D5 -> P5
      D4 -> P4
      BL -> P3 (backlight)
      EN -> P2
      RW -> P1
      RS -> P0
   This can vary by module, but the below is common. */
#define PCF8574_RS   0x01  // P0
#define PCF8574_RW   0x02  // P1
#define PCF8574_EN   0x04  // P2
#define PCF8574_BL   0x08  // P3 (backlight on = 1)
                          // D4->P4, D5->P5, D6->P6, D7->P7

///////////////////////////////////////////
#define ALL_MENU 9
#define MAIN_MENU_ITEMS 4 
#define POWER_FEED_MENU_ITEMS 4
#define THREADS_MENU_ITEMS 4 
#define SETTINGS_MENU_ITEMS 5 
//////////////////////////////////////////////
#define METRIC_MENU_ITEMS 4
#define TPI_MENU_ITEMS 4
////////////////////////////////////////////////
#define SET_ROTARY_ENCODER_MENU_ITEMS 4
#define SET_LEAD_SCREW_MENU_ITEMS 4
#define SET_STEPPER_MOTOR_MENU_ITEMS 4



int powerFeedValue=0;
int MetricValue  =0;
int TPIValue =0;
int RottaryEncoderValue =0;
int LeadScrewValue =0;
int StepperMotorValue=0;


int path[ALL_MENU]={0,1,1,1,1,1,1,1,1};

int PathMenu=1;// The menu that goes up and down the ">"
int Selection =1;
const char *Main_Menu[MAIN_MENU_ITEMS]  = {"Main Menu","Power Feed", "Threads", "Settings"};
const char *Threads_Menu[THREADS_MENU_ITEMS]  = {"Threads Menu","Metric", "TPI", "Return"};
const char *Settings_Menu[SETTINGS_MENU_ITEMS]  = {"Settings Menu","Set Rot. Encoder", "Set Lead Screw", "Set Stepper Motor", "Return"};
const char *OK_RETURN[4]={"","  Value : ","OK","Return"};
static uint8_t pcf8574_mask = PCF8574_BL; // Start with backlight ON

pcnt_unit_handle_t pcnt_unit = NULL;

void CreateMenu();

void ButtonPressed(void *pvParameters)
{
    // Configure the GPIO pin as input with pull-up
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // Disable interrupt
    io_conf.mode = GPIO_MODE_INPUT;       // Set as input mode
    io_conf.pin_bit_mask = (1ULL << SELECTOR_GPIO_BUTTON); // Bit mask for the GPIO pin
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     // Enable pull-up
    gpio_config(&io_conf);
 bool button_pressed = false; // Flag to track button press state
    while (1) {
        // Read the button state
        int button_state = gpio_get_level(SELECTOR_GPIO_BUTTON );

        if (button_state == 0) {
            if (!button_pressed) { // Check if the button was not already pressed
                printf("Button pressed!\n");
                button_pressed = true; // Set the flag to indicate the button is pressed
            }
        } else {
            if (button_pressed) { // Check if the button was previously pressed
                if (path[0] ==0){
                    if (path[1]==1){
                        // ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
                        path[0]=1;
                        path[1]=1;
                        PathMenu=2;
                        CreateMenu();
                    } else if (path[1]==2){
                        // ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
                        path[0]=2;
                        path[1]=1;
                        PathMenu=3;
                        CreateMenu();
                    } else if (path[1]==3){
                        // ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
                        path[0]=3;
                        path[1]=1;
                        PathMenu=4;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==1){//INFO POWER FEED
                    if (path[2]==1){
                        printf("Get Counter");
                    } else if (path[2]==2){
                        printf("RUN");
                    } else if (path[2]==3){ // INFO RETURN TO MAIN MENU
                        path[0]=0;
                        path[2]=1; 
                        PathMenu=1;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==2){//INFO THREADS MENY
                    if (path[3]==1){
                        path[0]=4;
                        path[3]=1;
                        PathMenu=5;
                        CreateMenu();
                    } else if (path[3]==2){
                        path[0]=5;
                        path[3]=1;
                        PathMenu=6;
                        CreateMenu();
                    } else if (path[3]==3){//INFO RETURN TO MAIN MENU
                        path[0]=0;
                        path[3]=1; 
                        PathMenu=1;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////APO EDO KAI KATO 
                }else if(path[0] ==3){//INFO Settings Menu 
                    if (path[4]==1){//BUG 
                        path[0]=4;
                        path[3]=1;
                        PathMenu=5;
                        CreateMenu();
                    } else if (path[4]==2){
                        path[0]=5;
                        path[3]=1;
                        PathMenu=6;
                        CreateMenu();
                    } else if (path[4]==3){//INFO RETURN TO MAIN MENU
                        path[0]=0;
                        path[3]=1; 
                        PathMenu=1;
                        CreateMenu();
                    } else if (path[4]==4){//INFO RETURN TO MAIN MENU
                        path[0]=0;
                        path[3]=1; 
                        PathMenu=1;
                        CreateMenu();
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==4){//INFO METRIC THREADS
                    if (path[5]==1){
                        printf("VALUES");
                    } else if (path[3]==2){
                        printf("RUN");
                    } else if (path[3]==3){//INFO RETURN TO MAIN MENU
                        path[0]=2;
                        path[5]=1; 
                        PathMenu=3;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==5){//INFO TPI MENY
                    if (path[6]==1){
                        printf("VALUES");
                    } else if (path[6]==2){
                        printf("RUN");
                    } else if (path[6]==3){//INFO RETURN TO MAIN MENU
                        path[0]=2;
                        path[6]=1; 
                        PathMenu=3;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==6){//INFO THREADS MENY
                    if (path[3]==1){
                        path[0]=4;
                        path[3]=1;
                        PathMenu=5;
                        CreateMenu();
                    } else if (path[3]==2){
                        path[0]=5;
                        path[3]=1;
                        PathMenu=6;
                        CreateMenu();
                    } else if (path[3]==3){//INFO RETURN TO MAIN MENU
                        path[0]=0;
                        path[3]=1; 
                        PathMenu=1;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==7){//INFO THREADS MENY
                    if (path[3]==1){
                        path[0]=4;
                        path[3]=1;
                        PathMenu=5;
                        CreateMenu();
                    } else if (path[3]==2){
                        path[0]=5;
                        path[3]=1;
                        PathMenu=6;
                        CreateMenu();
                    } else if (path[3]==3){//INFO RETURN TO MAIN MENU
                        path[0]=0;
                        path[3]=1; 
                        PathMenu=1;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==7){//INFO THREADS MENY
                    if (path[3]==1){
                        path[0]=4;
                        path[3]=1;
                        PathMenu=5;
                        CreateMenu();
                    } else if (path[3]==2){
                        path[0]=5;
                        path[3]=1;
                        PathMenu=6;
                        CreateMenu();
                    } else if (path[3]==3){//INFO RETURN TO MAIN MENU
                        path[0]=0;
                        path[3]=1; 
                        PathMenu=1;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                    }
                }
                button_pressed = false; // Reset the flag
            }
        }
        // Add a delay to avoid bouncing and flooding the console
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


void SelectorCounter(void *pvParameters)
{
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));


    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = SELECTOR_GPIO_A,
        .level_gpio_num = SELECTOR_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = SELECTOR_GPIO_B,
        .level_gpio_num = SELECTOR_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");

    // QueueHandle_t queue = xQueueCreate(10, sizeof(int));

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
    int prev_count = 0;
    while (1) {
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));

        if (prev_count != pulse_count){
            
            if (pulse_count %4 ==0) {
            printf("Pulse Count: %d\n", pulse_count);

            // Determine the direction of rotation
            if (prev_count < pulse_count) {
                // Clockwise rotation
                Selection++;
                if (Selection > 3) {
                    Selection = 1;
                }
            } else {
                // Counter-clockwise rotation
                Selection--;
                if (Selection < 1) {
                    Selection = 3;
                }
            }

            printf("PathMenu %d Selection: %d Pulse Count: %d PrevCount: %d\n", PathMenu, Selection, pulse_count, prev_count);
            path[PathMenu] = Selection;
            CreateMenu();
            prev_count = pulse_count;

            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}


/**
 * @brief Wrapper to write a single byte to the PCF8574 over I2C.
 */
static esp_err_t i2c_lcd_write_byte(uint8_t data)
{
    // The data we send includes the "pcf8574_mask" (for BL, etc.)
    uint8_t out = data | pcf8574_mask;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, out, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Pulse the EN pin to latch data into the LCD.
 */
static void lcd_pulse_enable(uint8_t nibble)
{
    i2c_lcd_write_byte(nibble | PCF8574_EN);
    // EN = 1, small delay
    vTaskDelay(pdMS_TO_TICKS(1));

    i2c_lcd_write_byte(nibble & ~PCF8574_EN);
    // EN = 0, small delay
    vTaskDelay(pdMS_TO_TICKS(1));
}

/**
 * @brief Send 4-bit nibble to LCD.
 */
static void lcd_write_nibble(uint8_t nibble, bool is_command)
{
    // RS bit: 0 = command, 1 = data
    uint8_t rs_mask = is_command ? 0 : PCF8574_RS;

    // Data lines are on upper nibble of the byte (D4-D7 -> P4-P7)
    // Shift nibble into upper bits, preserve RS and BL in lower bits
    uint8_t data = (nibble << 4) & 0xF0;
    data |= rs_mask;

    // Write out the nibble, then pulse enable
    lcd_pulse_enable(data);
}

/**
 * @brief Send a full byte to LCD (splits into two nibbles).
 */
static void lcd_write_byte(uint8_t val, bool is_command)
{
    lcd_write_nibble((val >> 4) & 0x0F, is_command); // high nibble
    lcd_write_nibble(val & 0x0F, is_command);        // low nibble
}

/**
 * @brief Send command byte to LCD.
 */
static inline void lcd_cmd(uint8_t cmd)
{
    lcd_write_byte(cmd, true);
}

/**
 * @brief Send data (character) byte to LCD.
 */
static inline void lcd_data(uint8_t data)
{
    lcd_write_byte(data, false);
}

/**
 * @brief Initialize the LCD in 4-bit mode.
 */
static void lcd_init(void)
{
    // The HD44780 datasheet recommends a specific power-on sequence.

    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for LCD to power up

    // Try to set 4-bit mode: we send 0x03 multiple times
    // while in 8-bit mode, to ensure the LCD enters 4-bit mode.
    lcd_write_nibble(0x03, true);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, true);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, true);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Now instruct the LCD to switch to 4-bit mode
    lcd_write_nibble(0x02, true);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Function Set: 4-bit mode, 2 lines, 5x8 dots
    lcd_cmd(LCD_CMD_FUNCTION_SET | LCD_FLAG_4BITMODE | LCD_FLAG_2LINE | LCD_FLAG_5x8DOTS);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Display Control: display on, cursor off, blink off
    lcd_cmd(LCD_CMD_DISPLAY_CONTROL | LCD_FLAG_DISPLAY_ON);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Clear display
    lcd_cmd(LCD_CMD_CLEAR_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Entry mode set: increment automatically, no shift
    lcd_cmd(LCD_CMD_ENTRY_MODE_SET | 0x02);
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "LCD initialized in 4-bit mode.");
}

/**
 * @brief Move the cursor to a specific row and column (0-based).
 */
static void lcd_set_cursor(uint8_t col, uint8_t row)
{
    if (row >= LCD_ROWS) row = 0;  // safety
    // On many 16x2 or 20x4 displays:
    // Row 0 DDRAM address = 0x00
    // Row 1 DDRAM address = 0x40
    // (Row 2 = 0x14, Row 3 = 0x54 for 20x4, etc.)
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_cmd(LCD_CMD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

/**
 * @brief Print a string on the LCD from current cursor position.
 */
static void lcd_print(const char *str)
{
    int count = 0;
    while (*str && count<20) {
        lcd_data((uint8_t)(*str));
        str++;
        count++;
    }
}

/**
 * @brief Initialize I2C master driver.
 */
static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0, // optional in older IDF versions
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM,
                       conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE,
                       0);

    ESP_LOGI(TAG, "I2C master initialized on SDA=%d, SCL=%d at %d Hz",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
}



static void PrintSettingsMenu(int pos, int menu){
    lcd_cmd(LCD_CMD_CLEAR_DISPLAY);
        
    int items = 0;
    const char **menuToPrint = NULL; 
    if (menu == 0){
        items = MAIN_MENU_ITEMS;
        menuToPrint= Main_Menu;
        // value = powerFeedValue;
    } else if (menu == 2){
         items = THREADS_MENU_ITEMS;
          menuToPrint = Threads_Menu;
    } else if (menu == 3){
         items = SETTINGS_MENU_ITEMS;
         menuToPrint = Settings_Menu;
    }

    for (int i = 0; i < items; i++) {
        lcd_set_cursor(0, i); // Move cursor to row i
        if (i==0){

            lcd_print(menuToPrint[i]);
        } else{// Print the menu item
            if (i == pos) {
                lcd_print("> "); // Highlight the selected item
             } else {
                lcd_print("  "); // Add spaces for alignment
            }
            lcd_print(menuToPrint[i]);
        }
    }
}

static void PrintValuesMenu(int pos, int menu ){
    lcd_cmd(LCD_CMD_CLEAR_DISPLAY);
    int items =0;
    int value = 0;
    char valueStr[16];

    if (menu == 1){
        OK_RETURN[0] = "Power Feed Menu";
        value = powerFeedValue;
        items = POWER_FEED_MENU_ITEMS;
    } else if (menu == 4){
         OK_RETURN[0] = "Metric Threads";
         value = MetricValue;
         items = METRIC_MENU_ITEMS;
    } else if (menu == 5){
         OK_RETURN[0] = "TPI";
         value=TPIValue;
         items=TPI_MENU_ITEMS;
    }else if (menu == 6){
         OK_RETURN[0] = "Rotary Encoder";
         items=SET_ROTARY_ENCODER_MENU_ITEMS;
         value=RottaryEncoderValue;
    }else if (menu == 7){
         OK_RETURN[0] = "Lead Screw";
         value=LeadScrewValue;
    }else if (menu == 8){
         OK_RETURN[0] = "Stepper Motor";
         value=StepperMotorValue;
    }
    printf("MENU[0] = %s, MENU[1] = %s, MENU[2] = %s\n",OK_RETURN[0],OK_RETURN[1],OK_RETURN[2] );
    for (int i = 0; i < items; i++) {
        lcd_set_cursor(0, i); // Move cursor to row i
        if (i == pos) {
            lcd_print("> "); // Highlight the selected item
        } else {
            lcd_print("  "); // Add spaces for alignment
        }
        if (i==1){
            sprintf(valueStr, "%d", value); // Convert integer to string
            lcd_print(OK_RETURN[i]);
            lcd_print(valueStr);
        } else {
            lcd_print(OK_RETURN[i]);
        } // Print the menu item
    }
}

void CreateMenu(){
printf("path[0] = %d, path[1] = %d, path[2] = %d, path[3] = %d, path[4] = %d, path[5] = %d, path[6] = %d, path[7] = %d, path[8] = %d \n", path[0], path[1], path[2], path[3], path[4], path[5], path[6], path[7], path[8]);
         switch (path[0])
    {
////////////////////////////////////////////////////////////////////////////////
    case 0: //PRINT MAIN MENU
        switch (path[1])
        {
        case 1: 
            PrintSettingsMenu(1,path[0]);
            break;
        case 2:
            PrintSettingsMenu(2,path[0]);
            break;
        case 3:
            PrintSettingsMenu(3,path[0]);
            break;
        
        }
        break;
/////////////////////////////////////////////////////////////////////////
    case 1://PRINT POWER FEED MENU
         switch (path[2]){
            case 1:
                PrintValuesMenu(path[2], path[0]);
                break;
            case 2:
                PrintValuesMenu(path[2], path[0]);
                break;
             case 3:
                PrintValuesMenu(path[2] ,path[0] );
                break;
            // break;
         }
        break;
/////////////////////////////////////////////////////////////////////////
    case 2://PRINT THREADS MENU
        switch (path[3])
        {
            case 1:
                PrintSettingsMenu(1,path[0]);
                break;
            case 2:
                PrintSettingsMenu(2,path[0]);
                break;
            case 3:
                PrintSettingsMenu(3,path[0]);
                break;

        }
        break;
/////////////////////////////////////////////////////////////////////////
    case 3://PRINT SETTINGS MENU
        switch (path[4])
        {
            case 1:
                PrintSettingsMenu(0,path[0]);
                break;
            case 2:
                PrintSettingsMenu(1,path[0]);
                break;
            case 3:
                PrintSettingsMenu(2,path[0]);
                break;
            case 4:
                PrintSettingsMenu(3,path[0]);
                break;
        }
        break;
/////////////////////////////////////////////////////////////////////////
    case 4://PRINT METRIC THREADS
        switch (path[5])
        {
            case 0:
                PrintValuesMenu(0,path[0]);
                break;
            case 1:
                PrintValuesMenu(2,path[0]);
                break;
            case 2:
                PrintValuesMenu(3,path[0]);
                break;
        }
        break;
/////////////////////////////////////////////////////////////////////////
    case 5://PRINT TPI THREADS
        switch (path[6])
        {
            case 0:
                PrintValuesMenu(0,path[0]);
                break;
            case 1:
                PrintValuesMenu(2,path[0]);
                break;
            case 2:
                PrintValuesMenu(3,path[0]);
                break;
        }
        break;
/////////////////////////////////////////////////////////////////////////
    case 6://PRINT SET ROTTARY ENCODER PPR
        switch (path[7])
        {
            case 0:
                PrintValuesMenu(0,path[0]);
                break;
            case 1:
                PrintValuesMenu(2,path[0]);
                break;
            case 2:
                PrintValuesMenu(3,path[0]);
                break;
        }
        break;
/////////////////////////////////////////////////////////////////////////
    case 7://PRINT SET LEAD SCREW PITCH
        switch (path[8])
        {
            case 0:
                PrintValuesMenu(0,path[0]);
                break;
            case 1:
                PrintValuesMenu(2,path[0]);
                break;
            case 2:
                PrintValuesMenu(3,path[0]);
                break;
        }
        break;
/////////////////////////////////////////////////////////////////////////
    case 8://PRINT SET STEPPER MOTOR
        switch (path[9])
        {
            case 0:
                PrintValuesMenu(0,path[0]);
                break;
            case 1:
                PrintValuesMenu(2,path[0]);
                break;
            case 2:
                PrintValuesMenu(3,path[0]);
                break;
        }
    default:
        break;
    }
}

void app_main(void)
{
    // 1. Initialize I2C bus
    i2c_master_init();

    // 2. Initialize the LCD
    lcd_init();

    // 3. Write something to the LCD
    
    CreateMenu();
    // lcd_set_cursor(0, 0);
    // lcd_print("KYRIAKO");
    // lcd_set_cursor(0, 1);
    // lcd_print("GAMIESE");
    // lcd_set_cursor(0, 2);
    // lcd_print("KYRIAKO");
    // lcd_set_cursor(0, 3);
    // lcd_print("GAMIESE");

    xTaskCreatePinnedToCore(
        SelectorCounter,            // Function that implements the task
        "SelectorCounter",      // Name of the task
        4096,             // Stack size
        NULL,             // Task input parameter
        1,                // Priority of the task
        NULL,             // Task handle
        tskNO_AFFINITY
        // 1                 // Core where the task should run (1 for core 1, 0 for core 0)

    );

        xTaskCreatePinnedToCore(
        ButtonPressed,            // Function that implements the task
        "ButtonPressed",      // Name of the task
        4096,             // Stack size
        NULL,             // Task input parameter
        1,                // Priority of the task
        NULL,             // Task handle
        tskNO_AFFINITY
        // 1                 // Core where the task should run (1 for core 1, 0 for core 0)

    );
    // Loop forever
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
