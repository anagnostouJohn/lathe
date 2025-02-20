

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
#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"


#include <string.h>
#include "driver/i2c.h"
#include "freertos/queue.h"






SemaphoreHandle_t xStopTaskBSemaphore = NULL;
QueueHandle_t xStartTasksQueue = NULL;
SemaphoreHandle_t xBinarySemaphoreClock;
SemaphoreHandle_t xBinarySemaphoreCounterClock;

TaskHandle_t TaskA_Handle = NULL;
TaskHandle_t TaskB_Handle = NULL;

void SetNVS(const char* , int32_t );
int GetNVS(const char* );
void CreateMenu();
void InitializeValues();

#define STORAGE "Data"


static const char *TAGLCD = "I2C_LCD";
static const char *TAGLATHE = "Lathe";

#define SELECTOR_GPIO_A 4           //changed GPIO to match PCB
#define SELECTOR_GPIO_B 5            //changed GPIO to match PCB
#define SELECTOR_GPIO_BUTTON 6        //changed GPIO to match PCB
#define EXAMPLE_EC11_GPIO_A 14         //changed GPIO to match PCB
#define EXAMPLE_EC11_GPIO_B 13          //changed GPIO to match PCB
#define DirPin GPIO_NUM_17               //changed GPIO to match PCB
#define PulcePin GPIO_NUM_16              //changed GPIO to match PCB


#define EXAMPLE_PCNT_HIGH_LIMIT 80
#define EXAMPLE_PCNT_LOW_LIMIT  -80



/* I2C configuration */
#define I2C_MASTER_SCL_IO          9   // GPIO for SCL signal *changed to default I2C SCL GPIO to match PCB 
#define I2C_MASTER_SDA_IO          8   // GPIO for SDA signal *changed to default I2C SCL GPIO to match PCB
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
#define ALL_MENU 10
#define MAIN_MENU_ITEMS 4 
#define POWER_FEED_MENU_ITEMS 4
#define THREADS_MENU_ITEMS 4 
#define SETTINGS_MENU_ITEMS 5 
//////////////////////////////////////////////
#define METRIC_MENU_ITEMS 4
#define TPI_MENU_ITEMS 4
////////////////////////////////////////////////
#define SET_ROTARY_ENCODER_MENU_ITEMS 3
#define SET_LEAD_SCREW_MENU_ITEMS 3
#define SET_STEPPER_MOTOR_MENU_ITEMS 3



float powerFeedValue=0.00;      // changed to zero to force user to select value
float metricThreadsValue =0.00;  // changed to zero to force user to select value
int tpiThreadsValue =0;
int rottaryEncoderValues=0;
int leadScrewValue=0;
int stepperMotorValue=0;


int FinalmetricThreadsValue =0;
int FinalrottaryEncoderValues;
int FinalleadScrewValue;
int FinalstepperMotorValue;

bool setSelection = true;
bool setPowerFeed = false;
bool setMetricThreads = false;
bool setTPIthreads = false;
bool setRottaryEncoder = false;
bool setleadScrew = false;
bool setStepperMotor=false;
bool powerFeedRun = false;
bool metricThreadsRun = false;
bool tpiThreadsRun = false;




int path[ALL_MENU]={0,1,1,1,1,1,1,1,1,1};

int PathMenu=1;// The menu that goes up and down the ">"
int Selection =1;
int maxRottarySelection = 3;
const char *Main_Menu[MAIN_MENU_ITEMS]  = {"   -*Main Menu*- ","Power Feed", "Threads", "Settings"};
const char *Threads_Menu[THREADS_MENU_ITEMS]  = {"  -*Threads Menu*-","Metric", "TPI", "Return"};
const char *Settings_Menu[SETTINGS_MENU_ITEMS]  = {" -*Settings Menu*-","Set Rot. Encoder", "Set Lead Screw", "Set Stepper Motor", "Return"};
const char *OK_RETURN[4]={"","Value : ","OK","Return"};
const char *RETURN[3]={"","Value : ","Return"};
const char *TYPE[1]={""};
static uint8_t pcf8574_mask = PCF8574_BL; // Start with backlight ON

pcnt_unit_handle_t pcnt_unit = NULL;

typedef struct {
    int *arr;      // pointer to the dynamically allocated array
    int length;    // how many elements are in arr
} block_data_t;




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

    ESP_LOGI(TAGLCD, "LCD initialized in 4-bit mode.");
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

    ESP_LOGI(TAGLCD, "I2C master initialized on SDA=%d, SCL=%d at %d Hz",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
}



static void PrintSettingsMenu(int pos, int menu){
    lcd_cmd(LCD_CMD_CLEAR_DISPLAY);
        
    int items = 0;
    const char **menuToPrint = NULL; 
    if (menu == 0){
        items = MAIN_MENU_ITEMS;
        menuToPrint= Main_Menu;
    } else if (menu == 2){
         items = THREADS_MENU_ITEMS;
          menuToPrint = Threads_Menu;
    } else if (menu == 3){
         items = SETTINGS_MENU_ITEMS;
         menuToPrint = Settings_Menu;
    }
    if ((menu==0)||(menu==2)){
        for (int i = 0; i < items; i++) {
            lcd_set_cursor(0, i); // Move cursor to row i
            
                if (i==0){

                    lcd_print(menuToPrint[i]);
                } else{// Print the menu item
                    if (i == pos) {
                        lcd_print(">"); // Highlight the selected item
                    } else {
                        lcd_print(" "); // Add spaces for alignment
                    }
                    lcd_print(menuToPrint[i]);
                }
            } 
        } else if(menu==3){
            if (path[4]==1){
                lcd_set_cursor(0, 0); 
                lcd_print(menuToPrint[0]);
                lcd_set_cursor(0, 1);
                lcd_print(">"); 
                lcd_print(menuToPrint[1]);
                lcd_set_cursor(0, 2);
                lcd_print(" "); 
                lcd_print(menuToPrint[2]);
                lcd_set_cursor(0, 3);
                lcd_print(" "); 
                lcd_print(menuToPrint[3]);
            } else if (path[4]==2){
                lcd_set_cursor(0, 0);
                lcd_print(menuToPrint[0]);
                lcd_set_cursor(0, 1);
                lcd_print(" "); 
                lcd_print(menuToPrint[1]);
                lcd_set_cursor(0, 2);
                lcd_print(">"); 
                lcd_print(menuToPrint[2]);
                lcd_set_cursor(0, 3);
                lcd_print(" "); 
                lcd_print(menuToPrint[3]);
        }else if (path[4]==3){
                lcd_set_cursor(0, 0);
                lcd_print(menuToPrint[0]);
                lcd_set_cursor(0, 1);
                lcd_print(" "); 
                lcd_print(menuToPrint[1]);
                lcd_set_cursor(0, 2);
                lcd_print(" "); 
                lcd_print(menuToPrint[2]);
                lcd_set_cursor(0, 3);
                lcd_print(">"); 
                lcd_print(menuToPrint[3]);
        }else if (path[4]==4){
                lcd_set_cursor(0, 0);
                lcd_print(menuToPrint[0]);
                lcd_set_cursor(0, 1);
                lcd_print(" "); 
                lcd_print(menuToPrint[2]);
                lcd_set_cursor(0, 2);
                lcd_print(" "); 
                lcd_print(menuToPrint[3]);
                lcd_set_cursor(0, 3);
                lcd_print(">"); 
                lcd_print(menuToPrint[4]);
        }
    }

}

static void PrintRun(){
    char valueStr[16];
    //int ivalue;
    lcd_cmd(LCD_CMD_CLEAR_DISPLAY);
    lcd_set_cursor(0, 0); // Move cursor to row i
    lcd_print(">>>>>> RUNNING <<<<<"); // Highlight the selected item    
    
    if (metricThreadsRun == true){      // added to indicate the selected metric thread
        lcd_set_cursor(0,1);
        lcd_print ("Metric Thread");
        sprintf(valueStr, "%.2f", metricThreadsValue); // Convert integer to string
         lcd_set_cursor(14,1);
         lcd_print(valueStr);
         lcd_set_cursor(18,1);
         lcd_print("mm");
    }
    else if (powerFeedRun == true)      // added to indicate the selected power feed
    {
        lcd_set_cursor(0,1);
        lcd_print ("Power Feed");
        sprintf(valueStr, "%.2f", powerFeedValue); // Convert integer to string
         lcd_set_cursor(14,1);
         lcd_print(valueStr);
         lcd_set_cursor(18,1);
         lcd_print("mm");
    }
    else if (tpiThreadsRun == true)     // added to indicate the selected TPI thread
    {
        lcd_set_cursor(0,1);
        lcd_print ("TPI Thread");
        sprintf(valueStr, "%.d", tpiThreadsValue); // Convert integer to string
         lcd_set_cursor(14,1);
         lcd_print(valueStr);
         lcd_set_cursor(17,1);
         lcd_print("TPI");
    }
        lcd_set_cursor (0,3);
        lcd_print(">");
        lcd_set_cursor (1,3);
        lcd_print("Main Menu");
       
        
    
     
}

static void PrintValuesMenu(int pos, int menu ){
    lcd_cmd(LCD_CMD_CLEAR_DISPLAY);
    int items =0;
    float fvalue = 0.0;
    int ivalue=0;
    char valueStr[16];

    if (menu == 1){
        OK_RETURN[0] = "-*Power Feed Menu*-";
        fvalue = powerFeedValue;
        items = POWER_FEED_MENU_ITEMS;
        TYPE[0] = "mm";             // Changed to correct description
    } else if (menu == 4){
         OK_RETURN[0] = "Metric Threads";
         fvalue = metricThreadsValue;
         items = METRIC_MENU_ITEMS;
        TYPE[0] = "mm";         // Changed to correct description
    } else if (menu == 5){
         OK_RETURN[0] = "TPI";
         ivalue=tpiThreadsValue;
         items=TPI_MENU_ITEMS;
         TYPE[0] = "TPI";       // Changed to correct description
    }else if (menu == 6){
         RETURN[0] = "Rotary Encoder";
         items=SET_ROTARY_ENCODER_MENU_ITEMS;
         ivalue= rottaryEncoderValues;
        TYPE[0] = "PPR";        // Changed to correct description
    }else if (menu == 7){
         RETURN[0] = "Lead Screw";
         ivalue= leadScrewValue;
         items=SET_LEAD_SCREW_MENU_ITEMS;
         TYPE[0] = "Pitch";     // Changed to correct description
    }else if (menu == 8){
         RETURN[0] = "Stepper Motor";
        ivalue= stepperMotorValue;
         items=SET_STEPPER_MOTOR_MENU_ITEMS;
         TYPE[0] = "SPR";
    }
    
    printf("MENU[0] = %s, MENU[1] = %s, MENU[2] = %s\n",OK_RETURN[0],OK_RETURN[1],OK_RETURN[2] );
    for (int i = 0; i < items; i++) {
        lcd_set_cursor(0, i); // Move cursor to row i
        if (i == pos) {
            lcd_print(">"); // Highlight the selected item
        } else {
            lcd_print(" "); // Add spaces for alignment
        }


        if ((menu == 6) || (menu == 7) || (menu == 8)){ // ONLY RETURN GLOBAL VARIABLES >>>LEAD >>>>STEPPER >>>>>ROTTARY
            if (i==1){
                sprintf(valueStr, "%d", ivalue); // Convert integer to string
                lcd_print(RETURN[i]);
                lcd_print(valueStr);
                lcd_print(" ");
                lcd_print(TYPE[0]);
                if (setleadScrew || setRottaryEncoder || setStepperMotor){
                     lcd_print("<");
                } 
            } else {
                lcd_print(RETURN[i]);
            } 
        } else{
            // FinalmetricThreadsValue = (int)(metricThreadsValue*100+0.5);
            float y = (int)(fvalue * 100+0.05) / 100.0; 
            if (i==1){
                if (menu==5){
                    sprintf(valueStr, "%d", ivalue); // >>>>TPI
                }else{
                    sprintf(valueStr, "%.2f", y); // RUN AND RETURN >>>>METRIC  >>>>POWERFEED
                }
                
                lcd_print(OK_RETURN[i]);
                lcd_print(valueStr);
                lcd_print(" ");
                lcd_print(TYPE[0]);
                if (setMetricThreads || setTPIthreads || setPowerFeed){
                     lcd_print("<");
                }
            } else {
                lcd_print(OK_RETURN[i]);
            } // Print the menu item
        }


    }
}

void CreateMenu(){
printf("path[0] = %d, path[1] = %d, path[2] = %d, path[3] = %d, path[4] = %d, path[5] = %d, path[6] = %d, path[7] = %d, path[8] = %d \n", path[0], path[1], path[2], path[3], path[4], path[5], path[6], path[7], path[8]);
         switch (path[0])
    {
////////////////////////////////////////////////////////////////////////////////
    case 0: //PRINT MAIN MENU
        PrintSettingsMenu(path[1],path[0]);
        break;
/////////////////////////////////////////////////////////////////////////
    case 1://PRINT POWER FEED MENU
        PrintValuesMenu(path[2], path[0]);
        break;
/////////////////////////////////////////////////////////////////////////
    case 2://PRINT THREADS MENU
        PrintSettingsMenu(path[3],path[0]);
        break;
/////////////////////////////////////////////////////////////////////////
    case 3://PRINT SETTINGS MENU
        PrintSettingsMenu(path[4],path[0]);
        break;
/////////////////////////////////////////////////////////////////////////
    case 4://PRINT METRIC THREADS
        PrintValuesMenu(path[5],path[0]);
        break;
/////////////////////////////////////////////////////////////////////////
    case 5://PRINT TPI THREADS
        PrintValuesMenu(path[6],path[0]);
        break;
/////////////////////////////////////////////////////////////////////////
    case 6://PRINT SET ROTTARY ENCODER PPR
        PrintValuesMenu(path[7],path[0]);
        break;
/////////////////////////////////////////////////////////////////////////
    case 7://PRINT SET LEAD SCREW PITCH
        PrintValuesMenu(path[8],path[0]);
        break;
/////////////////////////////////////////////////////////////////////////
    case 8://PRINT SET STEPPER MOTOR
        PrintValuesMenu(path[9],path[0]);
        break;
    case 9://PRINT SET STEPPER MOTOR
        PrintRun();
        break;
    default:
        break;
    }
}


void InitializeValues() {
    rottaryEncoderValues = GetNVS("rottary"); // INFO ROTTARY ENCODER STEPS * 4
    leadScrewValue = GetNVS("lead"); //INFO LEAD SCREW MILIMITERS
    stepperMotorValue = GetNVS("stepper"); //INFO STEPPER MOTOR STEPS
}

void SetNVS(const char* key, int32_t value){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);


    nvs_handle_t my_handle;
    err = nvs_open(STORAGE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("NVS handle opened successfully.\n");
    }


    err = nvs_set_i32(my_handle, key, value);// my_counter is the variable counter is the number
    if (err == ESP_OK) {
        // Commit written value.
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            printf("Error committing to NVS!\n");
        }
    } else {
        printf("Error setting value in NVS!\n");
    }

}


int GetNVS(const char* key){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);


    nvs_handle_t my_handle;
    err = nvs_open(STORAGE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("NVS handle opened successfully.\n");
    }


    int32_t stored_value = 0;
    err = nvs_get_i32(my_handle, key, &stored_value);
    switch (err) {
        case ESP_OK:
            printf("The stored counter value is: %ld\n", stored_value);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            SetNVS(key, 0);
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    nvs_close(my_handle);
    return stored_value;

}


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
            if (button_pressed) {
                
                Selection=1;
                 // Check if the button was previously pressed
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
                        path[0]=3;
                        path[1]=1;
                        PathMenu=4;
                        maxRottarySelection=4;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==1){//INFO POWER FEED
                    if (path[2]==1){
                        setPowerFeed= !setPowerFeed;//uhygvuvk
                        setSelection = !setSelection;
                        CreateMenu();
                    } else if (path[2]==2){
                        path[0]=9;
                        CreateMenu();
                        xSemaphoreGive(xStopTaskBSemaphore);
                        vTaskDelete(NULL);
                    } else if (path[2]==3){ // INFO RETURN TO MAIN MENU
                        path[0]=0;
                        path[2]=1;          
                        PathMenu=1;
                        maxRottarySelection = 3;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==2){//INFO THREADS MENU
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
                } else if(path[0] ==3){//INFO Settings Menu 
                    if (path[4]==1){//BUG 
                        path[0]=6;
                        path[4]=1;
                        PathMenu=7;
                        maxRottarySelection = 2;
                        CreateMenu();
                    } else if (path[4]==2){
                        path[0]=7;
                        path[4]=1;
                        PathMenu=8;
                        maxRottarySelection = 2;
                        CreateMenu();
                    } else if (path[4]==3){//INFO RETURN TO MAIN MENU
                        path[0]=8;
                        path[4]=1; 
                        PathMenu=9;
                        maxRottarySelection = 2;
                        CreateMenu();
                    } else if (path[4]==4){//INFO RETURN TO MAIN MENU
                        path[0]=0;
                        path[4]=1; 
                        PathMenu=1;
                        maxRottarySelection = 3;
                        CreateMenu();
                    }

////////////////////////////////////////////////////////////////////
                } else if(path[0] ==4){//INFO METRIC THREADS
                    if (path[5]==1){
                        setSelection = !setSelection;
                        setMetricThreads = !setMetricThreads;
                        CreateMenu();
                    } else if (path[5]==2){
                        path[0]=9;
                        CreateMenu();
                        xSemaphoreGive(xStopTaskBSemaphore);
                        vTaskDelete(NULL);
                    } else if (path[5]==3){//INFO RETURN TO MAIN MENU
                        path[0]=2;
                        path[5]=1; 
                        PathMenu=3;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==5){//INFO TPI MENY
                    if (path[6]==1){
                        setSelection = !setSelection;
                        setTPIthreads = !setTPIthreads;
                        CreateMenu();
                    } else if (path[6]==2){
                        path[0]=9;
                        CreateMenu();
                        xSemaphoreGive(xStopTaskBSemaphore);
                        vTaskDelete(NULL);
                    } else if (path[6]==3){//INFO RETURN TO MAIN MENU
                        path[0]=2;
                        path[6]=1; 
                        PathMenu=3;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==6){//INFO SET ROTTARY
                    if (path[7]==1){
                        setSelection = !setSelection;
                        
                        if (setRottaryEncoder==false){
                            setRottaryEncoder = true;
                            CreateMenu();
                        } else{
                            setRottaryEncoder = !setRottaryEncoder;
                            SetNVS("rottary", rottaryEncoderValues);
                            CreateMenu();
                        }
                    } else if (path[7]==2){//INFO RETURN TO SETTINGS MENU
                        path[0]=3;
                        path[7]=1; 
                        PathMenu=4;
                        maxRottarySelection = 4;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==7){//INFO SET LEAD SCREW
                    if (path[8]==1){
                        setSelection = !setSelection;
                        
                        if (setleadScrew==false){
                            setleadScrew = true;
                            CreateMenu();
                        } else{
                            setleadScrew = !setleadScrew;
                            SetNVS("lead", leadScrewValue);
                            CreateMenu();
                        }
                    } else if (path[8]==2){//INFO RETURN TO SETTINGS MENU
                        path[0]=3;
                        path[8]=1; 
                        PathMenu=4;
                        maxRottarySelection = 4;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                } else if(path[0] ==8){//INFO SET STEPPER MOTOR
                    if (path[9]==1){
                        setSelection = !setSelection;
                        
                        if (setStepperMotor==false){
                            setStepperMotor = true;
                            CreateMenu();
                        } else{
                            setStepperMotor = !setStepperMotor;
                            SetNVS("stepper", stepperMotorValue);
                            CreateMenu();
                        }
                    } else if (path[9]==2){//INFO RETURN TO SETTINGS MENU
                        path[0]=3;
                        path[3]=1; 
                        PathMenu=4;
                        maxRottarySelection = 4;
                        CreateMenu();
                    }
////////////////////////////////////////////////////////////////////
                    }                   
                    
                }
                button_pressed = false; // Reset the flag
            }
        
        // Add a delay to avoid bouncing and flooding the console
        vTaskDelay(pdMS_TO_TICKS(200));        //initial value 200
  } 
}




void SelectorCounter(void *pvParameters)
{
    ESP_LOGI(TAGLCD, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));


    ESP_LOGI(TAGLCD, "install pcnt channels");
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

    ESP_LOGI(TAGLCD, "set edge and level actions for pcnt channels");



    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAGLCD, "add watch points and register callbacks");

    // QueueHandle_t queue = xQueueCreate(10, sizeof(int));

    ESP_LOGI(TAGLCD, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAGLCD, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAGLCD, "start pcnt unit");
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
            printf("Pulse Count PRO 4: %d PREV Count PRO : %d 4\n", pulse_count, prev_count);
            if (pulse_count %4 ==0) {
            printf("Pulse Count: %d\n", pulse_count);
            
            // Determine the direction of rotation
            if (prev_count < pulse_count) {
                // Clockwise rotation
                if (setSelection == true){
                    Selection++;
                        if (Selection > maxRottarySelection) {
                            Selection = 1;
                    }
                } else if (setPowerFeed == true){
                        if (powerFeedValue<0.16){
                            powerFeedValue+=0.01;
                            if (powerFeedValue > 0.16) { // Ensure it doesn't go below zero
                                powerFeedValue = 0.16;
                            }
                            FinalmetricThreadsValue = (int)(powerFeedValue*100+0.1);
                            powerFeedRun = true;
                            metricThreadsRun= false;
                            tpiThreadsRun=false;
                        }
                }else if (setMetricThreads == true){
                        if (metricThreadsValue<6.00){
                            metricThreadsValue+=0.05;
                            if (metricThreadsValue>6.00){
                                metricThreadsValue=6.00;
                            }
                            FinalmetricThreadsValue = (int)(metricThreadsValue*100+0.5);
                            powerFeedRun = false;
                            metricThreadsRun= true;
                            tpiThreadsRun=false;
                        }
                }else if (setTPIthreads == true){
                        if (tpiThreadsValue<60){
                            tpiThreadsValue++;
                            if(tpiThreadsValue>60){
                                tpiThreadsValue=60;
                            }
                            FinalmetricThreadsValue = (int)((25.4/tpiThreadsValue)*100+0.5);
                            powerFeedRun = false;
                            metricThreadsRun= false;
                            tpiThreadsRun=true;

                        }
                }else if (setRottaryEncoder == true){
                        if (rottaryEncoderValues<10000){
                            rottaryEncoderValues+=100;
                            if (rottaryEncoderValues>10000){
                                rottaryEncoderValues=10000;
                            }
                        }

                }else if (setleadScrew == true){
                        if (leadScrewValue<300){
                            leadScrewValue+=10;
                            if (leadScrewValue>300){
                                leadScrewValue =300;
                            }
                        } 
                }else if (setStepperMotor == true){
                        if (stepperMotorValue<800){
                            stepperMotorValue+=100;
                            if (stepperMotorValue>800){   // Changed to correct description
                                stepperMotorValue = 800;
                            }
                        }
                }



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            } else {
                if (setSelection== true){
                // Counter-clockwise rotation
                    Selection--;
                    if (Selection < 1) {
                        Selection = maxRottarySelection;
                    }
                } else if (setPowerFeed == true){
                        if (powerFeedValue>0.00){
                            powerFeedValue -= 0.01;
                            if (powerFeedValue<0.00){
                                powerFeedValue = 0.00;
                            }
                        }
                        FinalmetricThreadsValue = (int)(powerFeedValue*100+0.1);
                        powerFeedRun = true;
                            metricThreadsRun= false;
                            tpiThreadsRun=false;
                } else if (setMetricThreads == true){
                        if (metricThreadsValue>0){
                            metricThreadsValue-=0.05;
                            if (metricThreadsValue < 0.00){
                                metricThreadsValue=0.00;
                            }
                        }
                        FinalmetricThreadsValue = (int)(metricThreadsValue*100+0.5);
                        powerFeedRun = false;
                            metricThreadsRun= true;
                            tpiThreadsRun=false;
                } else if (setTPIthreads == true){
                        if (tpiThreadsValue>0){
                            tpiThreadsValue--;
                            if (tpiThreadsValue < 0){
                                tpiThreadsValue = 0;
                            }
                        }
                        FinalmetricThreadsValue = (int)((25.4/tpiThreadsValue)*100+0.5);
                        powerFeedRun = false;
                            metricThreadsRun= false;
                            tpiThreadsRun=true;
                } else if (setRottaryEncoder == true){
                        if (rottaryEncoderValues>0){
                            rottaryEncoderValues-=100;
                            if (rottaryEncoderValues<600){
                                rottaryEncoderValues=600;
                            }
                        }
                }else if (setleadScrew == true){
                        if (leadScrewValue>0){
                            leadScrewValue-=10;
                            if (leadScrewValue<0){
                                leadScrewValue-=0;
                            }
                        }
                }else if (setStepperMotor == true){
                        if (stepperMotorValue>0){
                            stepperMotorValue-=100;
                            if (stepperMotorValue<0){
                                stepperMotorValue = 0;
                            }
                        }
                }
            }
            printf("PathMenu %d Selection: %d Pulse Count: %d PrevCount: %d Metric: %d, Metric Dec %.2f, Power %.2f TPI: %d Final TPIMETRIC: %d\n", PathMenu, Selection, pulse_count, prev_count, FinalmetricThreadsValue, metricThreadsValue, powerFeedValue, tpiThreadsValue,FinalmetricThreadsValue);
            path[PathMenu] = Selection;

            CreateMenu();
            prev_count = pulse_count;


            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));         //initial value 30
        

        if (xSemaphoreTake(xStopTaskBSemaphore, 0) == pdTRUE) {
            ESP_LOGI("TaskB", "TaskB received stop signal");
            vTaskDelete(NULL); // Delete TaskB
        }

    }
}

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
    int RottaryStepps = GetNVS("rottary");
    esp_task_wdt_add(NULL);
    block_data_t *p = (block_data_t *)pvParameters;
    int *numbers = p->arr;       // pointer to allocated array
    int sizeOfList = p->length;  // length


        ESP_LOGI("RottaryEnc", "sizeOfList = %d", sizeOfList);
    for (int k = 0; k < sizeOfList; k++) {
        ESP_LOGI("RottaryEnc", "Element %d = %d", k, numbers[k]);
    }

    ESP_LOGI(TAGLATHE, "install pcnt unit");
    pcnt_unit_handle_t pcnt_unit = NULL;// INFO  I TOOK IT FROM BELO
    pcnt_unit_config_t unit_config = {
        .high_limit = RottaryStepps,
        .low_limit = RottaryStepps*(-1),
    };
   
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAGLATHE, "install pcnt channels");
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

    ESP_LOGI(TAGLATHE, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));


    ESP_LOGI(TAGLATHE, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAGLATHE, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAGLATHE, "start pcnt unit");
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
    xStopTaskBSemaphore = xSemaphoreCreateBinary();
    if (xStopTaskBSemaphore == NULL) {
        ESP_LOGE("Main", "Failed to create semaphore");
        return;
    }

    // Create the queue to start TaskC and TaskE
    xStartTasksQueue = xQueueCreate(1, sizeof(int));
    if (xStartTasksQueue == NULL) {
        ESP_LOGE("Main", "Failed to create queue");
        return;
    }

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

    InitializeValues();
    // InitializeValues();

    


    i2c_master_init();
    lcd_init();
    // set a welcome screen
    CreateMenu();


    xTaskCreate(
        SelectorCounter,            // Function that implements the task
        "SelectorCounter",      // Name of the task
        4096,             // Stack size
        NULL,             // Task input parameter
        1,                // Priority of the task
        // NULL,             // Task handle
        &TaskA_Handle
        // 1                 // Core where the task should run (1 for core 1, 0 for core 0)

    );

    xTaskCreate(
        ButtonPressed,            // Function that implements the task
        "ButtonPressed",      // Name of the task
        4096,             // Stack size
        NULL,             // Task input parameter
        1,                // Priority of the task
        // NULL,             // Task handle
        &TaskB_Handle
        // 1                 // Core where the task should run (1 for core 1, 0 for core 0)

    );
    // Loop forever

    while (eTaskGetState(TaskA_Handle) != eDeleted || eTaskGetState(TaskB_Handle) != eDeleted) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf(  "Rottary : %d, Lead : %d, Stepper: %d, Metric : %d ",  rottaryEncoderValues ,   leadScrewValue ,   stepperMotorValue, FinalmetricThreadsValue );



   
    int LeadScrewPitch = GetNVS("lead");
    int StepperSteps = GetNVS("stepper");
    int SpindlePulses = GetNVS("rottary");

    // We want to express ratio = pulses / steps in simplest form.
    // ratio = 4000 / 175
    // 1) Compute GCD
    esp_task_wdt_config_t wd_config = {
        .timeout_ms =5000,
        .idle_core_mask=0,
        .trigger_panic=true,
       };
         esp_err_t ret = esp_task_wdt_reconfigure(&wd_config); // Apply configuration
        if (ret != ESP_OK) {
            printf("Task Watchdog Timer reconfiguration failed: %d\n", ret);
        }


    float fRleadScrew = StepperSteps * ((float)FinalmetricThreadsValue / (float)LeadScrewPitch);
    int RleadScrew = (int)fRleadScrew;
   
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
    vTaskDelay(1000000);

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