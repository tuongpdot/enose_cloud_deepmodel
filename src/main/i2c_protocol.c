#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_adc/adc_continuous.h"
#include "esp_log.h"

#include "driver/i2c.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num 
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */
#define DEBUG 0

#define I2C_MASTER_TIMEOUT_MS       1000

/** declare lval with rval from menuconfig **/
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

/** declare sensor's registers   **/
#define CCS811B_SENSOR_ADDR      0x5A            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_STATUS           0x00            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_MEAS_MODE        0x01            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_ALG_RESULT_DATA  0x02            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_RAW_DATA         0x03            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_ENV_DATA         0x05            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_NTC              0x06            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_THRESHOLDS       0x10            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_BASELINE         0x11            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_HW_ID            0x20            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_HW_VERSION       0x21            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_FW_BOOT_VERSION  0x23            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_FW_APP_VERSION   0x24            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_ERROR_ID         0xE0            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/
#define CCS811B_SW_RESET         0xFF            /*!< slave address for CCS811B sensor when ADDR pin is High (0x5A for LOW)*/

#define CCS811B_HW_ID_CODE      0x81
#define CCS811B_REF_RESISTOR    100000

#define CCS811B_BOOTLOADER_APP_ERASE  0xF1
#define CCS811B_BOOTLOADER_APP_DATA   0xF2
#define CCS811B_BOOTLOADER_APP_VERIFY 0xF3
#define CCS811B_BOOTLOADER_APP_START  0xF4

#define CCS811B_DRIVE_MODE_IDLE   0x00
#define CCS811B_DRIVE_MODE_1SEC   0x01
#define CCS811B_DRIVE_MODE_10SEC  0x02
#define CCS811B_DRIVE_MODE_60SEC  0x03
#define CCS811B_DRIVE_MODE_250MS  0x04

/** i2c flag **/
#define WRITE_BIT       I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT        I2C_MASTER_READ               /*!< I2C master read */
#define ACK_CHECK_EN    0x1                           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS   0x0                           /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0                           /*!< I2C ack value */
#define NACK_VAL        0x1                           /*!< I2C nack value */

/**************************************************************************/
/*!
 * @brief  Setups the I2C interface and hardware and checks for communication.
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
*/
/**************************************************************************/
esp_err_t master_write_to(i2c_port_t i2c_num,uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len)
{
  int ret = ESP_FAIL; /** Flag **/
  //i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  while(ret != ESP_OK){
    #if DEBUG == 1
      printf("\n  master_write_to: 0/1 ...");
    #endif /* DEBUG */

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(i2c_master_start(cmd) != ESP_OK) break; 
    if(i2c_master_write_byte(cmd, dev_addr << 1 | WRITE_BIT, ACK_CHECK_EN) != ESP_OK) break;
    if(i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN) != ESP_OK) break;
    
    /* Some sensors,including the CCS811b - C02,/
    require "fake write" into a bit on register to wake up some features */
    if(data != NULL && data_len != 0){
      if(i2c_master_write(cmd, data, data_len, ACK_CHECK_EN) != ESP_OK) break;
    }

    if(i2c_master_stop(cmd) != ESP_OK) break;
    if(i2c_master_cmd_begin(i2c_num, cmd, (1000/portTICK_PERIOD_MS)) != ESP_OK) break;
    i2c_cmd_link_delete(cmd);

    ret = ESP_OK;

    #if DEBUG == 1
      printf("1/1 ... Done. \n");
    #endif /* DEBUG */
  }

  return (ret != ESP_OK) ? (ESP_FAIL) : (ESP_OK);
}

/**
 * @brief i2c read 
 *
 */
esp_err_t master_read_from(i2c_port_t i2c_num,uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_h, uint8_t *data_l)
{
  int ret = ESP_FAIL; /** Flag **/

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  while(ret != ESP_OK){
    #if DEBUG == 1
      printf("  master_read_from: 0/2 ...");
    #endif /* DEBUG */

    if(i2c_master_start(cmd) != ESP_OK) break; 
    if(i2c_master_write_byte(cmd, dev_addr << 1 | WRITE_BIT, ACK_CHECK_EN) != ESP_OK) break; 
    if(i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN) != ESP_OK) break; 
    if(i2c_master_stop(cmd) != ESP_OK) break;
    if(i2c_master_cmd_begin(i2c_num, cmd, (1000/portTICK_PERIOD_MS)) != ESP_OK) break; 

    i2c_cmd_link_delete(cmd);
    ret = ESP_OK;
    break;
  }

  if(ret != ESP_OK){
    return ret; //@expect: ESP_FAIL
  }else {
    #if DEBUG == 1
      printf("1/2 ...");
    #endif /* DEBUG */
    
    ret = ESP_FAIL;
  }

  /* Delay between transistion */
  vTaskDelay(300 / portTICK_PERIOD_MS);

  while(ret != ESP_OK){
    cmd = i2c_cmd_link_create();
    if(i2c_master_start(cmd) != ESP_OK)break;
    if(i2c_master_write_byte(cmd, dev_addr << 1 | READ_BIT, ACK_CHECK_EN) != ESP_OK) break;
    if(i2c_master_read_byte(cmd, data_h, ACK_VAL) != ESP_OK) break;
    if(i2c_master_read_byte(cmd, data_l, NACK_VAL) != ESP_OK) break;
    if(i2c_master_stop(cmd) != ESP_OK) break;
    if(i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS) != ESP_OK) break;

    i2c_cmd_link_delete(cmd);

    #if DEBUG == 1
      printf("2/2 ... Done. \n");
    #endif /* DEBUG */

    ret = ESP_OK;
    break;
  }

  return (ret != ESP_OK) ? (ESP_FAIL) : (ESP_OK);
}

/**
 * @brief i2c read 
 *
 */
static esp_err_t master_setup(void)
{
  int masterPort = I2C_MASTER_NUM;
  int ret[2];
  int count = 0; 

  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };

  ret[0] = i2c_param_config(masterPort, &conf);
  ret[1] = i2c_driver_install(masterPort, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

  /* Check flags */
  for(size_t i = 0; i != *(&ret + 1) - ret; i++){
    if(*(ret+i) == ESP_OK) count++; 
  }

  printf("\n   ...\n   ...\n   ...\n");

  return (count != 2) ? (ESP_FAIL) : (ESP_OK); 
}

/**
 * @brief i2c read shortcut
 *
 */
uint8_t read_it(uint8_t reg_addr, uint8_t *data, size_t *data_len){
  return (master_read_from(I2C_MASTER_NUM, CCS811B_SENSOR_ADDR, reg_addr, data, data_len) != ESP_OK) ? (ESP_FAIL) : (*data);
}

/**
 * @brief i2c write shortcut 
 *
 */
esp_err_t write_it(uint8_t reg, uint8_t *cmd, uint8_t data_len){
  return master_write_to(I2C_MASTER_NUM, CCS811B_SENSOR_ADDR, reg, cmd, data_len) ;
}

/**
 * @brief i2c slave setup  
 *
 */
static esp_err_t slave_setup_ccs881b(void)
{
  uint8_t *cmd_data = NULL;

  uint8_t cmd_reset[] = {0x11, 0xE5, 0x72, 0x8A}; 
  uint8_t cmd_mode[] = {1 << 4};

  uint8_t ref;   /* tracking flag */ 
  uint8_t data_h = 0;
  uint8_t data_hl = 0;

  /*write the command to force sensor to be reset*/
  write_it(CCS811B_SW_RESET, cmd_reset, 4);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(read_it(CCS811B_STATUS, &data_h, &data_hl) != 0x10){
      printf("\n    Wasnt reset ... \n");
    }else{
      printf("\n    Sensor Reset: DONE. \n");
    }

    /*read the register to confirm sensor's ID being matched */
    if(read_it(CCS811B_HW_ID, &data_h, &data_hl) != 0x81){
      printf("    Didn't detect sensor ... \n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }else{
      printf("    Sensor Identification: DONE. \n");
    }

  /*   */
  write_it(CCS811B_BOOTLOADER_APP_START, NULL, 0);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(read_it(CCS811B_STATUS, &data_h, &data_hl) != 0x90){
      printf("    Wasnt switched ...\n");
    }else{
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      printf("    Sensor's Mode Switching: APP mode.\n");
    }

  /*   */
  write_it(CCS811B_MEAS_MODE, cmd_mode , 1); 
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  printf(" ********** REGISTER TRACKING AFTER INSTALLATION **********\n");
  printf("    Meas's register:   %d \n", read_it(CCS811B_MEAS_MODE , &data_h, &data_hl) );
  printf("    Status's register: %d \n", read_it(CCS811B_STATUS    , &data_h, &data_hl) );
  printf("    ERROR's register:  %d \n", read_it(CCS811B_ERROR_ID  , &data_h, &data_hl) );
  
  return ESP_OK;
}

static void doing_task(void *arg)
{
  int ret;
  uint8_t counter = 0;

  uint8_t _TimeStamp = 0;
  uint16_t _eCO2;
  uint8_t _TVOC = 0;
  uint8_t _Smoke = 0;
  uint8_t _Alcohol =0;
  uint8_t _Temp = 0;
  uint8_t _Humid = 0;

  //init the master
  printf("\nSetting Up: Master ....................");
  int ret_ret;
  ret_ret = master_setup(); 

  if(ret_ret == ESP_OK){
    ESP_LOGI(TAG,"DONE with MASTER.");
  }else{
    ESP_LOGE(TAG,"ERROR with MASTER.");
  }

  printf("\nSetting Up: Slaves ....................");
  ret_ret = slave_setup_ccs881b(); 
  if(ret_ret == ESP_OK){
    ESP_LOGI(TAG,"DONE with SLAVE.");
  }else{
    ESP_LOGE(TAG,"ERROR with SLAVE.");
  }
  printf("\n");

  printf(" ********** C02 & TVOC Reading: **********\n");
  //---------------------------------------------------
  while (counter < 20) {
    uint8_t buffer_h[8] ={0}, buffer_l[8]={0};
    // If for checking the Data Available Register!
    ret = master_read_from(I2C_MASTER_NUM, CCS811B_SENSOR_ADDR, CCS811B_ALG_RESULT_DATA, buffer_h, buffer_l);
    if(ret == ESP_OK){

      _eCO2 = (uint16_t)(buffer_h[0] << 8) | (uint16_t)(buffer_h[1]);
      vTaskDelay(1500 / portTICK_PERIOD_MS);

      printf("%d ", _TimeStamp);
      printf(",");

      //ADC - Channel 
      printf("%d ", _Temp);
      printf(",");
      printf("%d ", _Humid);
      printf(",");

      //I2C
      printf("%d ", _eCO2);
      printf(",");
      printf("%d ", _TVOC);
      printf(",");

      //ADC - Channel 
      printf("%d ", _Alcohol);
      printf(",");
      //ADC - Channel 
      printf("%d ", _Smoke);
      printf(",");
    } //end of if 
    counter++;
    printf("\n");
  }//End of while

  vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(doing_task, "task_doing", 1024 * 2, (void *)0, 10, NULL);
}
