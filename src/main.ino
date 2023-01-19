/**
 * @file main.cpp
 * @author Ab-Tx
 *         MiguelRodriguesESTG
 * @brief This software allows logging various sensors into a text file on an SDCard.
 * @version 0.2
 * @date 2023-01-18
 *
 * @copyright Copyright (c) 2023
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#include <Arduino.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_freertos_hooks.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "esp_adc_cal.h"

#include <WiFi.h>
#include "esp_wifi.h"
#include "time.h"
#include <ESP32Time.h> // RTC

#include "IIM42352_I2C.h"
#include "ATM90E36.h"
#include "Adafruit_Si7021.h"

#include <SPI.h>

#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>
 /* ################################################# */

#ifndef PINS_
#define PIN_ADC1_0 36
#define PIN_ATM_PM 39
#define PIN_ATM_CF4 34
#define PIN_ATM_CF3 35
#define PIN_ATM_CF2 32
#define PIN_ATM_CF1 33
#define PIN_ATM_IRQ1 25
#define PIN_ATM_IRQ0 26
#define PIN_ATM_DMA 27
#define PIN_HSPI_CLK 14 //SD
#define PIN_HSPI_MISO 12   //SD
#define PIN_HSPI_ID 13  //SD
#define PIN_ATM_WARNOUT 9
//#define GPIO10 10 // Connected to internal SPI
//#define GPIO11 11 // Connected to internal SPI
#define PIN_VSPI_MOSI 23
#define PIN_I2C_SCL 22
#define PIN_I2C_SDA 21
#define PIN_VSPI_MISO 19
#define PIN_VSPI_SCLK 18
#define PIN_VSPI_CS0 5
#define PIN_INT2 17
#define PIN_INT1 16
#define PIN_GPIO4 4 // Available
#define PIN_GPIO0 0 // Available
#define PIN_LED 2 // Available
#define PIN_HSPI_CS 15  //SD
//#define GPIO8 8 // Connected to internal SPI
//#define GPIO7 7 // Connected to internal SPI
//#define GPIO6 6 // Connected to internal SPI
#endif

// Do not enable both, different serial baudrate is used for each
#define EXTERNAL_BLUETOOTH_MODULE
//#define DEBUG_SERIAL_
#ifdef EXTERNAL_BLUETOOTH_MODULE
#ifdef DEBUG_SERIAL_
#error "Cannot define both, HC-11 is to be disconnected if you're debugging with serial."
#endif
#endif

Adafruit_Si7021 sensor = Adafruit_Si7021();
IIM42352 IIM42SENSOR(IIM42352_DEVICE_ADDRESS_68);
ATM90E36 eic;

/* ################################################# */

// ESP32Time rtc;
ESP32Time rtc(0); // offset in seconds GMT+0

/* Declare a variable of type SemaphoreHandle_t.  This is used to reference the
mutex type semaphore that is used to ensure mutual exclusive access to stdout. */
SemaphoreHandle_t xMutexSPI;
SemaphoreHandle_t xMutexHSPI;
SemaphoreHandle_t xMutexI2C;

/* The task functions. */
void vMainTask(void* pvParameters); // 250ms
void vWiFiTask(void* pvParameters); // async, runs once
void vADCTask(void* pvParameters); //250ms
void vAtmTask(void* pvParameters); //250ms
void vSiTask(void* pvParameters); //250ms
void vIImTask(void* pvParameters); //250ms
void vSDwriteTask(void* pvParameters); //250ms
static void vSerialSendTask(void* pvParameters); // async, for print only  
static void vSDHandlerTask(void* pvParameters); // async
bool my_vApplicationIdleHook(void); // idleTask

/* The service routine for the interrupt.  This is the interrupt that the task
will be synchronized with. */
static void IRAM_ATTR vSDInterruptHandler(void);
#define INTERRUPT_PIN 4 // GPIO 2 SDCard

/*  Structs */
typedef struct
{
  // edit
  float humidity;
  float temperature;
} data_si_t;

typedef struct
{
  // ATM90E36
  double currentA;
  double currentB;
  double currentC;
  double currentN;
  double currentCalcN;
  double voltageA;
  double voltageB;
  double voltageC;
  double activePowerA;
  double activePowerB;
  double activePowerC;
  double activePowerTotal;
  double reactivePowerA;
  double reactivePowerB;
  double reactivePowerC;
  double reactivePowerTotal;
  double apparentPowerA;
  double apparentPowerB;
  double apparentPowerC;
  double apparentPowerTotal;
  double powerFactorA;
  double powerFactorB;
  double powerFactorC;
  double powerFactorTotal;
  double voltageHarmonicsA;
  double voltageHarmonicsB;
  double voltageHarmonicsC;
  double currentHarmonicsA;
  double currentHarmonicsB;
  double currentHarmonicsC;
  double phaseA;
  double phaseB;
  double phaseC;
  double frequency;
  double temperature;
  double importEnergy;
  double exportEnergy;
  unsigned short sysStatus0;  // for checking erros
  unsigned short sysStatus1;
  unsigned short meterStatus0;
  unsigned short meterStatus1;
} data_atm_t;

typedef struct
{
  // edit
  float accelX;
  float accelY;
  float accelZ;
} data_iim_t;


volatile bool sdcardInserted = false;

/* Declare a variable of type QueueHandle_t.  This is used to store the queue
that is accessed by all three tasks. */
QueueHandle_t xQueueADC;  // LM35
QueueHandle_t xQueueATM;  // ATM90E36
QueueHandle_t xQueueSI;   // SI7021
QueueHandle_t xQueueIIM;  // IIM-42352
QueueHandle_t xQueueBlue; // Bluetooth, HC-11, EUSART
QueueHandle_t xQueueSD;   // Cartao SD
// Informar a Idle task a partir da
// handler task da presença do cartão

 /*
  * SD Card | ESP32 | HSPI
  *    D2       -
  *    D3       SS     GPIO 15
  *    CMD      MOSI   GPIO 13
  *    VSS      GND
  *    VDD      3.3V
  *    CLK      SCK    GPIO 14
  *    VSS      GND
  *    D0       MISO   GPIO 12
  *    D1       -
  */

  /* Declare a variable of type SemaphoreHandle_t.  This is used to reference the
  semaphore that is used to synchronize a task with an interrupt. */
SemaphoreHandle_t xBinarySemaphore;
#define LED_CHANNEL 1

// Task priorities:
#define PRIO_TASK_MAIN  6    
#define PRIO_TASK_HANDLER 4 
#define PRIO_TASK_WIFI  2    
#define PRIO_TASK_ADC   3
#define PRIO_TASK_ATM   3
#define PRIO_TASK_SI    3
#define PRIO_TASK_IIM   3
#define PRIO_TASK_SDWRITE   3
#define PRIO_TASK_SERIALPRINT 2

void setup()
{
  pinMode(PIN_ATM_PM, INPUT); // :/
  pinMode(PIN_ATM_CF1, INPUT);
  pinMode(PIN_ATM_CF2, INPUT);
  pinMode(PIN_ATM_CF3, INPUT);
  pinMode(PIN_ATM_CF4, INPUT);
  pinMode(PIN_ATM_IRQ0, INPUT);
  pinMode(PIN_ATM_IRQ1, INPUT);
  pinMode(PIN_ATM_DMA, OUTPUT); // set to 0 so ATM90E36 operates as slave
  digitalWrite(PIN_ATM_DMA, LOW); // set to 0 so ATM90E36 operates as slave
  pinMode(PIN_ATM_WARNOUT, INPUT);
  pinMode(PIN_VSPI_CS0, OUTPUT);
  digitalWrite(PIN_VSPI_CS0, HIGH); // SPI CS
  pinMode(21, INPUT_PULLUP); // I2C SDA pin
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH); // set ATM to normal mode


  /* Before a semaphore is used it must be explicitly created.  In this example
    a binary semaphore is created. */
  vSemaphoreCreateBinary(xBinarySemaphore);
  /* Before a semaphore is used it must be explicitly created.  In this example
  a mutex type semaphore is created. */
  xMutexSPI = xSemaphoreCreateMutex();
  xMutexHSPI = xSemaphoreCreateMutex();
  xMutexI2C = xSemaphoreCreateMutex();

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), &vSDInterruptHandler, CHANGE);
  if (digitalRead(INTERRUPT_PIN) == LOW) {
    sdcardInserted = true;
  }

  // Set loopTask max priority before deletion
  vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

#ifndef EXTERNAL_BLUETOOTH_MODULE
  // Init USART and set Baud-rate to 115200
  Serial.begin(115200);
#else
  // Init USART and set Baud-rate to 9600 -> only supported speed by Bluetooth module
  Serial.begin(9600);
#endif
  Wire.begin();
  SPI.begin();

  xTaskCreatePinnedToCore(vWiFiTask,                                 /* Pointer to the function that implements the task. */
    "WiFi Task",                               /* Text name for the task.  This is to facilitate debugging only. */
    3024,                                      /* Stack depth - most small microcontrollers will use much less stack than this. */
    NULL,                                      /* We are not using the task parameter. */
    PRIO_TASK_WIFI, /* Priority */ /* Range from 0 to 32, 32 being the highest priority */
    NULL,                                      /* We are not using the task handle. */
    0);                                        /* Core where the task should run */

  /* *************************************************** */
  /* The queue is created to hold a maximum of 5 values. */
  xQueueADC = xQueueCreate(1, sizeof(float));
  if (xQueueADC != NULL)
  {
    xTaskCreatePinnedToCore(vADCTask,                                 /* Pointer to the function that implements the task. */
      "ADC Task",                               /* Text name for the task.  This is to facilitate debugging only. */
      1024,                                      /* Stack depth - most small microcontrollers will use much less stack than this. */
      NULL,                                      /* We are not using the task parameter. */
      PRIO_TASK_ADC, /* Priority. */ /* Range from 0 to 32, 32 being the highest priority */
      NULL,                                      /* We are not using the task handle. */
      1);
  }
  else
  {
  #ifdef DEBUG_SERIAL_
    Serial.println("[ERROR] The queue xQueueADC could not be created.");
  #endif
  }

  xQueueATM = xQueueCreate(3, sizeof(data_atm_t));
  if (xQueueATM != NULL)
  {
    // ATM Task here
    eic.begin(PIN_VSPI_CS0, 50, 1, 1, 1, 1, 1, 1);
    xTaskCreatePinnedToCore(vAtmTask,                                 /* Pointer to the function that implements the task. */
      "Atm Task",                               /* Text name for the task.  This is to facilitate debugging only. */
      8096,                                      /* Stack depth - most small microcontrollers will use much less stack than this. */
      NULL,                                      /* We are not using the task parameter. */
      PRIO_TASK_ATM, /* Priority */ /* Range from 0 to 32, 32 being the highest priority */
      NULL,                                      /* We are not using the task handle. */
      1);                                        /* Core where the task should run */
  }
  else
  {
  #ifdef DEBUG_SERIAL_
    Serial.println("[ERROR] The queue xQueueATM could not be created.");
  #endif
  }

  xQueueSI = xQueueCreate(3, sizeof(data_si_t));
  if (xQueueSI != NULL)
  {
    // Si Task here
    xTaskCreatePinnedToCore(vSiTask, // Pointer to the function that implements the task. 
      "Si Task",  // Text name for the task.  This is to facilitate debugging only. 
      8000,// Stack depth - most small microcontrollers will use much less stack than this. 
      NULL, // We are not using the task parameter. 
      PRIO_TASK_SI,    // Priority 
      NULL, // We are not using the task handle. 
      1);   // Core where the task should run 
  }
  else
  {
  #ifdef DEBUG_SERIAL_
    Serial.println("[ERROR] The queue xQueueSI could not be created.");
  #endif
  }

  xQueueIIM = xQueueCreate(3, sizeof(data_iim_t));
  if (xQueueIIM != NULL)
  {
    // IIM Task here
    xTaskCreatePinnedToCore(vIImTask, // Pointer to the function that implements the task. 
      "IIM Task",  // Text name for the task.  This is to facilitate debugging only. 
      8000,// Stack depth - most small microcontrollers will use much less stack than this. 
      NULL, // We are not using the task parameter. 
      PRIO_TASK_IIM,    // Priority 
      NULL, // We are not using the task handle. 
      1);   // Core where the task should run 
  }
  else
  {
  #ifdef DEBUG_SERIAL_
    Serial.println("[ERROR] The queue xQueueIIM could not be created.");
  #endif
  }
  xQueueBlue = xQueueCreate(3, sizeof(char[300]));
  if (xQueueBlue != NULL)
  {
    // ...
    // received info handled in main task
    xTaskCreatePinnedToCore(vSerialSendTask, // Pointer to the function that implements the task. 
      "Serial print Task",  // Text name for the task.  This is to facilitate debugging only. 
      4000,// Stack depth - most small microcontrollers will use much less stack than this. 
      NULL, // We are not using the task parameter. 
      PRIO_TASK_SERIALPRINT,    // Priority 
      NULL, // We are not using the task handle. 
      1);   // Core where the task should run 
  }
  else
  {
  #ifdef DEBUG_SERIAL_
    Serial.println("[ERROR] The queue xQueueBlue could not be created.");
  #endif
  }
  xQueueSD = xQueueCreate(3, sizeof(std::string*));
  if (xQueueSD != NULL)
  {
    // SD Task here
    xTaskCreatePinnedToCore(vSDwriteTask, // Pointer to the function that implements the task. 
      "SDWrite Task",  // Text name for the task.  This is to facilitate debugging only. 
      4000,// Stack depth - most small microcontrollers will use much less stack than this. 
      NULL, // We are not using the task parameter. 
      PRIO_TASK_SDWRITE,    // Priority 
      NULL, // We are not using the task handle. 
      1);   // Core where the task should run 
  }
  else
  {
  #ifdef DEBUG_SERIAL_
    Serial.println("[ERROR] The queue xQueueSD could not be created.");
  #endif
  }

  pinMode(PIN_LED, OUTPUT);
  ledcSetup(LED_CHANNEL, 5000, 8);
  ledcAttachPin(PIN_LED /* PIN_LED */, LED_CHANNEL);
  // Idle Task hook
  esp_register_freertos_idle_hook(my_vApplicationIdleHook);

  xTaskCreatePinnedToCore(vMainTask, /* Pointer to the function that implements the task. */
    "Main Task",         /* Text name for the task.  This is to facilitate debugging only. */
    10240,             /* Stack depth - most small microcontrollers will use much less stack than this. */
    NULL,             /* We are not using the task parameter. */
    PRIO_TASK_MAIN, /* Priority */ /* Range from 0 to 32, 32 being the highest priority */
    NULL,             /* We are not using the task handle. */
    1);               /* Core where the task should run */

  /* Check the semaphore was created successfully. */
  if (xBinarySemaphore != NULL)
  {
    /* Create the 'handler' task.  This is the task that will be synchronized
    with the interrupt.  The handler task is created with a high priority to
    ensure it runs immediately after the interrupt exits.  In this case a
    priority of 3 is chosen. */
    xTaskCreatePinnedToCore(vSDHandlerTask, "Handler", 1024, NULL, PRIO_TASK_HANDLER, NULL, 1);

    /* Start the scheduler so the created tasks start executing. */
    // vTaskStartScheduler();
  }
}

void loop()
{
  vTaskDelete(NULL);
}

//------------------------------------------------------------------------------

static void IRAM_ATTR vSDInterruptHandler(void)
{
  static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

  /* 'Give' the semaphore to unblock the task. */
  xSemaphoreGiveFromISR(xBinarySemaphore, (signed portBASE_TYPE*) & xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE)
  {
    portYIELD_FROM_ISR(); // goes back to the highest priority task ( vSDHandlerTask )
  }
}

static void vSDHandlerTask(void* pvParameters)
{
  const TickType_t xDelay5ms = 10 / portTICK_PERIOD_MS;
  /* Note that when you create a binary semaphore in FreeRTOS, it is ready
  to be taken, so you may want to take the semaphore after you create it
  so that the task waiting on this semaphore will block until given by
  another task. */
#ifdef DEBUG_SERIAL_
  Serial.println(" - vSDHandlerTask");
#endif

  /* As per most tasks, this task is implemented within an infinite loop. */
  for (;;)
  {
    /* Use the semaphore to wait for the event.  The semaphore was created
    before the scheduler was started so before this task ran for the first
    time.  The task blocks indefinitely meaning this function call will only
    return once the semaphore has been successfully obtained - so there is no
    need to check the returned value. */

    xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
    if (digitalRead(INTERRUPT_PIN) == LOW) {  // contact closed, SD inserted
    #ifdef DEBUG_SERIAL_
      Serial.println("LOW");
    #endif
      vTaskDelay(xDelay5ms);
      sdcardInserted = true;
    }
    else {  // SD card removed
    #ifdef DEBUG_SERIAL_
      Serial.println("HIGH");
    #endif
      vTaskDelay(xDelay5ms);
      sdcardInserted = false;
    }
  }
}

/* Idle hook functions MUST be called vApplicationIdleHook(), take no parameters,
and return void. */
bool my_vApplicationIdleHook(void)
{
  const TickType_t xDelay150ms = 150 / portTICK_PERIOD_MS;
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  static bool inv = false, isOff = false;
  static byte pwmVal = 0;

  if (sdcardInserted) {
    isOff = false;
    if (xLastWakeTime + xDelay150ms < xTaskGetTickCount()) {
      if ((pwmVal + 10) > 255)  //max
        inv = true;
      else if ((pwmVal - 10) < 50) //min
        inv = false;

      if (inv == true)
        pwmVal = pwmVal - 10;
      else
        pwmVal = pwmVal + 10;


      ledcWrite(LED_CHANNEL, pwmVal);
      xLastWakeTime = xTaskGetTickCount();
    }
  }
  else {
    if (isOff == false) {
      pwmVal = 0;
      ledcWrite(LED_CHANNEL, 255);
      isOff = true;
    }
  }

  return true;
}

//------------------------------------------------------------------------------

bool is_number(const std::string& s)
{
  return !s.empty() && std::find_if(s.begin(),
    s.end(), [](unsigned char c) { return !std::isdigit(c); }) == s.end();
}

void vMainTask(void* pvParameters)
{
  const TickType_t xDelay250ms = 250 / portTICK_PERIOD_MS;
  float ADCReceivedValue = 0;
  portBASE_TYPE xStatus;
  data_atm_t atmVars;
  data_si_t siVars;
  data_iim_t iimVars;
  std::string sdcString;
  std::string serStr;
  for (;;)
  {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    //std::string* pStr2;
    if (Serial.available()) { // read the character and do what you need
      String msg = Serial.readString();
      serStr.clear();
      serStr.append(std::string(msg.c_str()));
      std::string answer;

      std::string cmd("setdate");
      std::size_t found = serStr.find(cmd); //get pos of string match
      if (found != std::string::npos) {  // if post is valid
        int year = -1, month = -1, day = -1;
        // serStr contains cmd
        // setdate yyyy/MM/dd
        if (serStr.length() > 18) {
          std::string buffer;
          buffer.append(serStr.substr((found + 8), 4));
          if (is_number(buffer.c_str())) {
            year = std::stoi(buffer);
          }
          buffer.clear();
          buffer.append(serStr.substr((found + 8 + 4 + 1), 2));
          if (is_number(buffer.c_str())) {
            month = std::stoi(buffer);
          }
          buffer.clear();
          buffer.append(serStr.substr((found + 8 + 4 + 1 + 2 + 1), 2));
          if (is_number(buffer.c_str())) {
            day = std::stoi(buffer);
          }
        }
        if (year > 0 && (month > 0 && month < 13) && (day > 0 && day < 32)) {
          rtc.setTime(rtc.getSecond(), rtc.getMinute(), rtc.getHour(true), day, month, year, /* ms */ 0);
          answer.append("Date set.\n");
        }
        else {
          answer.append("Error, please enter date in format:\nsetdate yyyy/MM/dd\n");
        }
      }
      cmd.clear();
      cmd.append("settime");
      found = serStr.find(cmd); //get pos of string match
      if (found != std::string::npos) {  // if post is valid
        int hour = -1, min = -1, sec = -1;
        // serStr contains cmd
        // settime hh/mm/ss
        if (serStr.length() > 15) {
          std::string buffer;
          buffer.append(serStr.substr((found + 8), 2));
          if (is_number(buffer.c_str())) {
            hour = std::stoi(buffer);
          }
          buffer.clear();
          buffer.append(serStr.substr((found + 8 + 2 + 1), 2));
          if (is_number(buffer.c_str())) {
            min = std::stoi(buffer);
          }
          buffer.clear();
          buffer.append(serStr.substr((found + 8 + 2 + 1 + 2 + 1), 2));
          if (is_number(buffer.c_str())) {
            sec = std::stoi(buffer);
          }
        }
        if ((hour >= 0 && hour < 24) && (min >= 0 && min < 59) && (sec >= 0 && sec < 59)) {
          rtc.setTime(sec, min, hour, rtc.getDay(), rtc.getMonth() + 1, rtc.getYear(), /* ms */ 0);
          answer.append("Time set.\n");
        }
        else {
          answer.append("Error, please enter date in format:\nsettime hh:mm:ss\n");
        }

      }
      cmd.clear();
      cmd.append("getdate");
      found = serStr.find(cmd); //get pos of string match
      if (found != std::string::npos) {  // if post is valid
        String msg2 = rtc.getTime("Date of the RTC: %A, %B %d %Y %H:%M:%S");
        answer.append(std::string(msg2.c_str()));
      }
      cmd.clear();
      cmd.append("help");
      found = serStr.find(cmd); //get pos of string match
      if (found != std::string::npos) {  // if post is valid
        answer.append("Commands available:\nsetdate yyyy/MM/dd\nsettime hh/mm/ss\ngetdate\n");
      }

      if (answer.empty())
        answer.append("Unkown, type 'help' for commands.\n");
      //pStr2 = new std::string(answer.c_str());
      char a[300];
      strcpy(a,answer.c_str());
      
      xQueueSendToBack(xQueueBlue, &a, 0);

    }
    bool infoCollected = false;
    xStatus = xQueueReceive(xQueueADC, &ADCReceivedValue, 0);
    //......................|||||||||...\\\\\\\\\\\\\\\\-> copy values to local variable 
    //......................\\\\\\\\\-> Queue Name
    if (xStatus == pdPASS)
    {
      /* Data was successfully received from the queue, print out the received
      value. */
    #ifdef DEBUG_SERIAL_ADC
      Serial.print(", ADC = ");
      Serial.print(ADCReceivedValue);
      Serial.print(" V, ");
    #endif
      sdcString.append("ADC: ");
      sdcString.append(std::to_string(ADCReceivedValue));
      sdcString.append(" mV");
    }
    else
    {
      sdcString.append("ADC: - ");
      /* We did not receive anything from the queue even after waiting for 100ms.
      This must be an error as the sending tasks are free running and will be
      continuously writing to the queue. */
    #ifdef DEBUG_SERIAL_
      Serial.print("ADC: Could not receive from the queue.\r\n");
    #endif
    }
    sdcString.append(" # ");  // divider symbol
    xStatus = xQueueReceive(xQueueATM, &atmVars, 0);
    if (xStatus == pdPASS) {
    #ifdef DEBUG_SERIAL_ATM
      Serial.print("ATM ");
      Serial.print(atmVars.sysStatus0);
      Serial.print(", ");
      Serial.print(atmVars.sysStatus1);
      Serial.print(", ");
      Serial.print(atmVars.meterStatus0);
      Serial.print(", ");
      Serial.print(atmVars.meterStatus1);
      Serial.print(", ");
      Serial.print(atmVars.temperature);
      Serial.println(". ");
    #endif
      sdcString.append("ATM90E36: ");
      sdcString.append("  sysStatus01[");
      sdcString.append(std::to_string(atmVars.sysStatus0));
      sdcString.append(" ");
      sdcString.append(std::to_string(atmVars.sysStatus1));
      sdcString.append("], meterStatus01[");
      sdcString.append(std::to_string(atmVars.meterStatus0));
      sdcString.append(" ");
      sdcString.append(std::to_string(atmVars.meterStatus1));
      sdcString.append("], Va=");
      sdcString.append(std::to_string(atmVars.voltageA));
      sdcString.append(" V, Vb=");
      sdcString.append(std::to_string(atmVars.voltageB));
      sdcString.append(" V, Vc=");
      sdcString.append(std::to_string(atmVars.voltageC));
      sdcString.append(" V, Ia=");
      sdcString.append(std::to_string(atmVars.currentA));
      sdcString.append(" A, Ib=");
      sdcString.append(std::to_string(atmVars.currentB));
      sdcString.append(" A, Ic=");
      sdcString.append(std::to_string(atmVars.currentC));
      sdcString.append(" A, IN=");
      sdcString.append(std::to_string(atmVars.currentN));
      sdcString.append(" A, IN(calc)=");
      sdcString.append(std::to_string(atmVars.currentCalcN));
      sdcString.append(" A, PhA=");
      sdcString.append(std::to_string(atmVars.phaseA));
      sdcString.append(" deg., PhB=");
      sdcString.append(std::to_string(atmVars.phaseB));
      sdcString.append(" deg., PhC=");
      sdcString.append(std::to_string(atmVars.phaseC));
      sdcString.append(" deg., PowFactorT=");
      sdcString.append(std::to_string(atmVars.powerFactorTotal));
      sdcString.append(" , PowFactorA=");
      sdcString.append(std::to_string(atmVars.powerFactorA));
      sdcString.append(" , PowFactorB=");
      sdcString.append(std::to_string(atmVars.powerFactorB));
      sdcString.append(" , PowFactorC=");
      sdcString.append(std::to_string(atmVars.powerFactorC));
      sdcString.append(" , f=");
      sdcString.append(std::to_string(atmVars.frequency));
      sdcString.append(" Hz, P=");
      sdcString.append(std::to_string(atmVars.activePowerTotal));
      sdcString.append(" W, S=");
      sdcString.append(std::to_string(atmVars.apparentPowerTotal));
      sdcString.append(" VA, Q=");
      sdcString.append(std::to_string(atmVars.apparentPowerTotal));
      sdcString.append(" VAR");
      infoCollected = true;
    }
    else {
      sdcString.append("ATM90E36: - ");
    #ifdef DEBUG_SERIAL_
      Serial.print("ATM90E36: Could not receive from the queue.\r\n");
    #endif
    }
    sdcString.append(" # ");  // divider symbol

    xStatus = xQueueReceive(xQueueSI, &siVars, 0);
    if (xStatus == pdPASS) {
      // values have been received
      sdcString.append("Si8021: ");
      sdcString.append("humidity= ");
      sdcString.append(std::to_string(siVars.humidity));
      sdcString.append("%, temperature= ");
      sdcString.append(std::to_string(siVars.temperature));
      sdcString.append(" deg.");
    }
    else {
      sdcString.append("Si8021: - ");
    #ifdef DEBUG_SERIAL_
      Serial.print("Si7021: Could not receive from the queue.\r\n");
    #endif
    }
    sdcString.append(" # ");  // divider symbol

    xStatus = xQueueReceive(xQueueIIM, &iimVars, 0);
    if (xStatus == pdPASS) {
      // values have been received
    #ifdef DEBUG_SERIAL_IIM
      Serial.print("IIM X: ");
      Serial.print(iimVars.accelX);
      Serial.print(", Y: ");
      Serial.print(iimVars.accelY);
      Serial.print(", Z: ");
      Serial.println(iimVars.accelZ);
    #endif
      sdcString.append("IIM-42352: ");
      sdcString.append("Accel.X= ");
      sdcString.append(std::to_string(iimVars.accelX));
      sdcString.append("g, Accel.Y= ");
      sdcString.append(std::to_string(iimVars.accelY));
      sdcString.append("g, Accel.Z= ");
      sdcString.append(std::to_string(iimVars.accelZ));
      sdcString.append("g");
    }
    else {
      sdcString.append("IIM-42352: - ");
    #ifdef DEBUG_SERIAL_
      //Serial.print("IIM: Could not receive from the queue.\r\n");
      Serial.print("IIM queue f ");
    #endif
    }
  #ifdef DEBUG_SERIAL_
    Serial.flush();
  #endif
    if (infoCollected) {
      std::string* pStr = new std::string(sdcString.c_str());
      xQueueSendToBack(xQueueSD, &pStr, 0);
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay250ms); // "Smart" Delay (250ms)
    sdcString.clear();
    //delete pStr2;
  }
}

//------------------------------------------------------------------------------

static void vSerialSendTask(void* pvParameters)
{
  char* pcMessageToPrint;

  for (;; )
  {
    /* Wait for a message to arrive. */
    char data[300];
    xQueueReceive(xQueueBlue, &data, portMAX_DELAY);
    std::string pStr;
    pStr.append(data);;

    Serial.println(pStr.c_str());
    Serial.flush();
  }
}

//------------------------------------------------------------------------------

// Statically allocate and initialize the spinlock
static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
#ifdef DEBUG_SERIAL_SD
  Serial.printf("Listing directory: %s\n", dirname);
#endif

  File root = fs.open(dirname);
  if (!root) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Failed to open directory");
  #endif
    return;
  }
  if (!root.isDirectory()) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Not a directory");
  #endif
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
    #ifdef DEBUG_SERIAL_SD
      Serial.print("  DIR : ");
      Serial.println(file.name());
    #endif
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    }
    else {
    #ifdef DEBUG_SERIAL_SD
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    #endif
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS& fs, const char* path) {
#ifdef DEBUG_SERIAL_SD
  Serial.printf("Creating Dir: %s\n", path);
#endif
  if (fs.mkdir(path)) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Dir created");
  #endif
  }
  else {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("mkdir failed");
  #endif
  }
}

void removeDir(fs::FS& fs, const char* path) {
#ifdef DEBUG_SERIAL_SD
  Serial.printf("Removing Dir: %s\n", path);
#endif
  if (fs.rmdir(path)) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Dir removed");
  #endif
  }
  else {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("rmdir failed");
  #endif
  }
}

void readFile(fs::FS& fs, const char* path) {
#ifdef DEBUG_SERIAL_SD
  Serial.printf("Reading file: %s\n", path);
#endif

  File file = fs.open(path);
  if (!file) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Failed to open file for reading");
  #endif
    return;
  }

#ifdef DEBUG_SERIAL_SD
  Serial.print("Read from file: ");
#endif
  while (file.available()) {
  #ifdef DEBUG_SERIAL_SD
    Serial.write(file.read());
  #endif
  }
  file.close();
}

void writeFile(fs::FS& fs, const char* path, const char* message) {
#ifdef DEBUG_SERIAL_SD
  Serial.printf("Writing file: %s\n", path);
#endif
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Failed to open file for writing");
  #endif
    return;
  }
  if (file.print(message)) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("File written");
  #endif
  }
  else {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Write failed");
  #endif
  }
  file.close();
}

void appendFile(fs::FS& fs, const char* path, const char* message) {
#ifdef DEBUG_SERIAL_SD
  Serial.printf("Appending to file: %s\n", path);
#endif

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Failed to open file for appending");
  #endif
    return;
  }
  if (file.print(message)) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Message appended");
  #endif
  }
  else {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Append failed");
  #endif
  }
  file.close();
}

void renameFile(fs::FS& fs, const char* path1, const char* path2) {
#ifdef DEBUG_SERIAL_SD
  Serial.printf("Renaming file %s to %s\n", path1, path2);
#endif
  if (fs.rename(path1, path2)) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("File renamed");
  #endif
  }
  else {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Rename failed");
  #endif
  }
}

void deleteFile(fs::FS& fs, const char* path) {
#ifdef DEBUG_SERIAL_SD
  Serial.printf("Deleting file: %s\n", path);
#endif
  if (fs.remove(path)) {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("File deleted");
  #endif
  }
  else {
  #ifdef DEBUG_SERIAL_SD
    Serial.println("Delete failed");
  #endif
  }
}

void vSDwriteTask(void* pvParameters) {
  const TickType_t xDelay250ms = 250 / portTICK_PERIOD_MS;
  portBASE_TYPE xStatus;
  bool writingToTempTxt = false;
#ifdef DEBUG_SERIAL_
  Serial.println(" - vSDwriteTask");
#endif
  int year = 0, month = -1, day = 0; // initial data is intentionally invalid
  std::string dirYearName, dirMonthName, fileName;

  while (sdcardInserted == false) {
    vTaskDelay(xDelay250ms);
  }
  SPIClass spi = HSPI;
  while (!SD.begin(15, spi, 4000000, "/sd", 5, false)) { // use this line for HSPI
  #ifdef DEBUG_SERIAL_
    Serial.println("Card Mount Failed");
  #endif    
    vTaskDelay(xDelay250ms);
  }
  if (rtc.getYear() < 2000) {
    fileName.append("/temp.txt");
    // save info on a file named "temp", if file exits, rename existing file
    uint8_t count = 0;
    xSemaphoreTake(xMutexHSPI, portMAX_DELAY);
    if (SD.exists("temp.txt")) {  // check if file exists
      std::string oldFileName, a;
      oldFileName.append("temp.txt");
      while (SD.exists(oldFileName.c_str()))
      {
        count++;
        oldFileName.erase(oldFileName.begin(), oldFileName.end());
        oldFileName.append("temp");
        oldFileName.append(std::to_string(count));
        oldFileName.append(".txt");
        if (count == 255) { // "temp255.txt"
          oldFileName.insert(0, "/"); // "/temp255.txt"
          deleteFile(SD, oldFileName.c_str());  // delete the file
          break;
        }
      }
      if (count != 255)
        a.append("/");
      a.append(oldFileName);
      renameFile(SD, "/temp.txt", a.c_str());  // rename old file, 
      // we can now write into "temp.txt" without likely losing previous info
    }
    xSemaphoreGive(xMutexHSPI);
    writingToTempTxt = true;
  }

  for (;;) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    if (digitalRead(INTERRUPT_PIN) == LOW) { // SDCard inserted
      std::string* pStr = NULL;
      xStatus = xQueueReceive(xQueueSD, &pStr, 0);
      std::string cpy;
      cpy.clear();
      if (xStatus == pdPASS) {
        cpy = *pStr;
      #ifdef DEBUG_SERIAL_SDWRITE
        Serial.print(" ,");
        Serial.print(cpy->c_str());
      #endif
        xSemaphoreTake(xMutexHSPI, portMAX_DELAY);
        if (rtc.getYear() < 2000) { // true if rtc not adjusted
          // save info on a file named "temp", if file exits, rename existing file
          std::string content;
          content.append(cpy.c_str());
          content.append("\n");
          writeFile(SD, fileName.c_str(), content.c_str());
        }
        else {  // rtc is adjusted 
          if ((rtc.getDay() != day) || (rtc.getMonth() != month) || (rtc.getYear() != year)) { // check if date has changed
            day = rtc.getDay();
            if ((rtc.getMonth() != month) || (rtc.getYear() != year)) {
              month = rtc.getMonth();
              dirMonthName.clear();
              dirMonthName.append("/");
              switch (month)
              {
              case 0:
                dirMonthName.append("Jan");
                break;
              case 1:
                dirMonthName.append("Fev");
                break;
              case 2:
                dirMonthName.append("Mar");
                break;
              case 3:
                dirMonthName.append("Abr");
                break;
              case 4:
                dirMonthName.append("Mai");
                break;
              case 5:
                dirMonthName.append("Jun");
                break;
              case 6:
                dirMonthName.append("Jul");
                break;
              case 7:
                dirMonthName.append("Ago");
                break;
              case 8:
                dirMonthName.append("Set");
                break;
              case 9:
                dirMonthName.append("Out");
                break;
              case 10:
                dirMonthName.append("Nov");
                break;
              case 11:
                dirMonthName.append("Dez");
                break;

              default:
                dirMonthName.append("unknown");
                break;
              }
              if (rtc.getYear() != year) {
                year = rtc.getYear();
                dirYearName.clear();
                dirYearName.append("/");
                dirYearName.append(std::to_string(year));
                createDir(SD, dirYearName.c_str());
              }
              std::string str;
              str.append(dirYearName);
              str.append(dirMonthName);
              createDir(SD, str.c_str());
            }
            fileName.clear();
            fileName.append(dirYearName);   // "/2023"
            fileName.append(dirMonthName);  // "/2023/Jan"
            fileName.append("/");           // "/2023/Jan/"
            fileName.append(std::to_string(day)); // "/2023/Jan/1"
            fileName.append(".txt");        // "/2023/Jan/1.txt"
            //update folders
          }
          vTaskPrioritySet(NULL, PRIO_TASK_MAIN + 1); // rise priority
          std::string content;
          content.clear();
          content.append("Time: ");
          content.append(std::to_string(rtc.getHour(true)));
          content.append("h");
          content.append(std::to_string(rtc.getMinute()));
          content.append(" ");
          content.append(std::to_string(rtc.getSecond()));
          content.append("s");
          content.append(" # ");
          content.append(cpy.c_str());
          content.append("\r\n");
          if (writingToTempTxt == true) { // if we just now got the updated time/date
            renameFile(SD, "/temp.txt", fileName.c_str());  // We'll rename the temp.txt to keep its contents
            writingToTempTxt = false;
          }
          taskENTER_CRITICAL(&my_spinlock);
          appendFile(SD, /* path */ fileName.c_str(), /* text */ content.c_str());
          taskEXIT_CRITICAL(&my_spinlock);
          vTaskPrioritySet(NULL, PRIO_TASK_SDWRITE);  // lower priority
        }
        xSemaphoreGive(xMutexHSPI);
      }
    #ifdef DEBUG_SERIAL_SDWRITE
      else {
        Serial.print("SD xStatus != pdPASS");
      }
    #endif
      delete pStr;
    }
    else {
      while (digitalRead(INTERRUPT_PIN) == HIGH)  // while SDCard not inserted
      {
        vTaskDelayUntil(&xLastWakeTime, xDelay250ms); // "Smart" Delay (250ms)
        TickType_t xLastWakeTime = xTaskGetTickCount();
      }
      while (!SD.begin(15, spi, 4000000, "/sd", 5, false)) { // use this line for HSPI
      #ifdef DEBUG_SERIAL_
        Serial.println("Card Mount Failed");
      #endif    
        vTaskDelayUntil(&xLastWakeTime, xDelay250ms); // "Smart" Delay (250ms)
        TickType_t xLastWakeTime = xTaskGetTickCount();
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xDelay250ms); // "Smart" Delay (250ms)
  }
}

//------------------------------------------------------------------------------

void vSiTask(void* pvParameters) {
  const TickType_t xDelay250ms = 250 / portTICK_PERIOD_MS;
  data_si_t siData;

#ifdef DEBUG_SERIAL_
  Serial.println(" - vSiTask");
#endif

  if (!sensor.begin())
  {
  #ifdef DEBUG_SERIAL_
    Serial.println("Did not find Si7021 sensor!");
  #endif
    vTaskDelete(NULL);
    // sensor.heater(enableHeater); // Causes a 1.8 ºC difference
  }
  else
    for (;;) {
      xSemaphoreTake(xMutexI2C, portMAX_DELAY);
      TickType_t xLastWakeTime = xTaskGetTickCount();

      siData.humidity = sensor.readHumidity();
      siData.temperature = sensor.readTemperature();

      xQueueSendToBack(xQueueSI, &siData, 0);
      xSemaphoreGive(xMutexI2C);
      vTaskDelayUntil(&xLastWakeTime, xDelay250ms); // "Smart" Delay (250ms)
    }
}

//------------------------------------------------------------------------------

void vIImTask(void* pvParameters) {
  const TickType_t xDelay250ms = 250 / portTICK_PERIOD_MS;
  data_iim_t iimData;

#ifdef DEBUG_SERIAL_
  Serial.println(" - vIImTask");
#endif
  byte rc;
  float acc[3];
  bool init = false;
  IIM42SENSOR.init();
  for (;;) {
    xSemaphoreTake(xMutexI2C, portMAX_DELAY);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    if (init == false) {
      rc = IIM42SENSOR.init();
      if (rc != 0) {
      #ifdef DEBUG_SERIAL_
        //Serial.print(F("IIM42 sensor initialization failed. "));
        Serial.print(F("IIM ini f "));
      #endif
      }
      else {
        init = true;
      }
    }
    if (init == true) {
      rc = IIM42SENSOR.get_val(acc);
      if (Wire._twoWireLastError == -1) {
        init = false;
      }
      else if (rc == 0) {
        iimData.accelX = acc[0];
        iimData.accelY = acc[1];
        iimData.accelZ = acc[2];
        xQueueSendToBack(xQueueIIM, &iimData, 0);
      }
    }

    xSemaphoreGive(xMutexI2C);
    vTaskDelayUntil(&xLastWakeTime, xDelay250ms);
  }
}

//------------------------------------------------------------------------------

void vAtmTask(void* pvParameters) {
  const TickType_t xDelay250ms = 250 / portTICK_PERIOD_MS;
  data_atm_t atmVars;

  for (;;) {
    xSemaphoreTake(xMutexSPI, portMAX_DELAY);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    atmVars.activePowerA = eic.GetActivePowerA();
    atmVars.activePowerB = eic.GetActivePowerB();
    atmVars.activePowerC = eic.GetActivePowerC();
    atmVars.activePowerTotal = eic.GetTotalActivePower();

    atmVars.voltageA = eic.GetLineVoltageA();
    atmVars.voltageB = eic.GetLineVoltageB();
    atmVars.voltageC = eic.GetLineVoltageC();

    atmVars.currentA = eic.GetLineCurrentA();
    atmVars.currentB = eic.GetLineCurrentB();
    atmVars.currentC = eic.GetLineCurrentC();
    atmVars.currentN = eic.GetLineCurrentN();
    atmVars.currentCalcN = eic.GetCalcLineCurrentN();

    atmVars.reactivePowerA = eic.GetReactivePowerA();
    atmVars.reactivePowerB = eic.GetReactivePowerB();
    atmVars.reactivePowerC = eic.GetReactivePowerC();
    atmVars.reactivePowerTotal = eic.GetTotalReactivePower();

    atmVars.apparentPowerA = eic.GetApparentPowerA();
    atmVars.apparentPowerB = eic.GetApparentPowerB();
    atmVars.apparentPowerC = eic.GetApparentPowerC();
    atmVars.apparentPowerTotal = eic.GetTotalApparentPower();

    atmVars.frequency = eic.GetFrequency();

    atmVars.powerFactorA = eic.GetPowerFactorA();
    atmVars.powerFactorB = eic.GetPowerFactorB();
    atmVars.powerFactorC = eic.GetPowerFactorC();
    atmVars.powerFactorTotal = eic.GetTotalPowerFactor();

    atmVars.voltageHarmonicsA = eic.GetVHarmA();
    atmVars.voltageHarmonicsB = eic.GetVHarmB();
    atmVars.voltageHarmonicsC = eic.GetVHarmC();

    atmVars.currentHarmonicsA = eic.GetCHarmA();
    atmVars.currentHarmonicsB = eic.GetCHarmB();
    atmVars.currentHarmonicsC = eic.GetCHarmC();

    atmVars.phaseA = eic.GetPhaseA();
    atmVars.phaseB = eic.GetPhaseB();
    atmVars.phaseC = eic.GetPhaseC();

    atmVars.temperature = eic.GetTemperature();

    atmVars.importEnergy = eic.GetImportEnergy();
    atmVars.exportEnergy = eic.GetExportEnergy();

    /* System Status */
    atmVars.sysStatus0 = eic.GetSysStatus0();
    atmVars.sysStatus1 = eic.GetSysStatus1();
    atmVars.meterStatus0 = eic.GetMeterStatus0();
    atmVars.meterStatus1 = eic.GetMeterStatus1();

    xQueueSendToBack(xQueueATM, &atmVars, 0);

    xSemaphoreGive(xMutexSPI);
    vTaskDelayUntil(&xLastWakeTime, xDelay250ms); // "Smart" Delay (250ms)
  }
}

//------------------------------------------------------------------------------

#define SSID "SSID"
#define PASSWORD "PASSWORD"
void vWiFiTask(void* pvParameters)
{
  const TickType_t xDelay250ms = 250 / portTICK_PERIOD_MS;

#ifdef DEBUG_SERIAL_
  Serial.println(" - vWiFiTask");
#endif
  uint8_t i;
  bool cancelWiFiSearch = false;
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(xDelay250ms); // Delay (250ms)
    i++;
    if (i > 40)                // if 10 seconds have passed and WiFi haven't connected
    {                          // ~10s
      cancelWiFiSearch = true; // do not adjust RTC
      break;
    }
  }

  if (cancelWiFiSearch == false) // adjust the RTC
  {
    configTime(0 /* gmtOffset_sec */, /* daylightOffset_sec */ 0, /* ntpServer */ /* "pool.ntp.org" */ "pt.pool.ntp.org");

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
    #ifdef DEBUG_SERIAL_
      Serial.println("Failed to obtain time");
    #endif
      vTaskDelete(NULL);
    }

  #ifdef DEBUG_SERIAL_
    Serial.println(&timeinfo, "Date received from NTP: %A, %B %d %Y %H:%M:%S");
  #endif

    if (getLocalTime(&timeinfo))
    {
      rtc.setTimeStruct(timeinfo);
    }

  #ifdef DEBUG_SERIAL_
    Serial.println(rtc.getTime("Adjusted date of the RTC: %A, %B %d %Y %H:%M:%S"));
  #endif
  }

  // disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_deinit(); // poweroff WiFI and free resources

  vTaskDelete(NULL);
}

// ---

#define ADC_OFFSET_ADJUST 0.2 // 200mV
void vADCTask(void* pvParameters)
{
  const TickType_t xDelay250ms = 250 / portTICK_PERIOD_MS;
  float adcValue = 0;
  float lValueToSend;
#ifdef DEBUG_SERIAL_
  Serial.println(" - vADCTask");
#endif
  pinMode(PIN_ADC1_0, INPUT);
  /* analogReadResolution(10);
  analogSetWidth(10);
  analogSetClockDiv(2); */
  for (;;) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Read ADC, default A=11dB, up to 2450mV (recommended range)
    adcValue = analogRead(PIN_ADC1_0); // GPIO2, uses ADC2_x, requires WiFi deactivated
    // do conversion
    /*  3V3 ~ 4095,     3V3 = 3300mV
        Vin ~ adcValue
    */
    lValueToSend = (adcValue * 3300) / 4095.0 + ADC_OFFSET_ADJUST;  // mV
    // Write to Queue
    xQueueSendToBack(xQueueADC, &lValueToSend, 0);
    vTaskDelayUntil(&xLastWakeTime, xDelay250ms); // "Smart" Delay (250ms)
  }
}
