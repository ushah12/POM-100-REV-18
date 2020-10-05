/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


  #ifndef _VARIANT_FEATHER52832_
  #define _VARIANT_FEATHER52832_

  Master clock frequency
  #define VARIANT_MCK       (64000000ul)

  #define USE_LFXO      // Board uses 32khz crystal for LF
  // define USE_LFRC    // Board uses RC for LF

  ----------------------------------------------------------------------------
        Headers
  ----------------------------------------------------------------------------

  #include "WVariant.h"

  #ifdef __cplusplus
  extern "C"
  {
  #endif // __cplusplus

  // Number of pins defined in PinDescription array
  #define PINS_COUNT           (32u)
  #define NUM_DIGITAL_PINS     (32u)
  #define NUM_ANALOG_INPUTS    (8u)
  #define NUM_ANALOG_OUTPUTS   (0u)

  // LEDs
  #define PIN_LED1             (17)
  #define PIN_LED2             (8)

  #define LED_BUILTIN          PIN_LED1
  #define LED_CONN             PIN_LED2

  #define LED_RED              PIN_LED1
  #define LED_BLUE             PIN_LED2

  #define LED_STATE_ON         1         // State when LED is litted


  Analog pins

  #define PIN_A0               (2)
  #define PIN_A1               (3)
  #define PIN_A2               (4)
  #define PIN_A3               (5)
  #define PIN_A4               (8)//(28)
  #define PIN_A5               (8)//(29)
  #define PIN_A6               (8)//(30)
  #define PIN_A7               (8)//(31)

  static const uint8_t A0  = PIN_A0 ;
  static const uint8_t A1  = PIN_A1 ;
  static const uint8_t A2  = PIN_A2 ;
  static const uint8_t A3  = PIN_A3 ;
  static const uint8_t A4  = PIN_A4 ;
  static const uint8_t A5  = PIN_A5 ;
  static const uint8_t A6  = PIN_A6 ;
  static const uint8_t A7  = PIN_A7 ;
  #define ADC_RESOLUTION    14

  // Other pins
  #define PIN_AREF           (24)
  #define PIN_VBAT           PIN_A7
  #define PIN_NFC1           (9)
  #define PIN_NFC2           (10)

  static const uint8_t AREF = PIN_AREF;


  Serial interfaces

  #define PIN_SERIAL_RX       (3)
  #define PIN_SERIAL_TX       (2)


  SPI Interfaces

  #define SPI_INTERFACES_COUNT 1

  #define PIN_SPI_MISO         (31)
  #define PIN_SPI_MOSI         (11)
  #define PIN_SPI_SCK          (30)

  static const uint8_t SS   = 4 ;
  static const uint8_t MOSI = PIN_SPI_MOSI ;
  static const uint8_t MISO = PIN_SPI_MISO ;
  static const uint8_t SCK  = PIN_SPI_SCK ;


  Wire Interfaces

  #define WIRE_INTERFACES_COUNT 1

  #define PIN_WIRE_SDA         (29)
  #define PIN_WIRE_SCL         (28)

  #define EXTERNAL_FLASH_DEVICES   S25FL064L
  #define EXTERNAL_FLASH_USE_SPI
  #define EXTERNAL_FLASH_USE_CS 4
  #ifdef __cplusplus
  }
  #endif

  //----------------------------------------------------------------------------
  //        Arduino objects - C++ only
  //----------------------------------------------------------------------------

  #endif

*/

#define Version "0.0.9"
#include <bluefruit.h>
#include "debug.h"
#include "BMA253.h"
#include "SerialFlash.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "SerialFlash.h"
#include <SPI.h>
#include "RTCInt.h"
#include <nrfx_wdt.h>
#include "AES128.h"
#include <nrfx_irqs_nrf52832.h>
RTCInt rtc;  //create an RTCInt type object


using namespace Adafruit_LittleFS_Namespace;
#define FILENAME    "/data.txt"
#define FILENAMEE    "/configration.txt"
#define ADDRESS_FILENAME     "/address_count.txt"
#define  REC_SAVE             "/records.txt"
#define FILENAMEEIS            "/password_update.txt"
#define RTC_TIME_FILE                    "/SaveRTC_Time.txt"
#define BUTTON_OFF_TIME 3000
#define BUTTON_ON_TIME 3000

#define INTR_COUNTER 5

#define millis_factor 2    // Used because millis is running fast (2 times)

#define FLASH_POWER_ONE 19
#define FLASH_POWER_TWO 18
#define total_time    1000
#define led_time      100
#define ARRAY_SIZE 40
#define GAP_RESOLUTION   5


File file(InternalFS);



bool low_battery = false;
bool disconnected = true;
bool set_time = false;
bool bmaRegVal = false;
bool scan = true;
bool conf_opened = false;
bool power_save = false;
bool max_records_reached = false;
bool read_btwn_adr = false;
bool start_addr = false;
bool end_addr = false;

uint32_t bma_intr_count = 0;
uint16_t my_conn_handle;
int8_t   rssi;
bool red_var = false;
bool green_var = false;
bool blue_var = false;
bool ledblink = false;

int8_t red_led_distance_cmp = 1;    //1m
int8_t blue_led_distance_cmp = 2;     //2m
int8_t  green_led_distance_cmp = 3; //3m
uint32_t STARTT_ADDRESS;
uint32_t ENDD_ADDRESS;
//const long onDuration = 800;
//const long offDuration = 800;
int no_distancecount = 0;
int red_LEDState = HIGH;
int green_LEDState = HIGH;
int blue_LEDState = HIGH;

int red_LEDState2 = HIGH;
int green_LEDState2 = HIGH;
int blue_LEDState2 = HIGH;

int buzzer_state = 0;
int vib_state = 0;
int vibrator_state = LOW;

String PASSWORD_KEY = "12345678";  //default
String MASTER_PASSWORD  = "12345678";
//bool intrpt_value = false;
int record_num ;
uint32_t record_diff;
uint32_t readlen;
uint32_t flash_write_address = 0;
uint32_t flash_read_address = 4096;           //Start from 4096 to create a Ring Buffer.
uint32_t last_read_addr = 0;

unsigned long red_rememberTime = 0;
unsigned long green_rememberTime = 0;
unsigned long blue_rememberTime = 0;
unsigned long rememberTimeBuzzer = 0;
unsigned long vibrator_rememberTime = 0;
int8_t buzzer_distance_cmp = 4; // 1m
int8_t vibrator_distance_cmp = 4; // 2m
int scan_interval_update = 8;  // 8 seconds
uint16_t max_records_saved = 100;//4;  // 1 no of records
int scan_gap_update = 6;  // 60 seconds with 5 resolution
uint8_t log_thresh_update = 4;
uint16_t adv_update = 500;  //in seconds

uint8_t timee[6];
const int maxValue = bit(12) - 1;

const unsigned long long eventinterval = 600000;///900000;    //battery voltage update
const unsigned long long notifyinterval = 60000;  // notify update

unsigned long long previoustime = 0;
unsigned long long notifytime = 0;

const unsigned long buzzer_interval = 5000;
unsigned long buzzer_ptime = 0;

const unsigned long vibrator_interval = 5000;
unsigned long vibrator_ptime = 0;

const unsigned long redled_interval = 5000;
unsigned long redled_ptime = 0;

const unsigned long greenled_interval = 5000;
unsigned long greenled_ptime = 0;

const unsigned long blueled_interval = 5000;
unsigned long blueled_ptime = 0;

unsigned long button_time = 0;
bool button_pressed = false;
unsigned long advbutton_time = 0;

unsigned long long bma_sleep = 0;
//default value
uint8_t voltper;
uint8_t countt = 0;
float ALPHA = 0.6;

float lleast_distance = 50.00;
//uint8_t least_distance = 50;
float least_distance = 50.00;
const int FlashChipSelect = 4;
uint64_t scantime = 0;
unsigned long flash_address = 0;
String data2;
char buff2[3];

uint32_t  MEND_ADDRESS = 8388608; // LAST ADDRESS OF FLASH
uint32_t  flash_index;
bool memful;

# define red_led   27        //GPIO 29 RED 
# define green_led  25      // GPIO 30 GREEN 
# define blue_led  26     // GPIO 31 BLUE

# define red_led2   16        //GPIO 15 RED 
# define green_led2  14      // GPIO 13 GREEN 
# define blue_led2  15      // GPIO 14 BLUE


# define  buzzer  5         // GPIO 23 PIEZEO
# define vibrator  22    //GPIO 36
const int slaveSelectPin = 4;

/**** BMA Pins ***/
#define SCL 28
#define SDA 29

#define BMA_INTERRUPT 23

#define TURNOFF_TIME 900000            //15 Minutes in Milli Seconds tto turn off System if No BMA Interrupt is available.
const int BUTTON = 24;   //GIO 38 SWITCH
int BUTTONstate = 0;


//GPIO 36 MOTOR
uint8_t vbat_per = 0;
uint64_t scanafter;
uint32_t dev_count = 0;
#define TIMEOUT_MS     (2500)
uint16_t rec_counter = 0;

uint64_t systemOnTime = 0;

uint8_t batt_firm[2] = {0x64, 0x11};
uint8_t mac[6];
uint8_t flag[8] = {mac[5], mac[4], mac[3], mac[2], mac[1], mac[0], batt_firm[0], batt_firm[1]};


bool command = false;
bool read_between_address = false;
bool hwPWM = false;
bool power_off = false;
bool wakeup = false;
///

byte key[] = { 
};
AES128 aes(key);
//
////////////////////////////// 1st service & char

const uint8_t CONF_UUID_SERVICE[] =  //PRIMARY SERVICE
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x2D, 0x4D, 0x4F, 0x50
};

const uint8_t RED_LED_THRESH_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x3D, 0x5D, 0x4F, 0x50
};
const uint8_t GREEN_LED_THRESH_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x4D, 0x5D, 0x4F, 0x50
};
const uint8_t BLUE_LED_THRESH_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x5D, 0x5D, 0x4F, 0x50

};
const uint8_t BUZZER_THRESH_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x4D, 0x6D, 0x4F, 0x50
};
const uint8_t VIBRATOR_THRESH_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x5D, 0x7D, 0x4F, 0x50
};
const uint8_t MAX_RECORDS_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x6D, 0x8D, 0x4F, 0x50
};
const uint8_t SCAN_INTERVAL_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x7D, 0x9D, 0x4F, 0x50
};
const uint8_t SCAN_GAP_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x8D, 0xAD, 0x4F, 0x50
};

const uint8_t LOG_THRESH_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x9D, 0xBD, 0x4F, 0x50
};

const uint8_t ADV_GAP_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0xAD, 0xCD, 0x4F, 0x50
};
const uint8_t CONF_PSWD_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0xBD, 0xDD, 0x4F, 0x50
};
const uint8_t UPDATE_PSWD_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0xCD, 0xED, 0x4F, 0x50
};
const uint8_t FIND_ME_UUID[] =
{
  0x31, 0x2D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0xDD, 0xFD, 0x4F, 0x50
};



////////////////////////////// 2nd service & char

const uint8_t TIME_UUID[] =
{
  0x32, 0x3D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x2D, 0x4D, 0x4F, 0x50
};

const uint8_t SET_TIME_UUID[] =
{
  0x32, 0x3D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x3D, 0x5D, 0x4F, 0x50
};



////////////////////////////// 3rd service & char

const uint8_t READ_DATA_UUID[] =
{
  0x33, 0x4D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x2D, 0x4D, 0x4F, 0x50
};

const uint8_t COMMMAND_UUID[] =
{
  0x33, 0x4D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x3D, 0x5D, 0x4F, 0x50
};
const uint8_t SET_DATA_UUID[] =
{
  0x33, 0x4D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x4D, 0x5D, 0x4F, 0x50
};

const uint8_t STARTINGGG_ADDRESS_UUID[] =
{
  0x33, 0x4D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x4D, 0x6D, 0x4F, 0x50
};

const uint8_t ENDINGG_ADDRESS_UUID[] =
{
  0x33, 0x4D, 0x44, 0x4B, 0x50, 0x2D, 0x50, 0x45,
  0x2D, 0x30, 0x30, 0x31, 0x5D, 0x7D, 0x4F, 0x50
};


////////////////////////////// 1st service & char


BLEService        Conf(CONF_UUID_SERVICE);
BLECharacteristic Red_Led_thresh(RED_LED_THRESH_UUID);
BLECharacteristic Green_Led_thresh(GREEN_LED_THRESH_UUID);
BLECharacteristic Blue_Led_thresh(BLUE_LED_THRESH_UUID);
BLECharacteristic Buzzer_thresh(BUZZER_THRESH_UUID);
BLECharacteristic Vibrator_thresh(VIBRATOR_THRESH_UUID);
BLECharacteristic Max_records(MAX_RECORDS_UUID);
BLECharacteristic Scan_interval(SCAN_INTERVAL_UUID);
BLECharacteristic Scan_gap(SCAN_GAP_UUID);
BLECharacteristic Log_thresh(LOG_THRESH_UUID);
BLECharacteristic Adv_gap(ADV_GAP_UUID);
//BLECharacteristic Conf_pswd(CONF_PSWD_UUID);
//BLECharacteristic Update_pswd(UPDATE_PSWD_UUID);
BLECharacteristic Find_me(FIND_ME_UUID);

////////////////////////////// 2nd service
BLEService        Time(TIME_UUID);
BLECharacteristic Set_time(SET_TIME_UUID);

////////////////////////////// 3rd service
BLEService        Read(READ_DATA_UUID);
BLECharacteristic Command(COMMMAND_UUID);
BLECharacteristic Start_address( STARTINGGG_ADDRESS_UUID);
BLECharacteristic End_address(ENDINGG_ADDRESS_UUID);
BLECharacteristic Set_data(SET_DATA_UUID);

// OTA DFU service
// OTA DFU service
// OTA DFU service
BLEDfu bledfu;

// Peripheral uart service
//BLEUart bleuart;

// Central uart client
BLEClientUart clientUart;



typedef struct data_record_s
{
  uint8_t  address[6];     // Six byte device address
  int8_t   rssid[250];       // RSSI value
  float dist;
  uint8_t _count;             //RSSI COUNT OF EVERY DEVICE
  int16_t averageRSSI;
  uint8_t batterylevel;
  uint8_t firm_version;
} data_record_t;

data_record_t data[ARRAY_SIZE];
typedef struct node_record_s
{
  uint8_t  addr[6];    // Six byte device address
  uint8_t   distance;
  uint8_t timestamp[6];
  uint8_t battery ;
  uint8_t firmware  ;   // Padding for word alignment
} node_record_t;

node_record_t records[ARRAY_SIZE];

void wdt_init(void)
{
  NVIC_SetPriority(WDT_IRQn, 7);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_EnableIRQ(WDT_IRQn);
  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV = 15 * 32768; // 150 sec. timout
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk; //Enable reload register 0
  NRF_WDT->INTENSET = WDT_INTENSET_TIMEOUT_Msk;//1;//WDT_INTENSET_TIMEOUT_Msk;
  //NRF_WDT->EVENTS_TIMEOUT = 1;
  NRF_WDT->TASKS_START = 1;
}

void setup()
{

  uint32_t u32Reset_reason = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = NRF_POWER->RESETREAS;
  pinMode (red_led, OUTPUT);
  pinMode (green_led, OUTPUT);
  pinMode (blue_led, OUTPUT);

  wdt_init();
  rtc.begin(TIME_H24); //init RTC in 24 hour mode
  //time settings
  pinMode (red_led2, OUTPUT);
  pinMode (green_led2, OUTPUT);
  pinMode (blue_led2, OUTPUT);


  pinMode (buzzer, OUTPUT);
  pinMode (vibrator, OUTPUT);
  pinMode(BUTTON, INPUT);


  pinMode(BMA_INTERRUPT, INPUT);

  //Flash Pin
  pinMode(FLASH_POWER_ONE, INPUT);    //flash power pin GPIO 35
  pinMode(FLASH_POWER_TWO, INPUT); // flash supply pin GPIO 16


  //digitalWrite(FLASH_POWER_ONE, HIGH);
  //digitalWrite(FLASH_POWER_TWO, HIGH);
  pinMode(slaveSelectPin, OUTPUT);


  digitalWrite(red_led, red_LEDState);
  digitalWrite(green_led, green_LEDState);
  digitalWrite(blue_led, blue_LEDState);

  digitalWrite(red_led2, red_LEDState2);
  digitalWrite(green_led2, green_LEDState2);
  digitalWrite(blue_led2, blue_LEDState2);

  digitalWrite(buzzer, buzzer_state);
  digitalWrite(vibrator, vibrator_state);
  attachInterrupt(BUTTON, NULL, LOW);
  // attachInterrupt(BMA_INTERRUPT, digital_callback,  LOW);
  //  attachInterrupt(BMA_INTERRUPT, digital_callback, CHANGE /*&& RISING*/);
  // analogWriteResolution( 12 );
  ////analogWrite(buzzer,2048);

  //  HwPWM0.addPin( buzzer );
  //  HwPWM0.begin();
  //  HwPWM0.setResolution(12);
  //  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_1);
  //  HwPWM0.stop();


  HwPWM1.addPin( vibrator );
  HwPWM1.begin();
  HwPWM1.setResolution(12);
  HwPWM1.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_1);
  HwPWM1.stop();


  HwPWM0.addPin( buzzer );
  HwPWM0.begin();
  HwPWM0.setResolution(12);
  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_1);
  HwPWM0.stop();

getkey();
AES128 aes(key);
D_init(115200);
  SerialFlash.begin(slaveSelectPin);
  S_PRINT("Reset Reason = "); S_PRINTLN(u32Reset_reason, HEX);
  S_PRINTLN("BEGIN");
  // Serial.println("PASSWORD_KEY at start = ");  Serial.print(PASSWORD_KEY);
  //smart_delay(200);
  chip_id();
  delay(1000);
  beginAccel();
  delay(1000);
  cellvoltage();
  if (low_battery == true) {
    for (int i = 0; i < 4; i++)
    {
      if (i % 2 == 0)
      {
        digitalWrite(red_led, LOW);
        digitalWrite(red_led2, LOW);
        digitalWrite(green_led2, LOW);
        digitalWrite(green_led, LOW);


      }
      else
      {

        digitalWrite(red_led, HIGH);
        digitalWrite(red_led2, HIGH);
        digitalWrite(green_led2, HIGH);
        digitalWrite(green_led, HIGH);
        delay(250);

      }
      delay(250);
    }
  }

  if (low_battery == false && u32Reset_reason != 2)
    startup_test();

  delay(1000);
  //smart_delay(200);
  //Serial.end();
  //nrf_cal_init();
  InternalFS.begin();
  if (u32Reset_reason == 2)
  {
    read_update_time();
  }
  en_flash();



  disable_flash();

  update_conf_var();
  password_updt_var();
  //readd_address();

  file.open(REC_SAVE, FILE_O_READ);
  if (file)
  {
    String myCount = "";
    char buf[9] ;
    memset(buf, 0, sizeof(buf));
    file.read(buf, sizeof(buf));
    myCount += String(buf);
    S_PRINT("myCount in = "); S_PRINTLN(myCount);
    rec_counter = (uint16_t)hex2Uint32(myCount.c_str());
    S_PRINT("rec_counter = "); S_PRINTLN(rec_counter);
    file.close();
  }
  D_PRINTLN("Bluefruit52 Dual Role BLEUART Example");
  D_PRINTLN("-------------------------------------\n");



  memset(records, 0, sizeof(records));
  for (uint8_t i = 0; i < ARRAY_SIZE; i++)
  {
    data[i].dist = 255;
    data[i].rssid[0] = -128;
  }
  //cellvoltage();
  Bluefruit.begin(1, 1);
  // Check bluefruit.h for supported values
  Bluefruit.setName("POM100");

  Bluefruit.setTxPower(DEVICE_TX_POWER);

  // Callbacks for Peripheral
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  bledfu.begin();

  // configure service
  Conf.begin();
  // set characteristics
  Red_Led_thresh.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Red_Led_thresh.setPermission(SECMODE_OPEN, SECMODE_OPEN);//
  Red_Led_thresh.setFixedLen(1);
  Red_Led_thresh.begin();
  Red_Led_thresh.write8(red_led_distance_cmp);
  Red_Led_thresh.setWriteCallback(red_led_write_callback);

  Green_Led_thresh.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Green_Led_thresh.setPermission(SECMODE_OPEN, SECMODE_OPEN);//
  Green_Led_thresh.setFixedLen(1);
  Green_Led_thresh.begin();
  Green_Led_thresh.write8(green_led_distance_cmp);
  Green_Led_thresh.setWriteCallback(green_led_write_callback);

  Blue_Led_thresh.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Blue_Led_thresh.setPermission(SECMODE_OPEN, SECMODE_OPEN);//
  Blue_Led_thresh.setFixedLen(1);
  Blue_Led_thresh.begin();
  Blue_Led_thresh.write8(blue_led_distance_cmp);
  Blue_Led_thresh.setWriteCallback(blue_led_write_callback);


  Buzzer_thresh.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE); //2
  Buzzer_thresh.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Buzzer_thresh.setFixedLen(1);
  Buzzer_thresh.begin();
  Buzzer_thresh.write8(buzzer_distance_cmp);
  Buzzer_thresh.setWriteCallback(buzzer_write_callback);


  Vibrator_thresh.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE); //3
  Vibrator_thresh.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Vibrator_thresh.setFixedLen(1);
  Vibrator_thresh.begin();
  Vibrator_thresh.write8(vibrator_distance_cmp);
  Vibrator_thresh.setWriteCallback(vibrator_write_callback);


  Max_records.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Max_records.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Max_records.setFixedLen(2);
  Max_records.begin();
  Max_records.write16(max_records_saved);
  Max_records.setWriteCallback(max_records_write_callback);


  Scan_interval.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Scan_interval.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Scan_interval.setFixedLen(1);                                   //5
  Scan_interval.begin();
  Scan_interval.write8(scan_interval_update);
  Scan_interval.setWriteCallback(scan_interval_write_callback);

  Scan_gap.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Scan_gap.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Scan_gap.setFixedLen(1);                                   //5
  Scan_gap.begin();
  Scan_gap.write8(scan_gap_update);
  Scan_gap.setWriteCallback(scan_gap_write_callback);

  Log_thresh.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Log_thresh.setPermission(SECMODE_OPEN, SECMODE_OPEN);            //6
  Log_thresh.setFixedLen(1);
  Log_thresh.begin();
  Log_thresh.write8(log_thresh_update);
  Log_thresh.setWriteCallback(log_thresh_write_callback);

   Adv_gap.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
   Adv_gap.setPermission(SECMODE_OPEN, SECMODE_OPEN);            //6
   Adv_gap.setFixedLen(2);
   Adv_gap.begin();
   Adv_gap.write16(adv_update);
   Adv_gap.setWriteCallback(adv_write_callback);


   // Conf_pswd.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
   // Conf_pswd.setPermission( SECMODE_NO_ACCESS, SECMODE_OPEN);            //6
    //Conf_pswd.setFixedLen(8);
   // Conf_pswd.begin();
   // Conf_pswd.setWriteCallback(conf_pswd_callback);


    //Update_pswd.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
   // Update_pswd.setPermission( SECMODE_NO_ACCESS, SECMODE_OPEN);            //6
   // Update_pswd.setFixedLen(8);
   // Update_pswd.begin();
   // Update_pswd.setWriteCallback(updt_pswd_callback);



    Find_me.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    Find_me.setPermission( SECMODE_NO_ACCESS, SECMODE_OPEN);            //6
    Find_me.setFixedLen(2);
    Find_me.begin();
    Find_me.setWriteCallback(find_me_callback);


  // second service
  Time.begin();
  Set_time.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Set_time.setPermission(SECMODE_OPEN, SECMODE_OPEN);//1       //byteset
  Set_time.setFixedLen(6);
  Set_time.begin();
  Set_time.setWriteCallback(set_time_write_callback);

  // third service
  Read.begin();


  Command.setProperties(CHR_PROPS_WRITE | CHR_PROPS_READ );
  Command.setPermission(SECMODE_OPEN,  SECMODE_OPEN);    //1     // byteset
  Command.setFixedLen(2);
  Command.begin();
  Command.setWriteCallback( Command_write_callback);


  Start_address.setProperties(CHR_PROPS_WRITE | CHR_PROPS_READ );
  Start_address.setPermission(SECMODE_OPEN,  SECMODE_OPEN);    //1     // byteset
  Start_address.setFixedLen(4);
  Start_address.begin();
  Start_address.setWriteCallback( start_address_callback);


  End_address.setProperties(CHR_PROPS_WRITE | CHR_PROPS_READ );
  End_address.setPermission(SECMODE_OPEN,  SECMODE_OPEN);    //1     // byteset
  End_address.setFixedLen(4);
  End_address.begin();
  End_address.setWriteCallback( end_address_callback);

  Set_data.setProperties( CHR_PROPS_NOTIFY);
  //Set_data.setPermission(SECMODE_OPEN,  SECMODE_NO_ACCESS);    //2   //byte??
  Set_data.setFixedLen(15);
  Set_data.begin();

  Bluefruit.setConnLedInterval(250);


  update_address(&flash_write_address, &flash_read_address);////////////////////////////////////////////////////////////////////////////////
  S_PRINT("flash_write_address = "); S_PRINTLN(flash_write_address);
  S_PRINT("flash_read_address = "); S_PRINTLN(flash_read_address);

  start_scan();
  D_PRINTLN("main millis =");
  D_PRINTLN(millis());

  // second service
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  // Set up and start advertising
  startAdv();
  button_time = millis();
  advbutton_time = millis();
  S_PRINTLN("BEGIN");


  //And this bit refreshes the WDT.  Just put it where ever you think is appropriate.
  NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0

}

void startup_test()
{
  digitalWrite (red_led, LOW);
  digitalWrite (red_led2, LOW);
  smart_delay(2000);
  digitalWrite (red_led, HIGH);
  digitalWrite (red_led2, HIGH);
  digitalWrite (green_led, LOW);
  digitalWrite (green_led2, LOW);
  smart_delay(2000);
  digitalWrite (green_led, HIGH);
  digitalWrite (green_led2, HIGH);
  digitalWrite (blue_led, LOW);
  digitalWrite (blue_led2, LOW);
  smart_delay(2000);
  digitalWrite (blue_led, HIGH);
  digitalWrite (blue_led2, HIGH);
    HwPWM0.begin();
  HwPWM0.writePin(buzzer, maxValue / 12, false);
  smart_delay(2000);
  HwPWM0.writePin(buzzer, 0, false);
   HwPWM1.begin();
  HwPWM1.writePin(vibrator, maxValue / 2, false);
  smart_delay(2000);
   HwPWM0.writePin(vibrator, 0, false);
  HwPWM0.stop();
  HwPWM1.stop();

}
void start_scan()
{
  if (low_battery == false) {
    Bluefruit.Scanner.setRxCallback(scan_callback);
    Bluefruit.Scanner.restartOnDisconnect(true);
    Bluefruit.Scanner.setInterval(SCAN_INTERVAL, SCAN_WINDOW); // in unit of 0.625 ms
    Bluefruit.Scanner.filterUuid(CONF_UUID_SERVICE);
    Bluefruit.Scanner.useActiveScan(false);
    if (low_battery == false)
      Bluefruit.Scanner.start(0);  // 0 = Don't stop scanning after n seconds////////////////
    scantime = millis();
    scanafter = millis();
  }
}
void startAdv()
{
  //uint8_t batt_firm[2] = {0x64, 0x01};

  //uint8_t mac[6];

  // Advertising packet

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(Conf);

  Bluefruit.getAddr(mac);
  uint8_t flag[8] = {mac[5], mac[4], mac[3], mac[2], mac[1], mac[0], batt_firm[0], batt_firm[1]};



  Bluefruit.Advertising.addManufacturerData((const void*)flag, 8);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval((adv_update / 0.625), (adv_update / 0.625)); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds

}

void red_led_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
   //if (conf_opened)
  {

    red_led_distance_cmp = (data[0]);
    D_PRINT("RED = "); D_PRINTLN(red_led_distance_cmp);
    conf_update();   //update configration settings in memory
  }
}
void green_led_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  // if (conf_opened)
  {

    green_led_distance_cmp =(data[0]);
    D_PRINT("GREEN = "); D_PRINTLN(green_led_distance_cmp);
    conf_update();
  }

}
void blue_led_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
 //  if (conf_opened)
  {

    blue_led_distance_cmp = (data[0]);
    D_PRINT("BLUE = "); D_PRINTLN(blue_led_distance_cmp);
    conf_update();
  }
}

void buzzer_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data1, uint16_t len)
{
   // if (conf_opened)
  {

    buzzer_distance_cmp = (data1[0]);
    D_PRINT("BUzzer = "); D_PRINTLN(buzzer_distance_cmp);
    conf_update();
  }
}
void vibrator_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data2, uint16_t len)
{
  // if (conf_opened)
  {

    vibrator_distance_cmp =(data2[0]);
    D_PRINT("Motor = "); D_PRINTLN(vibrator_distance_cmp);
    conf_update();
  }
}

void max_records_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data3, uint16_t len)
{
  // if (conf_opened)
  {

    //  update max records
    //max_records_saved = 0;
    max_records_saved = ((data3[0]) |  (data3[1] << 8));
    D_PRINT("MAX RECORDS = "); D_PRINTLN(max_records_saved);
    conf_update();
  }
}


void scan_interval_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data4, uint16_t len)
{
  // if (conf_opened)
  {

    scan_interval_update = (uint8_t)(data4[0]);
    D_PRINT("scan_interval_update = "); D_PRINTLN(scan_interval_update);
    conf_update();
  }
}

void scan_gap_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data5, uint16_t len)
{
  // if (conf_opened)
  {

    scan_gap_update = (uint8_t)(data5[0]);
    D_PRINT("scan_gap_update = "); D_PRINTLN(scan_gap_update);
    conf_update();
  }
}

void log_thresh_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data6, uint16_t len)
{
   // if (conf_opened)
  {

    log_thresh_update = (uint8_t)(data6[0]);
    D_PRINT("log_thresh_update = "); D_PRINTLN(log_thresh_update);
    conf_update();  //updating configrations in file
  }
}
//

void adv_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data6, uint16_t len)
{
//  if (conf_opened)
  {
   adv_update = ((data6[0]) |  (data6[1] << 8));
    Bluefruit.Advertising.stop();
    startAdv();
    D_PRINT("adv_update = "); D_PRINTLN(adv_update);
    conf_update();
  }
}

void conf_pswd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data6, uint16_t len)
{

  String  mydata = "";
  mydata += data6[0] - 48;
  mydata += data6[1] - 48;
  mydata += data6[2] - 48;
  mydata += data6[3] - 48;
  mydata += data6[4] - 48;
  mydata += data6[5] - 48;
  mydata += data6[6] - 48;
  mydata += data6[7] - 48;

  D_PRINT("len "); D_PRINTLN( len );
  D_PRINT("after conversion "); D_PRINTLN( mydata );
  D_PRINT("after conversion pswd key "); D_PRINTLN(  PASSWORD_KEY);

  /*mydata = (mydata << 8) | (uint8_t)(data6[1]);
    mydata = (mydata << 8) | (uint8_t)(data6[2]);
    mydata = (mydata << 8) | (uint8_t)(data6[3]);*/
  if (mydata == PASSWORD_KEY || mydata == MASTER_PASSWORD)

  {

    D_PRINT("PASSWORD MATCHED ");
    D_PRINT("NOW YOU CAN UPDATE CONFIGRATIONS AND NEW PASSWORD ");
    conf_opened = true;
  }
}
void updt_pswd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data6, uint16_t len)
{

 // if (conf_opened)
  {
    D_PRINT("OLD PASSWORD MATCHED");
    D_PRINT("NOW YOU CAN UPDATE PASSWORD ");
    String  mydata = "";
    mydata += data6[0] - 48;
    mydata += data6[1] - 48;
    mydata += data6[2] - 48;
    mydata += data6[3] - 48;
    mydata += data6[4] - 48;
    mydata += data6[5] - 48;
    mydata += data6[6] - 48;
    mydata += data6[7] - 48;
    //strcpy(PASSWORD_KEY,mydata);
    PASSWORD_KEY = mydata;
    D_PRINT("PASSWORD UPDATED ="); D_PRINT(PASSWORD_KEY);
    password_update();
  }
}

void find_me_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data9, uint16_t len)
{
  uint16_t mydata;
  mydata = (uint8_t)(data9[0]);
  mydata = (mydata << 8) | (uint8_t)(data9[1]);
  if (mydata == 0x012A)
  {
    ledblink = true;
  }
}

void set_time_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data7, uint16_t len)
{
  //if (!set_time)
  {
    uint8_t timee[6];
    timee[0] = (uint8_t)(data7[0]);
    timee[1] = (uint8_t)(data7[1]);
    timee[2] = (uint8_t)(data7[2]);
    timee[3] = (uint8_t)(data7[3]);
    timee[4] = (uint8_t)(data7[4]);
    timee[5] = (uint8_t)(data7[5]);
    D_PRINTLN((uint32_t)timee[0]);
    D_PRINTLN((uint32_t)timee[1]);
    D_PRINTLN((uint32_t)timee[2]);
    D_PRINTLN((uint32_t)timee[3]);
    D_PRINTLN((uint32_t)timee[4]);
    D_PRINTLN((uint32_t)timee[5]);
    //nrf_cal_init();
    // nrf_cal_set_time( (uint32_t)timee[0], (uint32_t)timee[1], (uint32_t)timee[2], (uint32_t)timee[3], (uint32_t)timee[4],  (uint32_t)timee[5]);
    //nrf_cal_set_time(20, 12, 31, 23, 59, 01);
    rtc.setHour(timee[3], 0); //setting hour
    rtc.setMinute(timee[4]);  //setting minute
    rtc.setSecond(timee[5]);   //setting second


    rtc.setDay(timee[2]);     //setting day
    rtc.setMonth(timee[1]);    //setting month
    rtc.setYear(timee[0]);    //setting year

    rtc.getDate();      //getting date in local structure (local_date)
    rtc.getTime();      //getting time in local structure(local_time)

    //printing date in format YYYY/MM/DD
    D_PRINTLN(rtc.date.year + 2000); // year
    D_PRINTLN('/');
    D_PRINTLN(rtc.date.month);    // month
    D_PRINTLN('/');
    D_PRINTLN(rtc.date.day);      // day
    D_PRINTLN(' ');

    //printing time
    D_PRINTLN(rtc.time.hour);    //hour
    D_PRINTLN(':');
    D_PRINTLN(rtc.time.minute);  //minute
    D_PRINTLN(':');
    D_PRINTLN(rtc.time.second);  //second

    //saveTime();
    //set_time = true;      //ABCDEFGH
  }
}

///////////////////
void Command_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data6, uint16_t len)
{
  char buf[9];
  uint16_t record_num = 0;
  String myCount = "";
  uint16_t mydata;
  mydata = (uint8_t)(data6[0]);
  mydata = (mydata << 8) | (uint8_t)(data6[1]);
  D_PRINT("mydata = "); D_PRINTLN(mydata);
  if (mydata == 0x027A)
  {
    D_PRINTLN("mydata Matched");
    file.close();
    file.open(REC_SAVE, FILE_O_READ);
    if (file)
    {
      D_PRINTLN("file Opened");
      memset(buf, 0, sizeof(buf));
      file.read(buf, sizeof(buf));
      myCount += String(buf);
      record_num = (uint16_t)hex2Uint32(myCount.c_str());
      D_PRINT("record_num_1 =");D_PRINTLN(record_num);
      file.close();
      delay(100);
      
      Command.write16(record_num);
      command = true;
    }
    else
    {
        Command.write16(0x0000);
        file.close();
        if ( file.open(REC_SAVE, FILE_O_WRITE) )
          {
            String writeCount = "";
            memset(buf, 0, sizeof(buf));
            sprintf(buf, "%08X", 0);
            writeCount += buf;
            S_PRINT("myCount in record_num_fun in read write= "); S_PRINTLN(myCount);
            file.seek(0);
            file.write(writeCount.c_str(), writeCount.length());
            file.close();
            //S_PRINTLN("Does it Closed? ");
          }
      
    }
    
  }
  else if (mydata == 0x027B)
  {
    SerialFlash.eraseAll();
    D_PRINT("eraseAll  flash erased= ");
    record_num = 0;
    flash_read_address = 4096;
    flash_write_address = 0;
    //
    if ( file.open(ADDRESS_FILENAME, FILE_O_WRITE) )
    {
      String myAddress = "";
      memset(buf, 0, sizeof(buf));
      sprintf(buf, "%08X", flash_write_address);
      myAddress += buf;

      memset(buf, 0, sizeof(buf));
      sprintf(buf, "%08X", flash_read_address);
      myAddress += buf;

      D_PRINT("Open " FILENAMEE " file to write ... ");
      file.seek(0);
      file.write(myAddress.c_str(), myAddress.length());
      file.close();
    }
    //

    //
    if ( file.open(REC_SAVE, FILE_O_WRITE) )
    {
      char new_buf[9];
      String resetCount = "";
      memset(new_buf, 0, sizeof(new_buf));
      sprintf(new_buf, "%08X", 0);
      resetCount += new_buf;

      D_PRINT("Open " FILENAMEE " file to write ... ");
      file.seek(0);
      file.write(resetCount.c_str(), resetCount.length());
      file.close();
    }
    Command.write16(record_num);
  }

  else if (mydata == 0x027C)
  {

    read_btwn_adr = true;
    if ((read_btwn_adr) && (start_addr) && (end_addr))
    {
      STARTT_ADDRESS = STARTT_ADDRESS * 16;
      ENDD_ADDRESS = ENDD_ADDRESS * 16;
      record_diff = ((ENDD_ADDRESS - STARTT_ADDRESS) / 16) + 1;
      D_PRINT("record_diff = "); D_PRINTLN(record_diff);
      delay(100);
      Command.write16(record_diff);
    }
    else if ((read_btwn_adr) && (start_addr))
    {

      STARTT_ADDRESS = STARTT_ADDRESS * 16;
      /**if (flash_write_address >= STARTT_ADDRESS)
        {
        record_diff = ((flash_write_address - STARTT_ADDRESS) / 16);
        Command.write16(record_diff);
        }
        else
        {
        record_diff = 0;
        Command.write16(record_diff);
        }**/
      if (MEND_ADDRESS >= STARTT_ADDRESS)
      {
        record_diff = ((MEND_ADDRESS - STARTT_ADDRESS) / 16) + 1;
        Command.write16(record_diff);
      }
      else
      {
        record_diff = 0;
        Command.write16(record_diff);
      }
    }


  }

}

void start_address_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data6, uint16_t len)
{
  uint32_t mydata;
  mydata = (uint8_t)(data6[0]);
  mydata = (mydata << 8) | (uint8_t)(data6[1]);
  mydata = (mydata << 8) | (uint8_t)(data6[2]);
  mydata = (mydata << 8) | (uint8_t)(data6[3]);
  STARTT_ADDRESS = mydata;
  start_addr = true;
  D_PRINT("STARTT_ADDRESS = "); D_PRINTLN(STARTT_ADDRESS);
}


void end_address_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data6, uint16_t len)
{

  uint32_t mydata;
  mydata = (uint8_t)(data6[0]);
  mydata = (mydata << 8) | (uint8_t)(data6[1]);
  mydata = (mydata << 8) | (uint8_t)(data6[2]);
  mydata = (mydata << 8) | (uint8_t)(data6[3]);
  ENDD_ADDRESS = mydata;
  end_addr = true;
  D_PRINT("ENDD_ADDRESS = "); D_PRINTLN(ENDD_ADDRESS);
}

//////////////

int8_t hex2int8(const char *hex) {
  int8_t val = 0;
  while (*hex) {
    // get current character then increment
    uint8_t bytes = *hex++;
    // transform hex character to the 4bit equivalent number, using the ascii table indexes
    if (bytes >= '0' && bytes <= '9') bytes = bytes - '0';
    else if (bytes >= 'a' && bytes <= 'f') bytes = bytes - 'a' + 10;
    else if (bytes >= 'A' && bytes <= 'F') bytes = bytes - 'A' + 10;
    // shift 4 to make space for new digit, and add the 4 bits of the new digit
    val = (val << 4) | (bytes & 0xF);
  }
  return val;
}
uint8_t hex2Uint8(const char *hex) {
  uint8_t val = 0;
  while (*hex) {
    // get current character then increment
    uint8_t bytes = *hex++;
    // transform hex character to the 4bit equivalent number, using the ascii table indexes
    if (bytes >= '0' && bytes <= '9') bytes = bytes - '0';
    else if (bytes >= 'a' && bytes <= 'f') bytes = bytes - 'a' + 10;
    else if (bytes >= 'A' && bytes <= 'F') bytes = bytes - 'A' + 10;
    // shift 4 to make space for new digit, and add the 4 bits of the new digit
    val = (val << 4) | (bytes & 0xF);
  }
  return val;
}

uint16_t hex2Uint16(const char *hex) {
  uint16_t val = 0;
  while (*hex) {
    // get current character then increment
    uint8_t bytes = *hex++;
    // transform hex character to the 4bit equivalent number, using the ascii table indexes
    if (bytes >= '0' && bytes <= '9') bytes = bytes - '0';
    else if (bytes >= 'a' && bytes <= 'f') bytes = bytes - 'a' + 10;
    else if (bytes >= 'A' && bytes <= 'F') bytes = bytes - 'A' + 10;
    // shift 4 to make space for new digit, and add the 4 bits of the new digit
    val = (val << 4) | (bytes & 0xF);
  }
  return val;
}
uint32_t hex2Uint32(const char *hex) {
  uint32_t val = 0;
  while (*hex) {
    // get current character then increment
    uint8_t bytes = *hex++;
    // transform hex character to the 4bit equivalent number, using the ascii table indexes
    if (bytes >= '0' && bytes <= '9') bytes = bytes - '0';
    else if (bytes >= 'a' && bytes <= 'f') bytes = bytes - 'a' + 10;
    else if (bytes >= 'A' && bytes <= 'F') bytes = bytes - 'A' + 10;
    // shift 4 to make space for new digit, and add the 4 bits of the new digit
    val = (val << 4) | (bytes & 0xF);
  }
  S_PRINT("val IN fUNC = "); S_PRINTLN(val);
  return val;
}

void loop()
{
  NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
  if ((power_save))
  {
    //while(1)
    {

      if (hwPWM) {
        hwPWM = false;
        HwPWM0.stop();
        HwPWM1.stop();
      }
      Bluefruit.Scanner.stop();
      sd_app_evt_wait();
      vPortSuppressTicksAndSleep(25);
      waitForEvent();
      NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0

    }
  }
  BUTTONstate = digitalRead(BUTTON);
  //D_PRINT("BUTTONstate = "); D_PRINTLN(BUTTONstate);
  // D_PRINT("BUTTONstate TIME  = "); D_PRINTLN(millis());
  // D_PRINT("advbutton TIME  = "); D_PRINTLN(advbutton_time);
  if ((BUTTONstate == LOW) && (!button_pressed))
  {
    //D_PRINTLN("MY BUTTON ON");
    button_pressed = true;
    button_time = millis();
  }
  else if ((millis() - button_time > 5000))
  {
    //D_PRINTLN("MY BUTTON OFF");
    button_pressed = false;
  }

  if ((BUTTONstate == LOW) && (!power_off))
  {
    unsigned long y = millis() - advbutton_time;
    if (y > BUTTON_OFF_TIME)
    {
      digitalWrite(red_led, LOW);
      digitalWrite(red_led2, LOW);
      digitalWrite(vibrator, HIGH);
      //smart_delay(1000);
      //delay(1000); /// stuck wdt reset occur
      y = millis()+2000;
      while(millis()<y);
      digitalWrite(vibrator, LOW);
      digitalWrite(red_led, HIGH);
      digitalWrite(red_led2, HIGH);


      D_PRINTLN("POWERING OFF");
      advbutton_time = millis();
      power_off = true;
    }
  }
  else if ((BUTTONstate == LOW) && (power_off))
  {
    unsigned long y = millis() - advbutton_time;
    if (y > BUTTON_ON_TIME)
    {
      digitalWrite(vibrator, HIGH);
      smart_delay(1000);
      digitalWrite(vibrator, LOW);
      digitalWrite(green_led, LOW);
      digitalWrite(green_led2, LOW);
      HwPWM0.writePin(buzzer, maxValue / 12, false);
      smart_delay(1000);
      digitalWrite(green_led, HIGH);
      digitalWrite(green_led2, HIGH);
      HwPWM0.writePin(buzzer, 0, false);

      D_PRINTLN("POWERING ON");
      advbutton_time = millis();
      wakeup = true;
    }
  }
  else
  {
    advbutton_time = millis();
  }
  /*
    if (intrpt_value)
    {
      bma_sleep = millis();                  //BMA SETTINGS
      bma_intr_count++;
      intrpt_value = false;
      S_PRINTLN("ITS INTERRUPT");
    }
    if (millis() - bma_sleep > (millis_factor * TURNOFF_TIME))
    {
      D_PRINTLN("POWERING OFF FROM BMA");
      bma_sleep = millis();
      power_off = true;
      S_PRINTLN("BMA POWERED OFF");
      bma_intr_count = 0;
    }
  */
  if ((power_off))
  {
    Bluefruit.Scanner.stop();
    Bluefruit.Advertising.stop();
    scan = false;
    power_save = true;
    HwPWM0.stop();
    hwPWM = false;
    if ((bma_intr_count > INTR_COUNTER) && (power_off))
    {
      S_PRINTLN("POWERING ON FROM BMA");
      power_off = false;
      wakeup = true;
    }
  }
  if (wakeup)
  {
    power_off = false;
    scan = true;
    power_save = false;
    start_scan();
    startAdv();
    wakeup = false;
  }
  if (!power_off)
  {

    scan_events();
    if (!scan)
    {
      distance_conditions();
      if ((millis() - greenled_ptime > greenled_interval + 500) || (least_distance == 50.00))
      {
        //S_PRINTLN("GOING TO POWER SAVE");
        power_save = true;
      }
    }
    //get the current "time" (actually the number of milliseconds since the program started)
    // 5 sec interval code for cell voltage
    unsigned long long currenttime = millis();
    if ((currenttime - previoustime >= millis_factor * eventinterval)) //eventinterval for battery voltage update
    {
      S_PRINTLN("15m battery read");
      cellvoltage();
      if (low_battery == true) {
        for (int i = 0; i < 4; i++)
        {
          if (i % 2 == 0)
          {
            digitalWrite(red_led, LOW);
            digitalWrite(red_led2, LOW);
            digitalWrite(green_led2, LOW);
            digitalWrite(green_led, LOW);
          }
          else
          {
            digitalWrite(red_led, HIGH);
            digitalWrite(red_led2, HIGH);
            digitalWrite(green_led2, HIGH);
            digitalWrite(green_led, HIGH);
            delay(250);
          }
          delay(250);
        }
      }
      previoustime = currenttime;
    }
  }



  if (ledblink)
  {
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(green_led, LOW);
      digitalWrite(green_led2, LOW);
       HwPWM0.writePin(buzzer,0, false);

      smart_delay(500);
      digitalWrite(green_led, HIGH);
      digitalWrite(green_led2, HIGH);
     HwPWM0.writePin(buzzer, maxValue / 12, false);

      smart_delay(500);
      HwPWM0.writePin(buzzer,0, false);

    }
    ledblink = false;

  }

  if (disconnected == false && millis() - notifytime >= millis_factor * notifyinterval)
  {
    S_PRINTLN("dis con by pheripheral");
    Bluefruit.disconnect(my_conn_handle);
    //notifytime = millis();

  }

  if (Bluefruit.connected() && Set_data.notifyEnabled() && command)
  {
    uint8_t sig[16];
    char buf[9];
    String myCount = "";
    uint32_t record_num = 0;
    int rec_cnt=0;
    file.open(REC_SAVE, FILE_O_READ);
    if (file)
    {
      memset(buf, 0, sizeof(buf));
      file.read(buf, sizeof(buf));
      myCount += String(buf);
      S_PRINT("myCount in = "); S_PRINTLN(myCount);
      record_num = hex2Uint32(myCount.c_str());
      S_PRINT("record_num in HEX = "); S_PRINTLN(record_num, HEX);
      file.close();
    }
    else
    {
       
  if ( file.open(REC_SAVE, FILE_O_WRITE) )
    {
      String writeCount = "";
      memset(buf, 0, sizeof(buf));
      sprintf(buf, "%08X", 0);
      writeCount += buf;
      S_PRINT("myCount in record_num_fun in read write= "); S_PRINTLN(myCount);
      file.seek(0);
      file.write(writeCount.c_str(), writeCount.length());
      file.close();
      //S_PRINTLN("Does it Closed? ");
    }
      
    }

    S_PRINTLN("Connected");
    S_PRINT("flash_write_address = "); S_PRINTLN(flash_write_address);
    S_PRINT("record_num = "); S_PRINTLN(record_num);
    S_PRINT("record_num in HEX = "); S_PRINTLN(record_num, HEX);

    if (last_read_addr == 0)
    {
      if (flash_read_address != 4096)               //Flash Read Addr is 4096 if Not equal to that than dont change Flash read addr this could be cause that device is shut down during Sending Data
      {
        flash_read_address = flash_read_address;
        S_PRINT("flash_read_address 1= "); S_PRINTLN(flash_read_address);

      }
      else
      {
        flash_read_address = flash_read_address * flash_index;
        S_PRINT("flash_read_address 2= "); S_PRINTLN(flash_read_address);
      }
    }
    else
    {
      flash_read_address = last_read_addr;
      S_PRINT("flash_read_address 3= "); S_PRINTLN(flash_read_address);
    }
    HwPWM0.stop();
    HwPWM1.stop();
    digitalWrite(blue_led, HIGH);
    digitalWrite(blue_led2, HIGH);
    digitalWrite(red_led, HIGH);
    digitalWrite(red_led2, HIGH);
    digitalWrite(green_led2, HIGH);
    digitalWrite(green_led, HIGH);
    //flash_read_address = 0;
    en_flash();
    //smart_delay(1000);
    rec_cnt=0;
    for (uint32_t i = 0; i < record_num ; i++)
    {
      S_PRINT("reading = "); S_PRINTLN(i);
      NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
      SerialFlash.read(flash_read_address, sig, 16);
      
      delay(400);
      Set_data.notify( sig, 16 );
      rec_cnt++;
      rec_counter--;
      delay(400);
      S_PRINTBuffer(sig, 16);
      S_PRINTLN();
      if (flash_read_address == MEND_ADDRESS)
      {
        S_PRINTLN("MEND_ADDRESS");
        flash_read_address = 0;
      }
      else
      {
        flash_read_address = flash_read_address + 16;
        S_PRINT("flash_read_address = "); S_PRINTLN(flash_read_address);
      }
      if (disconnected || !Set_data.notifyEnabled())
      {
        S_PRINTLN("DISCONNECTED");
        //disconnected = false;
       // removeDevices(i);
        break;
      }
    }
 
  removeDevices(rec_cnt);
    notifytime = millis();
    if (!disconnected && Set_data.notifyEnabled())
    {
      //removeDevices(record_num);
      flash_read_address = flash_write_address;
      S_PRINT("flash_read_address aas"); S_PRINTLN(flash_read_address);
    }

    disconnected = false;
    S_PRINT("flash_write_address = "); S_PRINTLN(flash_write_address);
    S_PRINT("flash_read_address = "); S_PRINTLN(flash_read_address);
    // if (flash_read_address != flash_write_address)
    {
      last_read_addr = flash_read_address;
      if ( file.open(ADDRESS_FILENAME, FILE_O_WRITE) )
      {
        String myAddress = "";
        memset(buf, 0, sizeof(buf));
        sprintf(buf, "%08X", flash_write_address);
        myAddress += buf;

        memset(buf, 0, sizeof(buf));
        sprintf(buf, "%08X", flash_read_address);
        myAddress += buf;

        D_PRINT("Open " FILENAMEE " file to write ... ");
        file.seek(0);
        file.write(myAddress.c_str(), myAddress.length());
        file.close();
      }

    }
    //    else {
    //      if ( file.open(ADDRESS_FILENAME, FILE_O_WRITE) )
    //      {
    //        String myAddress = "";
    //        flash_write_address = 0;
    //        flash_read_address = 4096;
    //        memset(buf, 0, sizeof(buf));
    //        sprintf(buf, "%08X", flash_write_address);
    //        myAddress += buf;
    //
    //        memset(buf, 0, sizeof(buf));
    //        sprintf(buf, "%08X", flash_read_address);
    //        myAddress += buf;
    //
    //        D_PRINT("Open " ADDRESS_FILENAME " file to write ... ");
    //        file.seek(0);
    //        file.write(myAddress.c_str(), myAddress.length());
    //        file.close();
    //      }
    //     //// SerialFlash.eraseAll();/////////////////////////////////////////////////////////////////////////////////////////////////////////why?
    //
    //      rec_counter = 0;
    //      if ( file.open(REC_SAVE, FILE_O_WRITE) )
    //      {
    //        char new_buf[9];
    //        String resetCount = "";
    //        memset(new_buf, 0, sizeof(new_buf));
    //        sprintf(new_buf, "%08X", 0);
    //        resetCount += new_buf;
    //
    //        D_PRINT("Open " FILENAMEE " file to write ... ");
    //        file.seek(0);
    //        file.write(resetCount.c_str(), resetCount.length());
    //        file.close();
    //      }
    //      flash_read_address = 4096;
    //      flash_write_address = 0;
    //    }
    command = false;
    disable_flash();
  }

  else if (Bluefruit.connected() && Set_data.notifyEnabled() &&  (start_addr) && (end_addr) && (read_btwn_adr) )
  {
    uint8_t sig[16];
    //STARTT_ADDRESS = STARTT_ADDRESS ;
    D_PRINT("STARTT_ADDRESS = "); D_PRINTLN(STARTT_ADDRESS);
    //ENDD_ADDRESS = ENDD_ADDRESS ;
    D_PRINT("ACTIVEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE ");
    D_PRINT("ENDD_ADDRESS = "); D_PRINTLN(ENDD_ADDRESS);

    record_diff = ((ENDD_ADDRESS - STARTT_ADDRESS) / 16) + 1;
    D_PRINT("record_diff "); D_PRINT(record_diff );
    for (uint32_t i = 0; i < record_diff ; i++)
    {
      NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
      SerialFlash.read(STARTT_ADDRESS, sig, 16);
      Set_data.notify( sig, 16 );
      S_PRINTBuffer(sig, 16);
      S_PRINTLN();
      STARTT_ADDRESS = STARTT_ADDRESS + 16;
      delay(250);
      if (STARTT_ADDRESS == ENDD_ADDRESS + 16)
      {
        break;
      }
    }
    notifytime = millis();
    start_addr = false;
    end_addr = false;
    read_btwn_adr = false;
  }

  else if (Bluefruit.connected() && Set_data.notifyEnabled() &&  (start_addr) && (read_btwn_adr) )
  {
    uint8_t sig[16];

    if (MEND_ADDRESS >= STARTT_ADDRESS)
    {
      record_diff = ((MEND_ADDRESS - STARTT_ADDRESS) / 16) + 1;
    }
    //    else if (flash_write_address < STARTT_ADDRESS && memful)
    //    {
    //      record_diff = (((MEND_ADDRESS - STARTT_ADDRESS) / 16) + (flash_write_address / 16));                            /////////////////////////////////
    //    }
    //    else if (flash_write_address >= STARTT_ADDRESS && memful)
    //    {
    //      record_diff = ((flash_write_address - STARTT_ADDRESS) / 16) + ((MEND_ADDRESS - flash_write_address) / 16);     /////////////////////////////////
    //    }
    else
    {
      record_diff = 0;
    }
    D_PRINT("ACTIVEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE AGAIN ");
    D_PRINT("flash_write_address ");  D_PRINT(flash_write_address );
    D_PRINT("record_diff again "); D_PRINT(record_diff );
    for (uint32_t i = 0; i < record_diff ; i++)
    {
      NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
      SerialFlash.read(STARTT_ADDRESS, sig, 16);
      Set_data.notify( sig, 16 );
      STARTT_ADDRESS = STARTT_ADDRESS + 16;
      delay(250);

    }
    notifytime = millis();
    start_addr = false;
    read_btwn_adr = false;
  }
}
void scan_events()
{
  if ((millis() - scantime > ((millis_factor * scan_interval_update * 1000)  )) && (scan)) //test whether the period has elapsed
  {
    S_PRINTLN("Scan Stopped");
    D_PRINTLN("stop millis =");
    scan = false;
    D_PRINTLN(millis());
    D_PRINTLN();
    Bluefruit.Scanner.stop();
    Weighted_Mean(data);
    S_PRINTLN("HERE?");
    redled_ptime = millis();
    greenled_ptime = millis();
    blueled_ptime = millis();
    buzzer_ptime = millis();
    vibrator_ptime = millis();
    for (int i = 0; i < dev_count; i++)
    {
      memset(records[i].addr, 0, 6);
      records[i].distance = 0;
      memset(records[i].timestamp, 0, 6);
      records[i].battery = 0;
      records[i].firmware = 0;
      data[i].averageRSSI = 0;
      memset(&data[i], 0, sizeof(data_record_t));

      data[i]._count = 0;
      data[i].rssid[0] = -128;
      for (int j = 0; j < dev_count; j++)
      {
        data[i].rssid[j] = -128;
      }
    }
    dev_count = 0;
    scan = false;
    scanafter = millis();
  }
  ////scan start again

  if ((millis() - scanafter > (GAP_RESOLUTION * millis_factor * scan_gap_update * 1000)) && (!scan)  && disconnected)
  {
    {
      lleast_distance = 50.00;
      //      least_distance = 50;
      least_distance = 50.00;
      cellvoltage();
      if (low_battery == false) {
        Bluefruit.Scanner.start(0);
        Bluefruit.Scanner.resume();
      }
      scantime = millis();

      scan = true;
      power_save = false;
      S_PRINTLN("Scan Start----------------------------------------------------------");
      D_PRINTLN("start millis =");
      D_PRINTLN(millis());
    }
  }
}
void cellvoltage()
{
  static int16_t valueRead[1];
  // bool low_battery = false;
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;
  NRF_SAADC->CH[0].CONFIG = SAADC_CH_CONFIG_BURST_Enabled << SAADC_CH_CONFIG_BURST_Pos |
                            SAADC_CH_CONFIG_GAIN_Gain1_5 << SAADC_CH_CONFIG_GAIN_Pos |
                            SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos |
                            SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos |
                            SAADC_CH_CONFIG_RESN_Pulldown << SAADC_CH_CONFIG_RESN_Pos |
                            SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos |
                            SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos;
  NRF_SAADC->CH[0].PSELN =  SAADC_CH_PSELN_PSELN_NC;
  NRF_SAADC->CH[0].PSELP =  SAADC_CH_PSELP_PSELP_VDD;

  //set oversample
  NRF_SAADC->OVERSAMPLE = 4;
  //set result buffer
  NRF_SAADC->RESULT.MAXCNT = 1;
  NRF_SAADC->RESULT.PTR = (uint32_t)valueRead;

  //start ADC conversion
  NRF_SAADC->EVENTS_STARTED = 0;
  NRF_SAADC->TASKS_START = 1;
  while (NRF_SAADC->EVENTS_STARTED == 0);

  NRF_SAADC->EVENTS_END = 0;
  NRF_SAADC->TASKS_SAMPLE = 1;
  while (NRF_SAADC->EVENTS_END == 0);

  //disable ADC
  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);
  float voltage = valueRead[0] * 3.00;
  D_PRINTLN("voltage Raw--");
  D_PRINTLN(valueRead[0]);
  voltage = (voltage / 1024.00);
  D_PRINTLN("voltage after resolution -------------------");
  D_PRINTLN(voltage);
  //smart_delay(1000);
  //voltper = round((voltage / 3) * 100);

  voltper = map(valueRead[0], 850, 1023, 0, 100);
  if ( valueRead[0] < 850)
    voltper = 0;
  else
    voltper = map(valueRead[0], 850, 1023, 0, 100);

  if (voltper > 100)
    voltper = 100;

  D_PRINTLN("voltper -------------------");
  D_PRINTLN(voltper);
  if (voltage < 2.65)
  { D_PRINTLN("LOW BATTERYYYYYYYYYYYYYYY-------------------");
    low_battery = true;

    //pinMode(BUTTON,  INPUT_PULLUP_SENSE);    // this pin (WAKE_LOW_PIN) is pulled up and wakes up the feather when externally connected to ground.
    // NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter << POWER_SYSTEMOFF_SYSTEMOFF_Pos;
  }
  else
  {
    low_battery = false;
  }

  batt_firm[0] = (batt_firm[0] & 0x80) | (voltper); ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

void timestamp(uint8_t timeStamp[6])
{
  //  uint8_t timeStamp[6];
  String cal_string;
  char time_date[13];
  rtc.getDate();      //getting date in local structure (local_date)
  rtc.getTime();      //getting time in local structure(local_time)
  //  strftime(cal_string, 13, "%Y%m%d%H%M%S", );
  //  //strftime(cal_string, 13, "%Y%m%d%H%M%S", (calibrated ? nrf_cal_get_time_calibrated() : nrf_cal_get_time()));
  //  memcpy(time_date, nrf_cal_get_time_string(true), 13);
  //
  //  cal_string = String(time_date);
  //
  //  byte user1[1], user2[2], user3[2], user4[2], user5[2], user6[2];
  //
  //  String user1str = cal_string.substring(0, 2);
  //  String user2str = cal_string.substring(2, 4);
  //  String user3str = cal_string.substring(4, 6);
  //  String user4str = cal_string.substring(6, 8);
  //  String user5str = cal_string.substring(8, 10);
  //  String user6str = cal_string.substring(10, 12);
  //
  //
  //
  //  timeStamp[0] = atoi( user1str.c_str());
  //  timeStamp[1] = atoi( user2str.c_str());
  //  timeStamp[2] = atoi( user3str.c_str());
  //  timeStamp[3] = atoi( user4str.c_str());
  //  timeStamp[4] = atoi( user5str.c_str());
  //  timeStamp[5] = atoi( user6str.c_str());

  timeStamp[0] = rtc.date.year;
  timeStamp[1] = rtc.date.month;
  timeStamp[2] = rtc.date.day;
  timeStamp[3] = rtc.time.hour;
  timeStamp[4] = rtc.time.minute;
  timeStamp[5] = rtc.time.second;


}


void cleardata()
{
  uint8_t flag[8] = {mac[5], mac[4], mac[3], mac[2], mac[1], mac[0], batt_firm[0], batt_firm[1]};

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);


  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(Conf);



  Bluefruit.Advertising.addManufacturerData((const void*)flag, 8);

  Bluefruit.ScanResponse.addName();

}



/*------------------------------------------------------------------*/
/* Peripheral
  ------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle)
{
  notifytime = millis();
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  D_PRINT("[Prph] Connected to ");
  D_PRINTLN(peer_name);
  disconnected = false;
  my_conn_handle = conn_handle;
  notifytime = millis();
}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
  (void) conn_handle;
  (void) reason;
  disconnected = true;
  conf_opened = false;
  D_PRINTLN();
  D_PRINTLN("[Prph] Disconnected");
}

void prph_bleuart_rx_callback(uint16_t conn_handle)
{


}

/*------------------------------------------------------------------*/
/* Central
  ------------------------------------------------------------------*/
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  S_PRINT("MY SCAN CALLBACK = ");
  S_PRINTLN(report->rssi);
  NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
  // D_PRINTBuffer(report->peer_addr.addr, 6, ':');
  if (insertRecord(report) == 1)                 /* Returns 1 if the list was updated */
  {
    //   printRecordList();                            /* The list was updated, print the new values */
    //D_PRINTLN("");
  }
  // D_PRINTLN();
  if (scan)
    Bluefruit.Scanner.resume();

}
/************** Weighted Mean/Feedback Filtering ******************************/
void Weighted_Mean(data_record_t data[ARRAY_SIZE])
{
  uint32_t new_coun = 0;
  int8_t filter_final_rssi = 0;
  uint8_t coun = 0;

  for (int j = 0; j < (data[j]._count - 1); j++)
  {
    for (int i = 1; i < data[j]._count; i++)
    {
      int8_t aPrevious_RSSI = data[j].rssid[i - 1];
      int8_t aCurrent_RSSI = data[j].rssid[i];
      int8_t filter_rssi = 0;
      int8_t alpha = 0.6;
      filter_rssi = (int8_t)(((1 - ALPHA) * ((float)aCurrent_RSSI)) + (ALPHA * ((float)aPrevious_RSSI)));
      data[j].rssid[i - 1] = filter_rssi;
      coun++;
    }
    data[j]._count--;
  }
  ///////////////////////average

  for (int j = 0; j < dev_count; j++)
  {
    uint8_t c = 0;
    data[j].averageRSSI = 0;
    for (int i = 0; i < data[j]._count; i++)
    {

      data[j].averageRSSI += data[j].rssid[i];

    }

  }
  uint8_t timeStamp[6];
  timestamp(timeStamp);
  D_PRINTLN("time stamp");
  for (int i = 0; i < 6; i++)
  {
    D_PRINT(timeStamp[i]);
  }
  D_PRINTLN("");

  en_flash();
  smart_delay(1000);
  for (int j = 0, x = 0; j < dev_count; j++)
  {
    S_PRINTF("The rssi values of device %i = ", data[j]._count);

    S_PRINTF("The total rssi of device %i = ", j);

    S_PRINTLN( data[j].averageRSSI );

    data[j].averageRSSI = data[j].averageRSSI / data[j]._count;
    if (data[j].averageRSSI > -90)
    {
      D_PRINTF("The average rssi of device %i = ", j);

      S_PRINTLN( data[j].averageRSSI );
      float p = MEASURED_TX_POWER - (data[j].averageRSSI);
      float pwr_factor = p / ENVIROMENTAL_FACTOR_MULTIPLIED_10;

      data[j].dist = pow(10, pwr_factor);
    }
    else
    {
      data[j].dist = 50.00;
      D_PRINTF("The average rssi of device %i -90 = ", j);

      S_PRINTLN( data[j].averageRSSI );

    }

    //   data[j].batterylevel=report->adv;// battery level update
    // firmware update
    D_PRINT("Average distance (m) ");
    D_PRINTLN( data[j].dist);
    D_PRINT("batterylevel ");
    D_PRINTLN( data[j].batterylevel);
    D_PRINT("Fimware version ");
    D_PRINTLN( data[j].firm_version);

    if (data[j].dist <= log_thresh_update)
    {
      memcpy(records[x].addr, data[j].address, 6);
      memcpy(records[x].timestamp, timeStamp, 6);
      records[x].battery = data[j].batterylevel;
      records[x].firmware = data[j].firm_version;

      if (data[j].dist > 0.00 && data[j].dist <= 1.00)
      {
        records[x].distance = 1;
      }
      else if (data[j].dist > 1.00 && data[j].dist <= 2.00)
      {
        records[x].distance = 2;
      }
      else if (data[j].dist > 2.00 && data[j].dist <= 3.00)
      {
        records[x].distance = 3;
      }
      else if (data[j].dist > 3.00 && data[j].dist <= 4.00)
      {
        records[x].distance = 4;
      }
      //records[x].distance = round (data[j].dist);
      // D_PRINT("address ");
      //      D_PRINT(records[x].timestamp);

      x++;
      rec_counter++;
      new_coun++;                        //Specifically created for Count Devices and is independent
    }
    D_PRINTLN("OK");
    ////////////////////////
    uint8_t char_array[16];
    uint8_t z;
    String data;
    char buff[3];
    String data2;
    char buff2[3];
    for (int i = 0; i < 6; i++)
    {
      //memset(buff, 0, sizeof(buff));
      //sprintf(buff, "%02X", records[x - 1].addr[i] );
      char_array[i] = records[x - 1].addr[i];
      //data += buff;
    }
    //memset(buff, 0, sizeof(buff));
    //sprintf(buff, "%02X", records[x - 1].distance );
    char_array[6] = records[x - 1].distance;
    //data += buff;
    for (int i = 0; i < 6; i++)
    {
      //memset(buff, 0, sizeof(buff));
      //sprintf(buff, "%02X", records[x - 1].timestamp[i] );
      char_array[7 + i] = records[x - 1].timestamp[i];
      //data += buff;
    }
    //memset(buff, 0, sizeof(buff));
    //sprintf(buff, "%02X", records[x - 1].battery );
    char_array[13] = records[x - 1].battery ;
    //data += buff;
    //memset(buff, 0, sizeof(buff));
    //sprintf(buff, "%02X", records[x - 1].firmware );
    char_array[14] = records[x - 1].firmware ;
    //data += buff;

    // D_PRINT(data);
    //data += "00";
    char_array[15] = 0;

    for (int i = 0  ; i < 16; i++)
    {
      S_PRINT(char_array[i], HEX);
    }
     aes.encrypt(char_array);
    // aes.decrypt(char_array);
   
    S_PRINTLN();
    if (flash_write_address % 4096 == 0 && flash_write_address != MEND_ADDRESS)
    {
      SerialFlash.eraseBlock( flash_write_address );
      if (flash_index > 0)
      {
        S_PRINT("flash_write_address ON mem full = "); S_PRINTLN(flash_write_address);
        //removeDevices(256);
      }
    }
   
    SerialFlash.write(flash_write_address, char_array, 16);

    flash_write_address = flash_write_address + 16;

    if (flash_write_address == MEND_ADDRESS)
    {
      flash_index++;
      memful = true;
      flash_write_address = 0;
    }
    else if ((memful) && (flash_write_address % 4096 == 0))
    {
      flash_index++;
    }
    if ( file.open(ADDRESS_FILENAME, FILE_O_WRITE) )
    {
      char buf[9];
      String myAddress = "";
      memset(buf, 0, sizeof(buf));
      sprintf(buf, "%08X", flash_write_address);
      myAddress += buf;

      memset(buf, 0, sizeof(buf));
      sprintf(buf, "%08X", flash_read_address);
      myAddress += buf;

      D_PRINT("Open " ADDRESS_FILENAME " file to write ... ");
      file.seek(0);
      file.write(myAddress.c_str(), myAddress.length());
      file.close();
    }//*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    lleast_distance = bubbleSort();

    //lleast_distance += Optimized_Distance;
    least_distance = lleast_distance;//round(lleast_distance);
  }
  disable_flash();
  S_PRINT("dev_count = "); S_PRINTLN(dev_count);
  S_PRINT("new_coun = "); S_PRINTLN(new_coun);
  record_num_fun(new_coun);

  S_PRINTLN("lest distance before");
  S_PRINTLN(lleast_distance);
  S_PRINTLN("lest distance after");
  S_PRINTLN(least_distance);

  if (least_distance == 50.00)
  {
    no_distancecount++;
  }
  else
    no_distancecount = 0;

  if (no_distancecount > 5)
  {
    saveTime();
    while (1); //wait for wdt reset

  }

  S_PRINTLN("SO IT DOESNT EXIT FROM WEIGHTED MEAN?");
}
void distance_conditions()
{

  signed char red_t = 0;
  signed char blue_t = 0;
  signed char green_t = 0;
  signed char buz_t = 0;
  signed char vib_t = 0;

  bool red_on = 0;
  bool blue_on = 0;
  bool green_on = 0;
  bool buz_on = 0;
  bool vib_on = 0;

  int index = 10;
  //  signed char initial = -100;
  float initial = -100.00;
  //  signed char max_val_thresh[3];
  float max_val_thresh[3];
  max_val_thresh[0] = least_distance - (float)red_led_distance_cmp;
  max_val_thresh[1] = least_distance - (float)blue_led_distance_cmp;
  max_val_thresh[2] = least_distance - (float)green_led_distance_cmp;
  buz_t = least_distance - (float)buzzer_distance_cmp;
  vib_t = least_distance - (float)vibrator_distance_cmp;


  if ((max_val_thresh[0] <= 0))
  {
    red_on = true;
  }
  if ((max_val_thresh[1] <= 0))
  {
    blue_on = true;
  }
  if ((max_val_thresh[2] <= 0))
  {
    green_on = true;
  }
  if ((buz_t <= 0))
  {
    buz_on = true;
  }
  if ((vib_t <= 0))
  {
    vib_on = true;
  }
  if (buz_t >= vib_t && buz_t <= 0)
  {
    vib_on = false;
  }
  else if (buz_t < vib_t && vib_t <= 0)
  {
    buz_on = false;
  }
  for (int i = 0; i < 3; i++)
  {
    if (max_val_thresh[i]  >= initial && max_val_thresh[i] <= 0)
    {
      initial = max_val_thresh[i];
      index = i;
    }
  }
  if (index == 0)
  {
    red_var = true;
    green_var = false;
    blue_var = false;
  }
  else if (index == 1)
  {
    red_var = false;
    green_var = false;
    blue_var = true;
  }
  else if (index == 2)
  {
    red_var = false;
    green_var = true;
    blue_var = false;
  }
  else
  {
    red_var = false;
    green_var = false;
    blue_var = false;
  }




  //RED LED
  if ((red_var)  && (millis() - redled_ptime <= redled_interval) && (millis() % total_time < led_time))
  {

    {

      {
        red_LEDState = HIGH;
        red_LEDState2 = HIGH;// change the state of LED
        red_rememberTime = millis(); // remember Current millis() time
        vibrator_state = LOW;
        digitalWrite(vibrator, LOW);
        buzzer_state = 0;
        vib_state = 0;
        digitalWrite(red_led, LOW );
        digitalWrite(red_led2, LOW );

      }
    }
  }
  else if ((millis() - redled_ptime > redled_interval))
  {
    digitalWrite(red_led, HIGH);
    digitalWrite(red_led2, HIGH);
  }

  //GREEN LED

  if ((green_var)  && (millis() - greenled_ptime <= greenled_interval) && (millis() % total_time < led_time))
  {
    {

      {
        green_LEDState = HIGH;
        green_LEDState2 = HIGH;// change the state of LED
        green_rememberTime = millis(); // remember Current millis() time
        vibrator_state = LOW;
        digitalWrite(vibrator, LOW);
        buzzer_state = 0;
        vib_state = 0;
        HwPWM0.writePin(buzzer,  buzzer_state, false);
      }
    }


    digitalWrite(green_led, !green_LEDState);

    digitalWrite(green_led2, !green_LEDState2);


  }

  else
  {

    digitalWrite(green_led, HIGH);
    digitalWrite(green_led2, HIGH);
  }


  //BLUE LED

  if ((blue_var) && (millis() - blueled_ptime <= blueled_interval) && (millis() % total_time < led_time))
  {
    if ( blue_LEDState == LOW &&  blue_LEDState2 == LOW )
    {
      //if ( (millis() - blue_rememberTime) >= onDuration)
      {
        blue_LEDState = HIGH;
        blue_LEDState2 = HIGH;
        blue_rememberTime = millis();
        vibrator_state = LOW;
        digitalWrite(vibrator, LOW);
        buzzer_state = 0;
        vib_state = 0;
        HwPWM0.writePin(buzzer,  buzzer_state, false);
      }
    }

    digitalWrite(blue_led, !blue_LEDState);
    digitalWrite(blue_led2, !blue_LEDState2);
  }
  else
  {
    digitalWrite(blue_led, HIGH);
    digitalWrite(blue_led2, HIGH);

  }
  if ((buz_on) && (millis() - buzzer_ptime <= buzzer_interval) && (!button_pressed) && (millis() % total_time > led_time))
  {
    if (!hwPWM)
    {
      HwPWM0.addPin( buzzer );
      HwPWM0.begin();
      hwPWM = true;
    }
    if ( buzzer_state == 0  )
    {
      //if ( (millis() - rememberTimeBuzzer) >= offDuration)
      {
        buzzer_state = maxValue / 12;
        rememberTimeBuzzer = millis();
      }
      digitalWrite(blue_led, HIGH);
      digitalWrite(blue_led2, HIGH);
      digitalWrite(green_led, HIGH);
      digitalWrite(green_led2, HIGH);
      digitalWrite(red_led, HIGH);
      digitalWrite(red_led2, HIGH);
      HwPWM0.writePin(buzzer,  buzzer_state, false);
    }
    //    else
    //    {
    //      //if ( (millis() - rememberTimeBuzzer) >= onDuration)
    //      {
    //        buzzer_state = 0;
    //       rememberTimeBuzzer = millis();
    //      }
    //    }

    //analogWrite(buzzer,buzzer_state);
  }
  else
  {
    //analogWrite(buzzer,0);
    buzzer_state = 0;
    HwPWM0.writePin(buzzer, 0, false);
    //hwPWM = false;
    HwPWM0.stop();

  }


  if ((vib_on) && (millis() - vibrator_ptime <= vibrator_interval) && (millis() % total_time > led_time))
  {

    if (!hwPWM)
    {
      HwPWM1.addPin( vibrator );
      HwPWM1.begin();
      hwPWM = true;
    }
    if ( vib_state == 0  )
    {
      vib_state = maxValue / 2;
      rememberTimeBuzzer = millis();
       HwPWM1.writePin(vibrator,  vib_state, false);
    }
    //if ( vibrator_state == LOW  )
    {
      // if ( (millis() - vibrator_rememberTime) >= offDuration)
      {
        //vibrator_state = HIGH;
        //vibrator_state = HIGH;
        vibrator_rememberTime = millis();
        digitalWrite(blue_led, HIGH);
        digitalWrite(blue_led2, HIGH);
        digitalWrite(green_led, HIGH);
        digitalWrite(green_led2, HIGH);
        digitalWrite(red_led, HIGH);
        digitalWrite(red_led2, HIGH);

      }
    }
    //digitalWrite(vibrator, vibrator_state);
  }
  else
  {
    vib_state = 0;
    HwPWM1.writePin(vibrator, 0, false);
    // hwPWM = false;
    HwPWM1.stop();
  }
  //S_PRINT("rec_counter = "); S_PRINTLN(rec_counter);
  //S_PRINT("max_records_saved = ");S_PRINTLN(max_records_saved);
  if ( rec_counter >= max_records_saved)
  {
    //S_PRINTLN("FULL");
    batt_firm[0] =  batt_firm[0] | 0x80;
    cleardata();
    max_records_reached = true;
  }
  else if ((max_records_reached) && (rec_counter <= max_records_saved))
  {
    //S_PRINTLN("LESS Than MAX");
    batt_firm[0] =  batt_firm[0] & 0x7F;
    cleardata();
    max_records_reached = false;
  }
}
/* Prints the current record list to the Serial Monitor */
void printRecordList(void)
{
  for (uint8_t i = 0; i < ARRAY_SIZE; i++)
  {
    D_PRINTF("[%i] ", i);
    D_PRINTBuffer(records[i].addr, 6, ':');
    D_PRINTF(" %u (%u ms)\n", records[i].distance, records[i].timestamp);


  }
}

/* This function performs a simple bubble sort on the records array */
/* It's slow, but relatively easy to understand */
/* Sorts based on RSSI values, where the strongest signal appears highest in the list */
float bubbleSort()
{
  float distance = 0;
  if (dev_count > 0)
  {
    D_PRINTLN("0");
    distance = data[0].dist;
    for (int i = 1; i < dev_count; i++)
    {
      if (data[i].dist < distance)
      {
        distance = data[i].dist;
      }
    }
  }
  else
  {
    D_PRINTLN("50.00");
    distance = 50.00;
  }
  return (distance);
}

int insertRecord(ble_gap_evt_adv_report_t* report)//node_record_t *record)
{
  uint8_t i;
  uint8_t match = 0;
  uint8_t a = 0;
  for (i = 0; i < dev_count; i++)
  {
    if (memcmp(data[i].address, report->peer_addr.addr, 6) == 0)
    {
      match = 1;
    }
    if (match)
    {
      data[i].rssid[data[i]._count] = report->rssi;
      data[i].batterylevel = (uint8_t)report->data.p_data[29] & 0x7F;
      data[i].firm_version = (uint8_t)report->data.p_data[30];
      data[i]._count++;
      // for (int x = 0; x < data[i]._count; x++)
      // {
      //   D_PRINTLN(data[i].rssid[x]);
      //  }
      return 1;

    }
  }
  uint8_t d_mac[6];
  memset(d_mac, 0, 6);
  /* 3. Check for zero'ed records */
  /*    Insert if a zero record is found, then sort */
  // for (i = 0; i < ARRAY_SIZE; i++)
  {
    //  if (memcmp(data[i].address, d_mac, 6) == 0)
    {
      memcpy(data[dev_count].address, report->peer_addr.addr, 6);
      data[dev_count].rssid[0] = report->rssi;
      data[dev_count].batterylevel = (uint8_t)(report->data.p_data[29] & 0x7F);
      data[dev_count].firm_version = (uint8_t)report->data.p_data[30];
      data[dev_count]._count++;

      dev_count++;
      // memcpy(records[i].addr, report->peer_addr.addr, 6);
      return 1;
    }
  }

  return 0;
}

void conf_update()
{
  if ( file.open(FILENAMEE, FILE_O_WRITE) )
  {
    file.seek(0);
    D_PRINT("Open " FILENAMEE " file to write ... ");
    D_PRINTLN("OK");
    ////////////////////////
    String data1;
    char buff[3];

    memset(buff, 0, sizeof(buff));
    sprintf(buff, "%02X", (int8_t)red_led_distance_cmp );
    data1 += buff;

    memset(buff, 0, sizeof(buff));
    sprintf(buff, "%02X", (int8_t)green_led_distance_cmp );
    data1 += buff;

    memset(buff, 0, sizeof(buff));
    sprintf(buff, "%02X",  (int8_t)blue_led_distance_cmp );
    data1 += buff;

    memset(buff, 0, sizeof(buff));
    sprintf(buff, "%02X", (int8_t) buzzer_distance_cmp );
    data1 += buff;

    memset(buff, 0, sizeof(buff));
    sprintf(buff, "%02X", (int8_t)vibrator_distance_cmp );
    data1 += buff;

    memset(buff, 0, sizeof(buff));
    sprintf(buff, "%02X", scan_interval_update );
    data1 += buff;

    char new_buff[5];
    memset(new_buff, 0, sizeof(new_buff));
    sprintf(new_buff, "%04X",  max_records_saved  );
    data1 += new_buff;


    memset(buff, 0, sizeof(buff));
    sprintf(buff, "%02X",  scan_gap_update  );
    data1 += buff;

    memset(buff, 0, sizeof(buff));
    sprintf(buff, "%02X", log_thresh_update  );
    data1 += buff;

    char neww_buff[5];
    memset(neww_buff, 0, sizeof(neww_buff));
    sprintf(neww_buff, "%04X", adv_update  );
    data1 += neww_buff;

    D_PRINTLN(data1);
    file.write(data1.c_str(), data1.length());
    file.close();
  }

}


void password_update()
{
  if ( file.open(FILENAMEEIS, FILE_O_WRITE) )
  {
    file.seek(0);
    D_PRINT("Open " FILENAMEEIS " file to write ... ");
    D_PRINTLN("OK");

    file.write(PASSWORD_KEY.c_str(), PASSWORD_KEY.length());
    file.close();
  }

}
void smart_delay(uint64_t delay_time)
{
  /*uint64_t start_time = millis();
    while (millis() - start_time < delay_time);*/
  delay(delay_time);
}

void update_conf_var()
{
  char data_buf[24];
  file.open(FILENAMEE, FILE_O_READ);
  if (file)
  {
    String val = "";
    D_PRINTLN(file.size());
    file.read(data_buf, sizeof(data_buf));
    D_PRINTLN(data_buf);
    file.close();

    D_PRINTLN("Printing Variables = ");
    val = String(data_buf).substring(0, 2);
    red_led_distance_cmp = hex2int8(val.c_str());
    D_PRINTLN(red_led_distance_cmp);

    val = String(data_buf).substring(2, 4);
    green_led_distance_cmp = hex2int8(val.c_str());
    D_PRINTLN(green_led_distance_cmp);

    val = String(data_buf).substring(4, 6);
    blue_led_distance_cmp = hex2int8(val.c_str());
    D_PRINTLN(blue_led_distance_cmp);

    val = String(data_buf).substring(6, 8);
    buzzer_distance_cmp =hex2int8(val.c_str());
    D_PRINTLN(buzzer_distance_cmp);

    val = String(data_buf).substring(8, 10);
    vibrator_distance_cmp =hex2int8(val.c_str());
    D_PRINTLN(vibrator_distance_cmp);

    val = String(data_buf).substring(10, 12);
    scan_interval_update = hex2Uint8(val.c_str());
    D_PRINTLN(scan_interval_update);

    val = String(data_buf).substring(12, 16);
    max_records_saved = hex2Uint16(val.c_str());
    D_PRINTLN(max_records_saved);

    val = String(data_buf).substring(16, 18);
    scan_gap_update = hex2Uint8(val.c_str());
    D_PRINTLN(scan_gap_update);

    val = String(data_buf).substring(18, 20);
    log_thresh_update = hex2Uint8(val.c_str());
    D_PRINTLN(log_thresh_update);

    val = String(data_buf).substring(20, 24);
    adv_update = hex2Uint16(val.c_str());
    D_PRINTLN("adv_update from memory =");
    D_PRINTLN(adv_update);

    D_PRINTLN("CONFIGURATION UPDATED ");

  }

}


void password_updt_var()
{
  char data_buf[8];
  file.open(FILENAMEEIS, FILE_O_READ);
  if (file)
  {
    D_PRINTLN(file.size());
    file.read(data_buf, sizeof(data_buf));
    PASSWORD_KEY = "";
    D_PRINT("PASSWORD Buffer = ");
    for (int i = 0; i < 8; i++)
    {
      D_PRINT(data_buf[i]);
      PASSWORD_KEY += String(data_buf[i]);
    }
    file.close();

    D_PRINTLN("PASSWORD UPDATED ");
    D_PRINTLN("UPDATED PASSWORD in file ="); D_PRINTLN(PASSWORD_KEY);

  }

}

void chip_id()
{
  unsigned char buf[3];

  unsigned char cmpbuf[3];
  int c = 0;
  memset(buf, 0, sizeof(buf));
  memset(cmpbuf, 0, sizeof(cmpbuf));
  cmpbuf[0] = 1;
  cmpbuf[1] = 96;
  cmpbuf[2] = 23;
  D_PRINTLN("Read Chip Identification:");
  SerialFlash.readID(buf);
  for (int i = 0; i < 3; i++)
  {
    if (buf[i] == cmpbuf[i])
    {
      c++;
    }
  }

  if (c == 3)
  {

    batt_firm[1] = batt_firm[1] | 0x40;
  }


}
void beginAccel()
{
  uint8_t chipID = 0;
  initBMA();                    //Initialization of BMA253
  readBMAChipID(&chipID);             //Reading Chip ID of BMA253
  DBMA_PRINTF("The BMA253 ChipID is %02X\n", chipID);
  if (chipID != 0xFA)
  {
    DBMA_PRINTLN("Error Chip ID");
  }
  if (chipID == 0xFA)
  {
    batt_firm[1] = batt_firm[1] | 0x80;
  }
  //bma_deep_suspended();
  setBMA_Motioninterrupts();
  en_low_pwr();                      //Enabling Low Power Mode..
}


/*void digital_callback(void)
  {
  intrpt_value = true;
  }
*/
void readd_address()
{

  char dataa_buf[4];
  file.open(ADDRESS_FILENAME, FILE_O_READ);
  if (file)                                                   //flash address update
  { String val = "";

    file.read(dataa_buf, sizeof(dataa_buf));
    val = String(dataa_buf).substring(0, 2);
    flash_write_address = hex2Uint8(val.c_str());

    val = String(dataa_buf).substring(2, 4);
    flash_read_address = hex2Uint8(val.c_str());

    file.close();
  }
}
void record_num_fun(uint32_t new_records)
{
  char buf[9];
  String myCount = "";
  uint32_t totalDevices = 0;
  file.open(REC_SAVE, FILE_O_READ);
  if (file)
  {
    memset(buf, 0, sizeof(buf));
    file.read(buf, sizeof(buf));
    myCount += String(buf);
    S_PRINT("myCount in record_num_fun in read = "); S_PRINTLN(myCount);
    totalDevices = hex2Uint32(myCount.c_str());
    totalDevices += new_records;
    S_PRINT("totalDevices = "); S_PRINTLN(totalDevices);
    myCount = "";
    file.close();
    if ( file.open(REC_SAVE, FILE_O_WRITE) )
    {
      myCount = "";
      memset(buf, 0, sizeof(buf));
      sprintf(buf, "%08X", totalDevices);
      myCount += buf;
      S_PRINT("myCount in record_num_fun in read write= "); S_PRINTLN(myCount);
      file.seek(0);
      file.write(myCount.c_str(), myCount.length());
      file.close();
      //S_PRINTLN("Does it Closed? ");
    }
  }
  else if ( file.open(REC_SAVE, FILE_O_WRITE) )
  {
    memset(buf, 0, sizeof(buf));
    sprintf(buf, "%08X", new_records);
    myCount += buf;
    S_PRINT("myCount in record_num_fun in write = "); S_PRINTLN(myCount);
    D_PRINT("Open " REC_SAVE " file to write ... ");
    file.seek(0);
    file.write(myCount.c_str(), myCount.length());
    file.close();
  }
  //S_PRINTLN("Does it Exit? ");
}
/************************************************************
  # Update address in case if Device is shut down
 ************************************************************/
void update_address(uint32_t *write_addr, uint32_t *read_addr)
{
  String myAddress = "";
  String my_write_address = "";
  String my_read_address = "";
  file.open(ADDRESS_FILENAME, FILE_O_READ);
  if (file)
  {
    char buf[16];
    memset(buf, 0, sizeof(buf));
    file.read(buf, sizeof(buf));
    myAddress += buf;
    file.close();
    my_write_address = myAddress.substring(0, 8);
    my_read_address = myAddress.substring(8, 16);
    S_PRINT("my_write_addressddr = "); S_PRINTLN(my_write_address);
    S_PRINT("my_read_address = "); S_PRINTLN(my_read_address);
    *write_addr = hex2Uint32(my_write_address.c_str());
    *read_addr = hex2Uint32(my_read_address.c_str());
    S_PRINT("write_addr = "); S_PRINTLN(*write_addr);
    S_PRINT("read_addr = "); S_PRINTLN(*read_addr);
  }
  else
    SerialFlash.eraseAll();
}
void removeDevices(uint32_t dev_to_remove)
{
  file.open(REC_SAVE, FILE_O_READ);
  if (file)
  {
    String myCount = "";
    char buf[9];
    memset(buf, 0, sizeof(buf));
    file.read(buf, sizeof(buf));
    file.close();
    myCount += buf;
    signed long devicesCount = (signed long)hex2Uint32(myCount.c_str());
    S_PRINT("devicesCount after removed = "); S_PRINTLN(devicesCount);
    devicesCount -= dev_to_remove;
    if (devicesCount > 400000)
      devicesCount = 0;
    if (file.open(REC_SAVE, FILE_O_WRITE))
    {
      memset(buf, 0, sizeof(buf));
      sprintf(buf, "%08X", devicesCount);
      file.seek(0);
      file.write(buf, sizeof(buf));
      file.close();
    }
  }
}
void read_update_time()
{
  file.open(RTC_TIME_FILE, FILE_O_READ);
  if (file)
  {
    String myTime = "";
    char buf[13];
    memset(buf, 0, sizeof(buf));
    file.read(buf, sizeof(buf));
    file.close();
    myTime += buf;
    S_PRINT("myTime = "); S_PRINTLN(myTime);
    uint8_t _year = hex2Uint8(myTime.substring(0, 2).c_str());
    uint8_t _month = hex2Uint8(myTime.substring(2, 4).c_str());
    uint8_t _day = hex2Uint8(myTime.substring(4, 6).c_str());
    uint8_t _hour = hex2Uint8(myTime.substring(6, 8).c_str());
    uint8_t _minute = hex2Uint8(myTime.substring(8, 10).c_str());
    uint8_t _second = hex2Uint8(myTime.substring(10, 12).c_str());

    S_PRINT("_year = "); S_PRINTLN(_year);
    S_PRINT("_month = "); S_PRINTLN(_month);
    S_PRINT("_day = "); S_PRINTLN(_day);
    S_PRINT("_hour = "); S_PRINTLN(_hour);
    S_PRINT("_minute = "); S_PRINTLN(_minute);
    S_PRINT("_second = "); S_PRINTLN(_second);

    rtc.setHour(_hour, 0); //setting hour
    rtc.setMinute(_minute);  //setting minute
    rtc.setSecond(_second);   //setting second

    rtc.setDay(_day);     //setting day
    rtc.setMonth(_month);    //setting month
    rtc.setYear(_year);    //setting year
  }
}
void saveTime()
{
  uint8_t timeStamp[6];
  rtc.getDate();      //getting date in local structure (local_date)
  rtc.getTime();      //getting time in local structure(local_time)

  //printing date in format YYYY/MM/DD
  timeStamp[0] = rtc.date.year; // year

  timeStamp[1] = rtc.date.month;    // month

  timeStamp[2] = rtc.date.day;      // day

  timeStamp[3] =  rtc.time.hour;    //hour

  timeStamp[4] = rtc.time.minute;  //minute

  timeStamp[5] = (rtc.time.second + 15);  //second
  String myTime = "";
  char buf[13];
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "%02X%02X%02X%02X%02X%02X", timeStamp[0], timeStamp[1], timeStamp[2], timeStamp[3], timeStamp[4], timeStamp[5]);
  myTime += buf;
  //S_PRINT("myTime saved = "); S_PRINTLN(myTime);
  NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
  if (file.open(RTC_TIME_FILE, FILE_O_WRITE))
  {
    file.seek(0);
    file.write(buf, sizeof(buf));
    file.close();
  }

}
void nrfx_wdt_0_irq_handler(void)
{
  //saveTime();
  S_PRINTLN("IRQ");
  //NRF_WDT->EVENTS_TIMEOUT = 0;


}
void en_flash()
{
  //    digitalWrite(FLASH_POWER_ONE, HIGH);
  //    digitalWrite(FLASH_POWER_TWO, HIGH);

  //pinMode(slaveSelectPin, OUTPUT);
}
void disable_flash()
{
  //    digitalWrite(FLASH_POWER_ONE, LOW);
  //    digitalWrite(FLASH_POWER_TWO, LOW);
  // pinMode(slaveSelectPin, INPUT);
  //  digitalWrite(slaveSelectPin, HIGH);
}


void getkey()
{
 Bluefruit.getAddr(mac);
byte a, b,c,d,e,f,g,h,i,j,k,l;

  
  a = mac[5] >> 4;
  b = mac[5] & 0xF;
  
  c = mac[4] >> 4;
  d = mac[4] & 0xF;
  
  e = mac[3] >> 4;
  f = mac[3] & 0xF;
  
  g = mac[2] >> 4;
  h = mac[2] & 0xF;
  
  i = mac[1] >> 4;
  j = mac[1] & 0xF;
  
  k = mac[0] >> 4;
  l = mac[0] & 0xF;
 
  byte key[] = { 
               
 (c |(a << 4)),(01 |(a << 4)),(02 |(c << 4)), (d |(b << 4)),(03 |(b << 4)),(04 |(d << 4)),
 ( g|(e << 4)),( 05|(e << 4)),( 06|(g << 4)),  ( h|(f << 4)),( 07|(f << 4)),( 8|(h << 4)),   //generate key
 ( k|(i << 4)),( 9|(i << 4)),( l|(j << 4)),( 10|(j << 4))

  }; 
}
