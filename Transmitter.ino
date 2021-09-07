/*
 * This sketch reads and sends data to Ball Aerospace's COSMOS.
 * Stolen from: William Osman
 *
 * Modified by Trent Buckley 07/09/2021
 *
 * There is an issue when trying to send too much data at the same time. I'm not sure which end it's on.
 * It's in the tens of bytes.
 *
 */
#include <SPI.h>  //one of my computers won't compile without this SPI here
#include "RF24.h"
#include <EEPROM.h>

#define LED_PIN 9

#define PAYLOAD_BUF 64

#define DEFAULT_SAMPLE_RATE 30

#define CALIBRATION_EEPROM_ADDRESS  0

//NRF24 Radio
#define CE_PIN  8
#define CS_PIN  10

//Battery level
#define BATTERY_PIN A6
#define DIVIDER_RATIO ((5100.0 + 2200) / 2200.0)
#define ADC_REF_VOLT    5.0
#define ADC_RESOLUTION  1024.0

//Analog Accelerometer Pins
#define ACCEL_X_PIN A0
#define ACCEL_Y_PIN A1
#define ACCEL_Z_PIN A2

#define ZERO_G_X  336
#define ZERO_G_Y  337
#define ZERO_G_Z  352
#define ONE_G_X   420
#define ONE_G_Y   412
#define ONE_G_Z   420
#define SCALE_G ((ONE_G_Z - ZERO_G) / 1)

//Command ID List
#define INCOMING_ID_SETTING 0x00
#define INCOMING_ID_EXECUTE 0x01

//COSMOS Command list
#define CMD_NONE  0x00
#define CMD_TEST  0x01
#define CMD_SEND  0x02
#define CMD_STOP  0x03
#define CMD_ACCELEROMETER_NEUTRAL 0x04
#define CMD_ACCELEROMETER_CALIBRATE 0x05

struct car_tel_t
{
  uint8_t length;
  uint8_t id;
  uint8_t sample_rate; //samples per second
  uint16_t sample_time;
  float x_acceleration;
  float y_acceleration;
  float z_acceleration;
  float battery_voltage = 0.0;
  uint8_t test;
};

struct car_cmd_setting_t
{
  uint8_t id;
  byte sample_rate;
  uint16_t sample_time; //time to sample
};

struct car_cmd_execute_t
{
  uint8_t id;
  byte command;
};

struct cartesian_t {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
};

struct calibration_t {
  cartesian_t accelerometer_offset;
  cartesian_t accelerometer_scale;
};

calibration_t calibration;

cartesian_t max_value;

// global packet data
car_tel_t carData;
car_cmd_setting_t gsIncoming_setting;
car_cmd_execute_t gsIncoming_execute;

byte address[][6] = {"1Node", "2Node"};
RF24 radio(CE_PIN, CS_PIN); //(CE_pin, CS_pin)

cartesian_t acceleration;
cartesian_t nuetral_acceleration;
cartesian_t velocity;

bool calibrate = false;
bool begin_calibration = false;
bool set_calibration = false;

uint16_t sample_delay;

bool send_data = true;




// the setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(9600);  // initialize serial:
  pinMode(LED_PIN, OUTPUT);

  Serial.println("radio boot");
  //Serial.print("SPI Speed: "), Serial.println(RF24_SPI_SPEED);

  radio_config();
  radio.powerUp();

  carData.id = 1;
  carData.sample_rate = DEFAULT_SAMPLE_RATE;
  carData.sample_time = 0;
  sample_delay = 1000 / carData.sample_rate;

  carData.test = 1;

  load_calibration();
  printCalibration();
  
  max_value.x = 999999;
  max_value.y = 999999;
  max_value.z = 999999;

}

// the loop function runs over and over again forever
void loop()
{
  handle_acceleration();
  handle_battery();
  sendTlm();
  readCmds();
}

//************** Accelerometer *****************************************
void handle_acceleration() {
  uint16_t x_raw = analogRead(ACCEL_X_PIN);
  uint16_t y_raw = analogRead(ACCEL_Y_PIN);
  uint16_t z_raw = analogRead(ACCEL_Z_PIN);

  if (!calibrate && !set_calibration)
  {
    carData.x_acceleration = convert2Gs(x_raw, calibration.accelerometer_offset.x, calibration.accelerometer_scale.x);
    carData.y_acceleration = convert2Gs(y_raw, calibration.accelerometer_offset.y, calibration.accelerometer_scale.y);
    carData.z_acceleration = convert2Gs(z_raw, calibration.accelerometer_offset.z, calibration.accelerometer_scale.z);
  }
  else if (calibrate)
  {
    accelerometer_calibration(false);
    //Serial.println("calibrate");
  }
  else if (set_calibration)
  {
    accelerometer_calibration(true);
    set_calibration = false;
    Serial.println("set cal");
  }

  static uint32_t last_sample_time = millis();
  uint32_t this_sample_time = millis();
  uint32_t sample_time = this_sample_time - last_sample_time;
  last_sample_time = this_sample_time;

  //carData.x_velocity += carData.x_acceleration * sample_time / 1000.0;
  //carData.y_velocity += carData.y_acceleration * sample_time / 1000.0;
  //carData.z_velocity += carData.z_acceleration * sample_time / 1000.0;
}

void accelerometer_calibration(bool set_cal)
{
  /*
   * calibration procedure consists of rotating the sensor.
   * gravity is used as the standard acceleration.
   * each axis is oriented inline with gravity while recording min/max sensor values.
   * flip each axis to record acceleration in the opposite direction as well.
   */
  static cartesian_t max_acceleration;
  static cartesian_t min_acceleration = max_value;

  const float smoothing = 0.995;
  static cartesian_t smooth_accel;

  if (begin_calibration)
  {
    smooth_accel.x = analogRead(ACCEL_X_PIN);
    smooth_accel.y = analogRead(ACCEL_Y_PIN);
    smooth_accel.z = analogRead(ACCEL_Z_PIN);
    min_acceleration = max_value;

    begin_calibration = false;
  }

  smooth_accel.x = (smooth_accel.x * smoothing) + (analogRead(ACCEL_X_PIN) * (1.0 - smoothing));
  smooth_accel.y = (smooth_accel.y * smoothing) + (analogRead(ACCEL_Y_PIN) * (1.0 - smoothing));
  smooth_accel.z = (smooth_accel.z * smoothing) + (analogRead(ACCEL_Z_PIN) * (1.0 - smoothing));

  carData.x_acceleration = convert2Gs(smooth_accel.x, calibration.accelerometer_offset.x, calibration.accelerometer_scale.x);
  carData.y_acceleration = convert2Gs(smooth_accel.y, calibration.accelerometer_offset.y, calibration.accelerometer_scale.y);
  carData.z_acceleration = convert2Gs(smooth_accel.z, calibration.accelerometer_offset.z, calibration.accelerometer_scale.z);

  //log the min and max values
  max_acceleration.x = max(max_acceleration.x, smooth_accel.x);
  min_acceleration.x = min(min_acceleration.x, smooth_accel.x);
  max_acceleration.y = max(max_acceleration.y, smooth_accel.y);
  min_acceleration.y = min(min_acceleration.y, smooth_accel.y);
  max_acceleration.z = max(max_acceleration.z, smooth_accel.z);
  min_acceleration.z = min(min_acceleration.z, smooth_accel.z);

  if (set_cal) {
    Serial.println("set calibration");
    //accelerometer scale is 1g to -1g, so 2g delta
    //scale = (max - min) / 2g
    //offset = (max - min) / 2 + min

    calibration.accelerometer_scale.x = (max_acceleration.x - min_acceleration.x) / 2.0;
    calibration.accelerometer_scale.y = (max_acceleration.y - min_acceleration.y) / 2.0;
    calibration.accelerometer_scale.z = (max_acceleration.z - min_acceleration.z) / 2.0;
    calibration.accelerometer_offset.x = (max_acceleration.x - min_acceleration.x) / 2.0 + min_acceleration.x;
    calibration.accelerometer_offset.y = (max_acceleration.y - min_acceleration.y) / 2.0 + min_acceleration.y;
    calibration.accelerometer_offset.z = (max_acceleration.z - min_acceleration.z) / 2.0 + min_acceleration.z;
/*
    Serial.print("x scale: "), Serial.println(calibration.accelerometer_scale.x);
    Serial.print("x offset: "), Serial.println(calibration.accelerometer_offset.x);

    Serial.print("y scale: "), Serial.println(calibration.accelerometer_scale.y);
    Serial.print("y offset: "), Serial.println(calibration.accelerometer_offset.y);

    Serial.print("z scale: "), Serial.println(calibration.accelerometer_scale.z);
    Serial.print("z offset: "), Serial.println(calibration.accelerometer_offset.z);
*/
    printCalibration();

    cartesian_t blank;
    max_acceleration = blank;
    min_acceleration = blank;
    smooth_accel = blank;

    store_calibration();
  }
}

void accelerometer_nuetral()
{
  cartesian_t neutral_acceleration;
  const int samples = 50;

  //nuetral_acceleration.x = 0;
  //nuetral_acceleration.y = 0;
  //nuetral_acceleration.z = 0;

  for (int i = 0; i < samples; i++)
  {
    neutral_acceleration.x += analogRead(ACCEL_X_PIN);
    neutral_acceleration.y += analogRead(ACCEL_Y_PIN);
    neutral_acceleration.z += analogRead(ACCEL_Z_PIN);
    delay(2);
  }
  neutral_acceleration.x /= 100;
  neutral_acceleration.y /= 100;
  neutral_acceleration.z /= 100;
}

float convert2Gs(uint16_t analog_raw, uint16_t offset, uint16_t scale) {
  return ((float)analog_raw - (float)offset) / (float)scale;
}

//************** Read Battery *****************************************

void handle_battery()
{
  float raw_battery = analogRead(BATTERY_PIN);
  float battery_voltage = (ADC_REF_VOLT / ADC_RESOLUTION * raw_battery) * DIVIDER_RATIO;
  carData.battery_voltage = battery_voltage;//(float)analogRead(BATTERY_PIN) * DIVIDER_RATIO;
}

//************** EEPROM Calibration Storage *****************************************
void store_calibration()
{
  EEPROM.put(CALIBRATION_EEPROM_ADDRESS, calibration);
}

void load_calibration()
{
  EEPROM.get(CALIBRATION_EEPROM_ADDRESS, calibration);
}

void printCalibration()
{
  Serial.print("x scale: "), Serial.println(calibration.accelerometer_scale.x);
  Serial.print("x offset: "), Serial.println(calibration.accelerometer_offset.x);

  Serial.print("x scale: "), Serial.println(calibration.accelerometer_scale.x);
  Serial.print("x offset: "), Serial.println(calibration.accelerometer_offset.x);

  Serial.print("x scale: "), Serial.println(calibration.accelerometer_scale.x);
  Serial.print("x offset: "), Serial.println(calibration.accelerometer_offset.x);
}

//************** Telemetry Transceive *****************************************
void readCmds()
{
  if (radio.available())
  {
    Serial.println("radio available");
    uint8_t buf[PAYLOAD_BUF];
    uint8_t len = radio.getDynamicPayloadSize();

    //load payload into buffer
    readTlm(buf, len);

    byte incoming_ID = buf[0]; //load the first ID byte

    switch (incoming_ID)
    {
      case INCOMING_ID_SETTING:
        gsIncoming_setting = *((car_cmd_setting_t*)buf);
        carData.sample_rate = gsIncoming_setting.sample_rate;
        sample_delay = 1000 / gsIncoming_setting.sample_rate;
        break;

      case INCOMING_ID_EXECUTE:
        gsIncoming_execute = *((car_cmd_execute_t*)buf);
        executeCmd(gsIncoming_execute.command);
        break;

      default:
        //invalid packet ID
        break;
    }
  }
}

void load2Mem(uint8_t* buf, uint8_t* data, uint8_t size)
{
  for (int i = 1; i <= size; i++)
  {
    *data = *buf;
    data++;
    buf++;
  }

}

void executeCmd(byte command) {
  switch (command) {
    case CMD_NONE:
      //blank command
      digitalWrite(LED_PIN, HIGH);
      break;
    case CMD_TEST:
      //test command
      digitalWrite(LED_PIN, LOW);
      break;
    case CMD_SEND:
      //start sending data
      send_data = true;
      break;
    case CMD_STOP:
      //stop sending data
      send_data = false;
      break;
    case CMD_ACCELEROMETER_NEUTRAL:
      //records neutral position
      break;
    case CMD_ACCELEROMETER_CALIBRATE:
      if (calibrate)
      {
        //write calibration values to eeprom
        set_calibration = true;
        calibrate = false;
      }
      else {
        calibrate = true;
        begin_calibration = true;
      }
      break;
    default:
      //not recognized
      break;

  };
}

bool readTlm(void* data, uint8_t size) //return 0 for timeout
{
  if ( radio.available() ) {
    radio.read(data, size);
  }
}

void sendTlm()
{
  static uint32_t sendTime_last = 0;
  uint32_t time_current = millis();
  uint32_t dtime = time_current - sendTime_last;

  if (send_data && (dtime >= sample_delay))
  {
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    radio.stopListening();
    sendTime_last = time_current; //update timer
    uint32_t timer = millis();  //how long send takes?

    carData.length = sizeof(carData);
    //Serial.println(carData.length);
    writeTlm(&carData, carData.length);

    carData.test = millis() - timer;

    radio.startListening();
  }
}

void writeTlm(const void* pkt, byte size)
{
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  radio.write(pkt, size);
}

//************** NRF24 RADIO *****************************************
void radio_config()
{
  //SPISettings(1000000, MSBFIRST, SPI_MODE0);
  //SPI.setClockDivider(SPI_CLOCK_DIV64);
  if (!radio.begin()) Serial.println("radio fail");
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicPayloads();
  //radio.setPayloadSize(sizeof(myData));
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  radio.setDataRate(RF24_1MBPS);

  //radio.setChannel(53);
  //radio.setRetries(2, 2);
  //radio.setAutoAck(1);

  radio.startListening();
  radio.printDetails();
}
