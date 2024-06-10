// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <NMEA2000_esp32.h>

#include "Wire.h"
#include "sender/n2k_senders.h"
#include "sensesp/system/serial_number.h"
#include "sensesp/transforms/typecast.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/data/wind_data.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"
#include "sensesp_nmea0183/wiring.h"
#include "ssd1306_display.h"

using namespace sensesp;

reactesp::ReactESP app;

constexpr int kWindBitRate = 4800;
constexpr int kWindRxPin = 19;
// set the Tx pin to -1 if you don't want to use it
constexpr int kWindTxPin = 18;

// CAN bus pins for SH-ESP32
constexpr gpio_num_t kCANRxPin = GPIO_NUM_34;
constexpr gpio_num_t kCANTxPin = GPIO_NUM_32;

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging();

  esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);

  Serial.begin(115200);

  Serial1.begin(4800, SERIAL_8N1, 19, 18);

  // Scan I2C bus for devices

  char error = 0;
  char address;
  char num_devices = 0;

  Wire.setPins(16, 17);
  Wire.begin();

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      num_devices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("wind-n2k")
                    ->enable_ota("thisisfine")
                    ->get_app();

  Serial1.begin(kWindBitRate, SERIAL_8N1, kWindRxPin, kWindTxPin);

  NMEA0183* nmea = new NMEA0183(&Serial1);

  ApparentWindData* apparent_wind_data = new ApparentWindData();

  WIMWVSentenceParser* wind_sentence_parser = new WIMWVSentenceParser(
      nmea, &apparent_wind_data->speed, &apparent_wind_data->angle);

  apparent_wind_data->angle.connect_to(new SKOutputFloat(
      "environment.wind.angleApparent", "/SK Path/Apparent Wind Angle"));
  apparent_wind_data->speed.connect_to(new SKOutputFloat(
      "environment.wind.speedApparent", "/SK Path/Apparent Wind Speed"));

  // Send wind instrument configuration messages at startup
  ReactESP::app->onDelay(10000, [nmea]() {
    String msg;
    char checksum;
    char buf[200];

    msg = "$PATC,IIMWV,CFG,10";
    checksum = CalculateChecksum(msg.c_str());
    // sprintf(buf, "%s*%02X\r\n", msg.c_str(), checksum);
    sprintf(buf, "%s\r\n", msg.c_str());
    ESP_LOGD("NMEA0183", "Sending %s", buf);
    nmea->output_raw(buf);

    msg = "$PATC,IIMWV,TXP,100";
    checksum = CalculateChecksum(msg.c_str());
    // sprintf(buf, "%s*%02X\r\n", msg.c_str(), checksum);
    sprintf(buf, "%s\r\n", msg.c_str());
    ESP_LOGD("NMEA0183", "Sending %s", buf);
    nmea->output_raw(buf);
  });

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  tNMEA2000* nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20240601",  // Manufacturer's Model serial code (max 32 chars)
      105,         // Manufacturer's product code
      "Wind-N2K",  // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      130,                     // Device function: Weather Instruments
      85,                      // Device class: Sensor Communication Interface
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    72  // Default N2k node address
  );
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, [nmea2000]() { nmea2000->ParseMessages(); });

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 wind data sender

  N2kWindDataSender* wind_data_sender = new N2kWindDataSender(
      "/Wind/NMEA2000", tN2kWindReference::N2kWind_Apparent, nmea2000, true);

  apparent_wind_data->speed.connect_to(
      &(wind_data_sender->wind_speed_consumer_));

  apparent_wind_data->angle.connect_to(
      &(wind_data_sender->wind_angle_consumer_));

  /////////////////////////////////////////////////////////////////////
  // Initialize the OLED display

  InfoDisplay* display = new InfoDisplay(&Wire);
  apparent_wind_data->speed.connect_to(
      &(display->apparent_wind_speed_consumer));
  apparent_wind_data->angle.connect_to(
      &(display->apparent_wind_angle_consumer));

  /////////////////////////////////////////////////////////////////////
  // TODO: Initialize the ICM-20948 IMU
}

void loop() { app.tick(); }
