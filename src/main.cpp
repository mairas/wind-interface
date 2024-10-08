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
#include "autonnic_a5120_parser.h"
#include "autonnic_config.h"
#include "elapsedMillis.h"
#include "sender/n2k_senders.h"
#include "sensesp/system/serial_number.h"
#include "sensesp/system/stream_producer.h"
#include "sensesp/transforms/filter.h"
#include "sensesp/transforms/typecast.h"
#include "sensesp/transforms/zip.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"
#include "sensesp/ui/ui_controls.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/data/wind_data.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"
#include "sensesp_nmea0183/wiring.h"
#include "ssd1306_display.h"

using namespace sensesp;
using namespace sensesp::nmea0183;
using namespace wind_interface;

constexpr int kWindBitRate = 4800;
constexpr int kWindRxPin = 19;
// set the Tx pin to -1 if you don't want to use it
constexpr int kWindTxPin = 18;

// CAN bus pins for SH-ESP32
constexpr gpio_num_t kCANRxPin = GPIO_NUM_34;
constexpr gpio_num_t kCANTxPin = GPIO_NUM_32;

ObservableValue<int> n2k_rx_counter = 0;
ObservableValue<int> n2k_tx_counter = 0;

elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging();

  // esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);

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
                    ->set_hostname("wind")
                    ->enable_ota("thisisfine")
                    ->get_app();

  Serial1.begin(kWindBitRate, SERIAL_8N1, kWindRxPin, kWindTxPin);

  NMEA0183IOTask* nmea0183_io_task = new NMEA0183IOTask(&Serial1);

  ApparentWindData* apparent_wind_data = new ApparentWindData();

  ConnectApparentWind(&(nmea0183_io_task->parser_), apparent_wind_data);

  // Connect the response parser
  AutonnicPATCWIMWVParser* autonnic_response_parser =
      new AutonnicPATCWIMWVParser(&(nmea0183_io_task->parser_));

  ReferenceAngleConfig* reference_angle_config = new ReferenceAngleConfig(
      nmea0183_io_task, 0, autonnic_response_parser, "/Wind/Reference Angle");

  ConfigItem(reference_angle_config)
      ->set_title("Reference Angle")
      ->set_description(
          "Reference angle offset for wind data (in degrees). "
          "Enter the angle readout when the wind vane is pointing "
          "straight ahead.")
      ->set_sort_order(300);

  WindDirectionDampingConfig* wind_direction_damping_config =
      new WindDirectionDampingConfig(nmea0183_io_task, 50.0,
                                     autonnic_response_parser,
                                     "/Wind/Direction Damping");

  ConfigItem(wind_direction_damping_config)
      ->set_title("Wind Direction Damping")
      ->set_description(
          "Wind direction damping factor (0-100.0). Default is "
          "50.0.")
      ->set_sort_order(400);

  WindSpeedDampingConfig* wind_speed_damping_config =
      new WindSpeedDampingConfig(nmea0183_io_task, 50.0,
                                 autonnic_response_parser,
                                 "/Wind/Speed Damping");

  ConfigItem(wind_speed_damping_config)
      ->set_title("Wind Speed Damping")
      ->set_description("Wind speed damping factor (0-100.0). Default is 50.0.")
      ->set_sort_order(500);

  WindOutputRepetitionRateConfig* wind_output_repetition_rate_config =
      new WindOutputRepetitionRateConfig(nmea0183_io_task, 500,
                                         autonnic_response_parser,
                                         "/Wind/Message Repetition Rate");

  ConfigItem(wind_output_repetition_rate_config)
      ->set_title("Message Repetition Rate")
      ->set_description(
          "Wind message repetition rate in milliseconds. Default "
          "is 500.")
      ->set_sort_order(200);

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
  nmea2000->SetMsgHandler([](const tN2kMsg& msg) {
    n2k_rx_counter = n2k_rx_counter.get() + 1;
    n2k_time_since_rx = 0;
  });
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  event_loop()->onRepeat(1, [nmea2000]() { nmea2000->ParseMessages(); });

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 wind data sender

  N2kWindDataSender* wind_data_sender = new N2kWindDataSender(
      "/Wind/NMEA2000", tN2kWindReference::N2kWind_Apparent, nmea2000, true);

  apparent_wind_data->speed.connect_to(&(wind_data_sender->wind_speed_));

  apparent_wind_data->angle.connect_to(&(wind_data_sender->wind_angle_));

  // wind_data_sender emits whenever it sends a message; count the messages
  wind_data_sender->connect_to(new LambdaConsumer<std::pair<double, double>>(
      [](std::pair<double, double> wind_data) {
        n2k_tx_counter = n2k_tx_counter.get() + 1;
        n2k_time_since_tx = 0;
      }));

  /////////////////////////////////////////////////////////////////////
  // Initialize the Signal K wind data sender

  auto apparent_wind_speed_sk_output = new SKOutputFloat(
      "/SK Path/Apparent Wind Speed", "environment.wind.speedApparent",
      new SKMetadata("Apparent Wind Speed", "m/s"));

  auto apparent_wind_angle_sk_output = new SKOutputFloat(
      "/SK Path/Apparent Wind Angle", "environment.wind.angleApparent",
      new SKMetadata("Apparent Wind Angle", "rad"));

  apparent_wind_data->speed.connect_to(apparent_wind_speed_sk_output);
  apparent_wind_data->angle.connect_to(apparent_wind_angle_sk_output);

  /////////////////////////////////////////////////////////////////////
  // Configuration elements

  CheckboxConfig* enable_n2k_watchdog_config = new CheckboxConfig(
      false, "Enable NMEA 2000 Watchdog", "/NMEA2000/Enable Watchdog");

  ConfigItem(enable_n2k_watchdog_config)
      ->set_title("Enable NMEA 2000 Watchdog")
      ->set_description(
          "Enable the NMEA 2000 watchdog. If enabled, the device will reboot "
          "after two minutes if no NMEA 2000 messages are received. This "
          "setting requires a device restart to take effect.")
      ->set_sort_order(100);

  if (enable_n2k_watchdog_config->get_value()) {
    event_loop()->onRepeat(1000, [nmea2000]() {
      if (n2k_time_since_rx > 120000) {
        ESP_LOGE("NMEA2000", "No messages received in 2 minutes. Restarting.");
        // All hope is lost; it doesn't matter if we delay for a bit to ensure
        // the log message is sent.
        delay(10);
        ESP.restart();
      }
    });
  }

  auto n2k_rx_ui_output = new StatusPageItem<int>("NMEA 2000 Received Messages",
                                                  0, "NMEA 2000", 300);

  n2k_rx_counter.connect_to(n2k_rx_ui_output);

  auto n2k_tx_ui_output = new StatusPageItem<int>(
      "NMEA 2000 Transmitted Messages", 0, "NMEA 2000", 310);

  n2k_tx_counter.connect_to(n2k_tx_ui_output);

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

void loop() { event_loop()->tick(); }
