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
#include "sensesp/ui/ui_controls.h"
#include "sensesp/ui/ui_output.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/data/wind_data.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"
#include "sensesp_nmea0183/wiring.h"
#include "ssd1306_display.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

constexpr int kWindBitRate = 4800;
constexpr int kWindRxPin = 19;
// set the Tx pin to -1 if you don't want to use it
constexpr int kWindTxPin = 18;

// CAN bus pins for SH-ESP32
constexpr gpio_num_t kCANRxPin = GPIO_NUM_34;
constexpr gpio_num_t kCANTxPin = GPIO_NUM_32;

int n2k_rx_counter = 0;
int n2k_tx_counter = 0;

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

  StreamLineProducer* wind_line_producer = new StreamLineProducer(&Serial1);

  Filter<String>* sentence_filter = new Filter<String>([](const String& line) {
    return line.startsWith("!") || line.startsWith("$");
  });

  wind_line_producer->connect_to(sentence_filter);

  NMEA0183* nmea = new NMEA0183();
  sentence_filter->connect_to(nmea);

  ApparentWindData* apparent_wind_data = new ApparentWindData();

  ConnectApparentWind(nmea, apparent_wind_data);

  // Connect the response parser
  AutonnicPATCWIMWVParser* autonnic_response_parser =
      new AutonnicPATCWIMWVParser(nmea);

  ReferenceAngleConfig* reference_angle_config = new ReferenceAngleConfig(
      0, &Serial1, autonnic_response_parser, "/Wind/Reference Angle");
  reference_angle_config->set_description(
      "Reference angle offset for wind data (in degrees). Enter the angle "
      "readout when the wind vane is pointing straight ahead.");
  reference_angle_config->set_sort_order(300);

  WindDirectionDampingConfig* wind_direction_damping_config =
      new WindDirectionDampingConfig(50.0, &Serial1, autonnic_response_parser,
                                     "/Wind/Direction Damping");
  wind_direction_damping_config->set_description(
      "Wind direction damping factor (0-100.0). Default is 50.0.");
  wind_direction_damping_config->set_sort_order(400);

  WindSpeedDampingConfig* wind_speed_damping_config =
      new WindSpeedDampingConfig(50.0, &Serial1, autonnic_response_parser,
                                 "/Wind/Speed Damping");
  wind_speed_damping_config->set_description(
      "Wind speed damping factor (0-100.0). Default is 50.0.");
  wind_speed_damping_config->set_sort_order(500);

  WindOutputRepetitionRateConfig* wind_output_repetition_rate_config =
      new WindOutputRepetitionRateConfig(500, &Serial1, autonnic_response_parser,
                                         "/Wind/Message Repetition Rate");
  wind_output_repetition_rate_config->set_description(
      "Wind message repetition rate in milliseconds. Default is 500.");
  wind_output_repetition_rate_config->set_sort_order(200);

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
    n2k_rx_counter++;
    n2k_time_since_rx = 0;
  });
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  SensESPBaseApp::get_event_loop()->onRepeat(1, [nmea2000]() { nmea2000->ParseMessages(); });

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 wind data sender

  N2kWindDataSender* wind_data_sender = new N2kWindDataSender(
      "/Wind/NMEA2000", tN2kWindReference::N2kWind_Apparent, nmea2000, true);

  apparent_wind_data->speed.connect_to(
      &(wind_data_sender->wind_speed_consumer_));

  apparent_wind_data->angle.connect_to(
      &(wind_data_sender->wind_angle_consumer_));

  // wind_data_sender emits whenever it sends a message; count the messages
  wind_data_sender->connect_to(new LambdaConsumer<std::pair<double, double>>(
      [](std::pair<double, double> wind_data) {
        n2k_tx_counter++;
        n2k_time_since_tx = 0;
      }));

  /////////////////////////////////////////////////////////////////////
  // Configuration elements

  CheckboxConfig* enable_n2k_watchdog_config = new CheckboxConfig(
      false, "Enable NMEA 2000 Watchdog", "/NMEA2000/Enable Watchdog");
  enable_n2k_watchdog_config->set_description(
      "Enable the NMEA 2000 watchdog. If enabled, the device will reboot after "
      "two minutes if no NMEA 2000 messages are received. This setting "
      "requires "
      "a device restart to take effect.");
  enable_n2k_watchdog_config->set_sort_order(100);

  if (enable_n2k_watchdog_config->get_value()) {
    SensESPBaseApp::get_event_loop()->onRepeat(1000, [nmea2000]() {
      if (n2k_time_since_rx > 120000) {
        ESP_LOGE("NMEA2000", "No messages received in 2 minutes. Restarting.");
        // All hope is lost; it doesn't matter if we delay for a bit to ensure
        // the log message is sent.
        delay(10);
        ESP.restart();
      }
    });
  }

  UILambdaOutput<int>* n2k_rx_ui_output = new UILambdaOutput<int>(
      "NMEA 2000 Received Messages", []() { return n2k_rx_counter; },
      "NMEA 2000", 300);

  UILambdaOutput<int>* n2k_tx_ui_output = new UILambdaOutput<int>(
      "NMEA 2000 Transmitted Messages", []() { return n2k_tx_counter; },
      "NMEA 2000", 310);

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

void loop() { SensESPBaseApp::get_event_loop()->tick(); }
