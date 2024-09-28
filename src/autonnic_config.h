#ifndef AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_
#define AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_

#include <elapsedMillis.h>
#include <sensesp/transforms/zip.h>

#include <tuple>

#include "ReactESP.h"
#include "autonnic_a5120_parser.h"
#include "sensesp.h"
#include "sensesp/system/async_response_handler.h"
#include "sensesp/system/configurable.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_nmea0183/nmea0183.h"

using namespace sensesp;

String AutonnicReferenceAngleSentence(const float& offset) {
  // Example sentence: $PATC,IIMWV,AHD,x.x
  float offset_degrees = offset * 180 / M_PI;
  char buf[100];
  snprintf(buf, sizeof(buf), "$PATC,IIMWV,AHD,%0.1f", offset_degrees);
  // Note that the checksum is not included in the sentence.
  return buf;
}

String AutonnicWindDirectionDampingSentence(const float& damping_factor) {
  // Example sentence: $PATC,IIMWV,DWD,x.x
  char buf[100];
  snprintf(buf, sizeof(buf), "$PATC,IIMWV,DWD,%0.1f", damping_factor);
  // Note that the checksum is not included in the sentence.
  return buf;
}

String AutonnicWindSpeedDampingSentence(const float& damping_factor) {
  // Example sentence: $PATC,IIMWV,DWD,x.x
  char buf[100];
  snprintf(buf, sizeof(buf), "$PATC,IIMWV,DSP,%0.1f", damping_factor);
  // Note that the checksum is not included in the sentence.
  return buf;
}

String AutonnicMessageRepetitionRateSentence(const int& repetition_rate) {
  // Example sentence: $PATC,IIMWV,TXP,xxxx
  char buf[100];
  snprintf(buf, sizeof(buf), "$PATC,IIMWV,TXP,%d", repetition_rate);
  // Note that the checksum is not included in the sentence.
  return buf;
}

class ReferenceAngleConfig : public Configurable {
 public:
  ReferenceAngleConfig(float offset, Stream* serial,
                       AutonnicPATCWIMWVParser* parser, String config_path = "")
      : Configurable(config_path),
        offset_{offset},
        serial_{serial},
        parser_{parser} {
    load_configuration();
    parser_->connect_to(&set_response_handler_);
  }

  virtual void get_configuration(JsonObject& doc) override {
    doc["offset"] = offset_;
  }

  virtual bool is_async() override { return true; }

  virtual ConfigurableResult async_get_configuration() override {
    // Autonnic A5120 does not support getting the configuration. Indicate
    // that the locally stored configuration is available.
    return ConfigurableResult::kOk;
  }

  virtual ConfigurableResult poll_get_result(
      JsonObject& config_object) override {
    // Autonnic A5120 does not support getting the configuration, so just
    // return the current locally stored configuration.
    get_configuration(config_object);
    return ConfigurableResult::kOk;
  }

  virtual bool set_configuration(const JsonObject& config) override {
    String expected_keys[] = {"offset"};
    for (auto& key : expected_keys) {
      if (!config.containsKey(key)) {
        return false;
      }
    }
    offset_ = config["offset"];
    return true;
  }

  virtual ConfigurableResult async_set_configuration(
      const JsonObject& config) override {
    if (!set_configuration(config)) {
      return ConfigurableResult::kError;
    }
    // Send the command to the device
    ESP_LOGD("ReferenceAngleConfig", "Sending command: %f", offset_);
    String sentence = AutonnicReferenceAngleSentence(offset_);
    ESP_LOGD("ReferenceAngleConfig", "Sending sentence: %s", sentence.c_str());
    serial_->println(sentence);
    set_response_handler_.activate();
    return ConfigurableResult::kPending;
  }

  virtual ConfigurableResult poll_set_result() override {
    auto status = set_response_handler_.get_status();
    ESP_LOGV("ReferenceAngleConfig", "status_: %d", status);
    switch (status) {
      case AsyncResponseStatus::kReady:
        return ConfigurableResult::kOk;
      case AsyncResponseStatus::kPending:
        return ConfigurableResult::kPending;
      case AsyncResponseStatus::kSuccess:
        return ConfigurableResult::kOk;
      case AsyncResponseStatus::kFailure:
        return ConfigurableResult::kError;
      case AsyncResponseStatus::kTimeout:
        return ConfigurableResult::kError;
    }
    return ConfigurableResult::kError;
  }

  virtual String get_config_schema() override {
    const char schema[] = R"({
      "type": "object",
      "properties": {
        "offset": { "title": "Offset", "type": "number", "displayMultiplier": 0.017453292519943295, "displayOffset": 0 }
      }
    })";
    return schema;
  }

 protected:
  float offset_;  // Offset in radians
  Stream* serial_;
  AutonnicPATCWIMWVParser* parser_;
  AsyncResponseHandler set_response_handler_;
};

class WindDirectionDampingConfig : public Configurable {
 public:
  WindDirectionDampingConfig(float damping_factor, Stream* serial,
                             AutonnicPATCWIMWVParser* parser,
                             String config_path = "")
      : Configurable(config_path),
        damping_factor_{damping_factor},
        serial_{serial},
        parser_{parser} {
    load_configuration();
    parser_->connect_to(&set_response_handler_);
  }

  virtual void get_configuration(JsonObject& doc) override {
    doc["damping_factor"] = damping_factor_;
  }

  virtual bool is_async() override { return true; }

  virtual ConfigurableResult async_get_configuration() override {
    // Autonnic A5120 does not support getting the configuration. Indicate
    // that the locally stored configuration is available.
    return ConfigurableResult::kOk;
  }

  virtual ConfigurableResult poll_get_result(
      JsonObject& config_object) override {
    // Autonnic A5120 does not support getting the configuration, so just
    // return the current locally stored configuration.
    get_configuration(config_object);
    return ConfigurableResult::kOk;
  }

  virtual bool set_configuration(const JsonObject& config) override {
    String expected_keys[] = {"damping_factor"};
    for (auto& key : expected_keys) {
      if (!config.containsKey(key)) {
        return false;
      }
    }
    damping_factor_ = config["damping_factor"];
    return true;
  }

  virtual ConfigurableResult async_set_configuration(
      const JsonObject& config) override {
    if (!set_configuration(config)) {
      return ConfigurableResult::kError;
    }
    // Send the command to the device
    ESP_LOGD("WindDirectionDampingConfig", "Sending command: %f",
             damping_factor_);
    String sentence = AutonnicWindDirectionDampingSentence(damping_factor_);
    ESP_LOGD("WindDirectionDampingConfig", "Sending sentence: %s",
             sentence.c_str());
    serial_->println(sentence.c_str());
    set_response_handler_.activate();
    return ConfigurableResult::kPending;
  }

  virtual ConfigurableResult poll_set_result() override {
    auto status = set_response_handler_.get_status();
    ESP_LOGV("WindDirectionDampingConfig", "status_: %d", status);
    switch (status) {
      case AsyncResponseStatus::kReady:
        return ConfigurableResult::kOk;
      case AsyncResponseStatus::kPending:
        return ConfigurableResult::kPending;
      case AsyncResponseStatus::kSuccess:
        return ConfigurableResult::kOk;
      case AsyncResponseStatus::kFailure:
        return ConfigurableResult::kError;
      case AsyncResponseStatus::kTimeout:
        return ConfigurableResult::kError;
    }
    return ConfigurableResult::kError;
  }

  virtual String get_config_schema() override {
    const char schema[] = R"({
      "type": "object",
      "properties": {
        "damping_factor": { "title": "Damping Factor", "type": "number" }
      }
    })";
    return schema;
  }

 protected:
  float damping_factor_;  // Damping factor in percent
  Stream* serial_;
  AutonnicPATCWIMWVParser* parser_;
  AsyncResponseHandler set_response_handler_;
};

class WindSpeedDampingConfig : public Configurable {
 public:
  WindSpeedDampingConfig(float damping_factor, Stream* serial,
                         AutonnicPATCWIMWVParser* parser,
                         String config_path = "")
      : Configurable(config_path),
        damping_factor_{damping_factor},
        serial_{serial},
        parser_{parser} {
    load_configuration();
    parser_->connect_to(&set_response_handler_);
  }

  virtual void get_configuration(JsonObject& doc) override {
    doc["damping_factor"] = damping_factor_;
  }

  virtual bool is_async() override { return true; }

  virtual ConfigurableResult async_get_configuration() override {
    // Autonnic A5120 does not support getting the configuration. Indicate
    // that the locally stored configuration is available.
    return ConfigurableResult::kOk;
  }

  virtual ConfigurableResult poll_get_result(
      JsonObject& config_object) override {
    // Autonnic A5120 does not support getting the configuration, so just
    // return the current locally stored configuration.
    get_configuration(config_object);
    return ConfigurableResult::kOk;
  }

  virtual bool set_configuration(const JsonObject& config) override {
    String expected_keys[] = {"damping_factor"};
    for (auto& key : expected_keys) {
      if (!config.containsKey(key)) {
        return false;
      }
    }
    damping_factor_ = config["damping_factor"];
    return true;
  }

  virtual ConfigurableResult async_set_configuration(
      const JsonObject& config) override {
    if (!set_configuration(config)) {
      return ConfigurableResult::kError;
    }
    // Send the command to the device
    ESP_LOGD("WindSpeedDampingConfig", "Sending command: %f", damping_factor_);
    String sentence = AutonnicWindSpeedDampingSentence(damping_factor_);
    ESP_LOGD("WindSpeedDampingConfig", "Sending sentence: %s",
             sentence.c_str());
    serial_->println(sentence.c_str());
    set_response_handler_.activate();
    return ConfigurableResult::kPending;
  }

  virtual ConfigurableResult poll_set_result() override {
    auto status = set_response_handler_.get_status();
    ESP_LOGV("WindSpeedDampingConfig", "status_: %d", status);
    switch (status) {
      case AsyncResponseStatus::kReady:
        return ConfigurableResult::kOk;
      case AsyncResponseStatus::kPending:
        return ConfigurableResult::kPending;
      case AsyncResponseStatus::kSuccess:
        return ConfigurableResult::kOk;
      case AsyncResponseStatus::kFailure:
        return ConfigurableResult::kError;
      case AsyncResponseStatus::kTimeout:
        return ConfigurableResult::kError;
    }
    return ConfigurableResult::kError;
  }

  virtual String get_config_schema() override {
    const char schema[] = R"({
      "type": "object",
      "properties": {
        "damping_factor": { "title": "Damping Factor", "type": "number" }
      }
    })";
    return schema;
  }

 protected:
  float damping_factor_;  // Damping factor in percent
  Stream* serial_;
  AutonnicPATCWIMWVParser* parser_;
  AsyncResponseHandler set_response_handler_;
};

class WindOutputRepetitionRateConfig : public Configurable {
 public:
  WindOutputRepetitionRateConfig(float repetition_rate,
                                 Stream* serial,
                                 AutonnicPATCWIMWVParser* parser,
                                 String config_path = "")
      : Configurable(config_path),
        repetition_rate_{repetition_rate},
        serial_{serial},
        parser_{parser} {
    load_configuration();

    parser->connect_to(&set_response_handler_);
  }

  virtual void get_configuration(JsonObject& doc) override {
    doc["repetition_rate"] = repetition_rate_;
  }

  virtual bool is_async() override { return true; }

  virtual ConfigurableResult async_get_configuration() override {
    // Autonnic A5120 does not support getting the configuration. Indicate
    // that the locally stored configuration is available.
    return ConfigurableResult::kOk;
  }

  virtual ConfigurableResult poll_get_result(
      JsonObject& config_object) override {
    // Autonnic A5120 does not support getting the configuration, so just
    // return the current locally stored configuration.
    get_configuration(config_object);
    return ConfigurableResult::kOk;
  }

  virtual bool set_configuration(const JsonObject& config) override {
    String expected_keys[] = {"repetition_rate"};
    for (auto& key : expected_keys) {
      if (!config.containsKey(key)) {
        return false;
      }
    }
    repetition_rate_ = config["repetition_rate"];
    return true;
  }

  virtual ConfigurableResult async_set_configuration(
      const JsonObject& config) override {
    if (!set_configuration(config)) {
      return ConfigurableResult::kError;
    }
    // Send the command to the device
    ESP_LOGD("WindOutputRepetitionRate", "Sending command: %f",
             repetition_rate_);
    String sentence = AutonnicMessageRepetitionRateSentence(repetition_rate_);
    ESP_LOGD("WindOutputRepetitionRate", "Sending sentence: %s",
             sentence.c_str());
    serial_->println(sentence.c_str());
    set_response_handler_.activate();
    return ConfigurableResult::kPending;
  }

  virtual ConfigurableResult poll_set_result() override {
    auto status = set_response_handler_.get_status();
    ESP_LOGV("WindOutputRepetitionRate", "status_: %d", status);
    switch (status) {
      case AsyncResponseStatus::kReady:
        return ConfigurableResult::kOk;
      case AsyncResponseStatus::kPending:
        return ConfigurableResult::kPending;
      case AsyncResponseStatus::kSuccess:
        return ConfigurableResult::kOk;
      case AsyncResponseStatus::kFailure:
        return ConfigurableResult::kError;
      case AsyncResponseStatus::kTimeout:
        return ConfigurableResult::kError;
    }
    return ConfigurableResult::kError;
  }

  virtual String get_config_schema() override {
    const char schema[] = R"({
      "type": "object",
      "properties": {
        "repetition_rate": { "title": "Message Repetition Rate", "type": "integer" }
      }
    })";
    return schema;
  }

 protected:
  float repetition_rate_;  // Damping factor in percent
  Stream* serial_;
  AutonnicPATCWIMWVParser* parser_;
  AsyncResponseHandler set_response_handler_;
};

#endif  // AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_
