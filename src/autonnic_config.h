#ifndef AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_
#define AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_

#include <elapsedMillis.h>
#include <sensesp/transforms/zip.h>

#include <tuple>

#include "ReactESP.h"
#include "autonnic_a5120_parser.h"
#include "sensesp.h"
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

enum AsyncCommandStatus {
  kReady,
  kPending,
  kSuccess,
  kFailure,
  kTimeout,
};

/**
 * @brief Abstract base class for handling async command sending to remote
 * systems.
 *
 * This class is used to send a command to a remote system and wait for a
 * response. The response is then emitted as a value.
 *
 * @tparam T Command data type
 * @tparam U Response data type
 */
template <typename T, typename U>
class AsyncCommand : public ValueProducer<U> {
 public:
  AsyncCommand() : ValueProducer<U>() {}
  AsyncCommand(int timeout) : ValueProducer<U>(), timeout_{timeout} {}

  bool send(T value) {
    elapsed_millis_ = 0;
    status_ = kPending;

    if (timeout_reaction_ != nullptr) {
      ReactESP::app->remove(timeout_reaction_);
      timeout_reaction_ = nullptr;
    }
    timeout_reaction_ = ReactESP::app->onDelay(timeout_, [this]() {
      if (status_ == kPending) {
        status_ = kTimeout;
      }
      this->timeout_reaction_ = nullptr;
    });

    // Send a command to the device
    return send_command(value);
  }

  /**
   * @brief Send a command to the remote system.
   *
   * @param value
   * @return true
   * @return false
   */
  virtual bool send_command(T value) = 0;

 protected:
  DelayReaction* timeout_reaction_ = nullptr;
  AsyncCommandStatus status_ = kReady;
  String result_message_;
  int timeout_ = 3000;  // Default timeout in ms
  elapsedMillis elapsed_millis_;

  void report_result(bool success, U& result) {
    if (status_ != kPending) {
      return;
    }

    // Clear the timeout reaction
    if (timeout_reaction_ != nullptr) {
      ReactESP::app->remove(timeout_reaction_);
      timeout_reaction_ = nullptr;
    }

    if (success) {
      status_ = kSuccess;
      this->emit(result);
    } else {
      status_ = kFailure;
    }
  }
};

class ReferenceAngleConfig : public Configurable,
                             public AsyncCommand<float, bool> {
 public:
  ReferenceAngleConfig(float offset, const NMEA0183* nmea0183,
                       AutonnicPATCWIMWVParser* parser, String config_path = "")
      : Configurable(config_path),
        AsyncCommand(),
        offset_{offset},
        nmea0183_{nmea0183},
        parser_{parser} {
    parser->result_.connect_to(new LambdaConsumer<bool>([this](bool result) {
      report_result(result, result);  // Result also indicates success status
    }));
    load_configuration();
  }

  virtual void get_configuration(JsonObject& doc) override {
    doc["offset"] = offset_;
  }

  virtual bool is_async() override { return true; }

  virtual ConfigurableResult async_get_configuration() override {
    // Autonnic A5120 does not support getting the configuration. Indicate
    // that the locally stored configuration is available.
    return ConfigurableResult::kConfigOk;
  }

  virtual ConfigurableResult poll_get_result(
      JsonObject& config_object) override {
    // Autonnic A5120 does not support getting the configuration, so just
    // return the current locally stored configuration.
    get_configuration(config_object);
    return ConfigurableResult::kConfigOk;
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
      return ConfigurableResult::kConfigError;
    }
    this->send(offset_);
    return ConfigurableResult::kConfigPending;
  }

  virtual ConfigurableResult poll_set_result() override {
    ESP_LOGV("ReferenceAngleConfig", "status_: %d", status_);
    switch (status_) {
      case kReady:
        return ConfigurableResult::kConfigOk;
      case kPending:
        return ConfigurableResult::kConfigPending;
      case kSuccess:
        return ConfigurableResult::kConfigOk;
      case kFailure:
        return ConfigurableResult::kConfigError;
      case kTimeout:
        return ConfigurableResult::kConfigError;
    }
    return ConfigurableResult::kConfigError;
  }

  virtual bool send_command(float value) override {
    // Send the command to the device
    ESP_LOGD("ReferenceAngleConfig", "Sending command: %f", value);
    String sentence = AutonnicReferenceAngleSentence(value);
    ESP_LOGD("ReferenceAngleConfig", "Sending sentence: %s", sentence.c_str());
    nmea0183_->output_raw(sentence.c_str());
    return true;  // Cannot fail
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
  const NMEA0183* nmea0183_;
  AutonnicPATCWIMWVParser* parser_;
};

class WindDirectionDampingConfig : public Configurable,
                                   public AsyncCommand<float, bool> {
 public:
  WindDirectionDampingConfig(float damping_factor, const NMEA0183* nmea0183,
                             AutonnicPATCWIMWVParser* parser,
                             String config_path = "")
      : Configurable(config_path),
        AsyncCommand(),
        damping_factor_{damping_factor},
        nmea0183_{nmea0183},
        parser_{parser} {
    parser->result_.connect_to(new LambdaConsumer<bool>([this](bool result) {
      report_result(result, result);  // Result also indicates success status
    }));
    load_configuration();
  }

  virtual void get_configuration(JsonObject& doc) override {
    doc["damping_factor"] = damping_factor_;
  }

  virtual bool is_async() override { return true; }

  virtual ConfigurableResult async_get_configuration() override {
    // Autonnic A5120 does not support getting the configuration. Indicate
    // that the locally stored configuration is available.
    return ConfigurableResult::kConfigOk;
  }

  virtual ConfigurableResult poll_get_result(
      JsonObject& config_object) override {
    // Autonnic A5120 does not support getting the configuration, so just
    // return the current locally stored configuration.
    get_configuration(config_object);
    return ConfigurableResult::kConfigOk;
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
      return ConfigurableResult::kConfigError;
    }
    this->send(damping_factor_);
    return ConfigurableResult::kConfigPending;
  }

  virtual ConfigurableResult poll_set_result() override {
    ESP_LOGV("WindDirectionDampingConfig", "status_: %d", status_);
    switch (status_) {
      case kReady:
        return ConfigurableResult::kConfigOk;
      case kPending:
        return ConfigurableResult::kConfigPending;
      case kSuccess:
        return ConfigurableResult::kConfigOk;
      case kFailure:
        return ConfigurableResult::kConfigError;
      case kTimeout:
        return ConfigurableResult::kConfigError;
    }
    return ConfigurableResult::kConfigError;
  }

  virtual bool send_command(float value) override {
    // Send the command to the device
    ESP_LOGD("WindDirectionDampingConfig", "Sending command: %f", value);
    String sentence = AutonnicWindDirectionDampingSentence(value);
    ESP_LOGD("WindDirectionDampingConfig", "Sending sentence: %s",
             sentence.c_str());
    nmea0183_->output_raw(sentence.c_str());
    return true;  // Cannot fail
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
  const NMEA0183* nmea0183_;
  AutonnicPATCWIMWVParser* parser_;
};

class WindSpeedDampingConfig : public Configurable,
                               public AsyncCommand<float, bool> {
 public:
  WindSpeedDampingConfig(float damping_factor, const NMEA0183* nmea0183,
                         AutonnicPATCWIMWVParser* parser,
                         String config_path = "")
      : Configurable(config_path),
        AsyncCommand(),
        damping_factor_{damping_factor},
        nmea0183_{nmea0183},
        parser_{parser} {
    parser->result_.connect_to(new LambdaConsumer<bool>([this](bool result) {
      report_result(result, result);  // Result also indicates success status
    }));
    load_configuration();
  }

  virtual void get_configuration(JsonObject& doc) override {
    doc["damping_factor"] = damping_factor_;
  }

  virtual bool is_async() override { return true; }

  virtual ConfigurableResult async_get_configuration() override {
    // Autonnic A5120 does not support getting the configuration. Indicate
    // that the locally stored configuration is available.
    return ConfigurableResult::kConfigOk;
  }

  virtual ConfigurableResult poll_get_result(
      JsonObject& config_object) override {
    // Autonnic A5120 does not support getting the configuration, so just
    // return the current locally stored configuration.
    get_configuration(config_object);
    return ConfigurableResult::kConfigOk;
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
      return ConfigurableResult::kConfigError;
    }
    this->send(damping_factor_);
    return ConfigurableResult::kConfigPending;
  }

  virtual ConfigurableResult poll_set_result() override {
    ESP_LOGV("WindSpeedDampingConfig", "status_: %d", status_);
    switch (status_) {
      case kReady:
        return ConfigurableResult::kConfigOk;
      case kPending:
        return ConfigurableResult::kConfigPending;
      case kSuccess:
        return ConfigurableResult::kConfigOk;
      case kFailure:
        return ConfigurableResult::kConfigError;
      case kTimeout:
        return ConfigurableResult::kConfigError;
    }
    return ConfigurableResult::kConfigError;
  }

  virtual bool send_command(float value) override {
    // Send the command to the device
    ESP_LOGD("WindSpeedDampingConfig", "Sending command: %f", value);
    String sentence = AutonnicWindSpeedDampingSentence(value);
    ESP_LOGD("WindSpeedDampingConfig", "Sending sentence: %s",
             sentence.c_str());
    nmea0183_->output_raw(sentence.c_str());
    return true;  // Cannot fail
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
  const NMEA0183* nmea0183_;
  AutonnicPATCWIMWVParser* parser_;
};

class WindOutputRepetitionRateConfig : public Configurable,
                                       public AsyncCommand<float, bool> {
 public:
  WindOutputRepetitionRateConfig(float repetition_rate,
                                 const NMEA0183* nmea0183,
                                 AutonnicPATCWIMWVParser* parser,
                                 String config_path = "")
      : Configurable(config_path),
        AsyncCommand(),
        repetition_rate_{repetition_rate},
        nmea0183_{nmea0183},
        parser_{parser} {
    parser->result_.connect_to(new LambdaConsumer<bool>([this](bool result) {
      report_result(result, result);  // Result also indicates success status
    }));
    load_configuration();
  }

  virtual void get_configuration(JsonObject& doc) override {
    doc["repetition_rate"] = repetition_rate_;
  }

  virtual bool is_async() override { return true; }

  virtual ConfigurableResult async_get_configuration() override {
    // Autonnic A5120 does not support getting the configuration. Indicate
    // that the locally stored configuration is available.
    return ConfigurableResult::kConfigOk;
  }

  virtual ConfigurableResult poll_get_result(
      JsonObject& config_object) override {
    // Autonnic A5120 does not support getting the configuration, so just
    // return the current locally stored configuration.
    get_configuration(config_object);
    return ConfigurableResult::kConfigOk;
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
      return ConfigurableResult::kConfigError;
    }
    this->send(repetition_rate_);
    return ConfigurableResult::kConfigPending;
  }

  virtual ConfigurableResult poll_set_result() override {
    ESP_LOGV("WindOutputRepetitionRate", "status_: %d", status_);
    switch (status_) {
      case kReady:
        return ConfigurableResult::kConfigOk;
      case kPending:
        return ConfigurableResult::kConfigPending;
      case kSuccess:
        return ConfigurableResult::kConfigOk;
      case kFailure:
        return ConfigurableResult::kConfigError;
      case kTimeout:
        return ConfigurableResult::kConfigError;
    }
    return ConfigurableResult::kConfigError;
  }

  virtual bool send_command(float value) override {
    // Send the command to the device
    ESP_LOGD("WindOutputRepetitionRate", "Sending command: %f", value);
    String sentence = AutonnicMessageRepetitionRateSentence(value);
    ESP_LOGD("WindOutputRepetitionRate", "Sending sentence: %s",
             sentence.c_str());
    nmea0183_->output_raw(sentence.c_str());
    return true;  // Cannot fail
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
  const NMEA0183* nmea0183_;
  AutonnicPATCWIMWVParser* parser_;
};

#endif  // AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_
