#ifndef AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_
#define AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_

#include <elapsedMillis.h>

#include <tuple>

#include "ReactESP.h"
#include "sensesp.h"
#include "sensesp/system/configurable.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_nmea0183/nmea0183.h"

using namespace sensesp;

String AutonnicReferenceAngleSentence(const float& offset) {
  // Example sentence: $PATC,IIMWV,AHD,x.x
  float offset_degrees = offset * 180 / M_PI;
  String sentence = "$PATC,IIMWV,AHD," + String(offset_degrees, 1);
  // Note that the checksum is not included in the sentence.
  return sentence;
}

enum AsyncCommandStatus {
  kReady,
  kPending,
  kSuccess,
  kFailure,
  kTimeout,
};

template <typename T, typename U>
class AsyncCommand : public ValueProducer<U> {
 public:
  bool send(T value) {
    elapsed_millis_ = 0;
    status_ = kPending;

    if (timeout_reaction_ != nullptr) {
      ReactESP::app->remove(timeout_reaction_);
      timeout_reaction_ = nullptr;
    }
    timeout_reaction_ = ReactESP::app->onDelay(2000, [this]() {
      if (status_ == kPending) {
        status_ = kTimeout;
      }
      this->timeout_reaction_ = nullptr;
    });

    // Send a command to the device
    return send_command(value);
  }

  virtual bool send_command(T value) = 0;

  LambdaConsumer<std::tuple<bool, String, U>> result_consumer_{
      [this](std::tuple<bool, String> result) {
        if (status_ != kPending) {
          return;
        }
        // Clear the timeout reaction
        if (timeout_reaction_ != nullptr) {
          ReactESP::app->remove(timeout_reaction_);
          timeout_reaction_ = nullptr;
        }
        result_message_ = std::get<1>(result);
        if (std::get<0>(result)) {
          status_ = kSuccess;
          this->emit(std::get<2>(result));
        } else {
          status_ = kFailure;
        }
      }};

 protected:
  DelayReaction* timeout_reaction_ = nullptr;
  AsyncCommandStatus status_ = kReady;
  String result_message_;
  elapsedMillis elapsed_millis_;
};

class ReferenceAngleConfig : public Configurable,
                             public AsyncCommand<float, String> {
 public:
  ReferenceAngleConfig(float offset, const NMEA0183* nmea0183,
                       String config_path = "")
      : Configurable(config_path),
        AsyncCommand(),
        offset_{offset},
        nmea0183_{nmea0183} {}

  virtual void get_configuration(JsonObject& doc) override {
    doc["offset"] = offset_;
  }

  virtual bool set_local_configuration(const JsonObject& config) override {
    String expected_keys[] = {"offset"};
    for (auto& key : expected_keys) {
      if (!config.containsKey(key)) {
        return false;
      }
    }
    offset_ = config["offset"];
    return true;
  }

  virtual void set_configuration(const JsonObject& config,
                                 ConfigurableResult& result) override {
    if (!set_local_configuration(config)) {
      result = ConfigurableResult::kConfigError;
      return;
    }
    this->send(offset_);
    result = ConfigurableResult::kConfigPending;
  }

  virtual bool send_command(float value) override {
    // Send the command to the device
    String sentence = AutonnicReferenceAngleSentence(value);
    nmea0183_->output_raw(sentence.c_str());
  }

  virtual String get_config_schema() override {
    const char schema[] = R"({
      "type": "object",
      "properties": {
        "offset": { "title": "Offset", "type": "number" }
      }
    })";
  }

 protected:
  float offset_;  // Offset in radians
  const NMEA0183* nmea0183_;
};

#endif  // AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_
