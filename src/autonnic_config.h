#ifndef AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_
#define AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_

#include <elapsedMillis.h>
#include <sensesp/transforms/zip.h>

#include <tuple>

#include "ReactESP.h"
#include "autonnic_a5120_parser.h"
#include "sensesp.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/semaphore_value.h"
#include "sensesp/system/serializable.h"
#include "sensesp_nmea0183/nmea0183.h"

namespace wind_interface {

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

class ReferenceAngleConfig : public sensesp::FileSystemSaveable,
                             virtual public sensesp::Serializable {
 public:
  ReferenceAngleConfig(sensesp::nmea0183::NMEA0183IOTask* nmea_io_task,
                       float offset, AutonnicPATCWIMWVParser* parser,
                       String config_path = "")
      : sensesp::FileSystemSaveable(config_path),
        sensesp::Serializable(),
        nmea_io_task_{nmea_io_task},
        offset_{offset},
        response_parser_{parser} {
    load();
    response_parser_->connect_to(&response_semaphore_);
  }

  inline virtual bool to_json(JsonObject& doc) override {
    doc["offset"] = offset_;
    return true;
  }

  inline virtual bool from_json(const JsonObject& config) override {
    String expected_keys[] = {"offset"};
    for (auto& key : expected_keys) {
      if (!config[key].is<JsonVariant>()) {
        return false;
      }
    }
    offset_ = config["offset"];
    return true;
  }

  inline virtual bool load() override {
    // Autonnic A5120 does not support getting the configuration. Load
    // from local file storage.
    return this->FileSystemSaveable::load();
  }

  inline virtual bool save() override {
    // Save to local filesystem
    this->FileSystemSaveable::save();
    // Send the command to the device
    ESP_LOGD("ReferenceAngleConfig", "Sending command: %f", offset_);
    String sentence = AutonnicReferenceAngleSentence(offset_);
    ESP_LOGD("ReferenceAngleConfig", "Sending sentence: %s", sentence.c_str());
    response_semaphore_.clear();
    // Indirection using onDelay ensures the command is sent from the event loop
    // thread and does not interfere with any other serial communication.
    sensesp::event_loop()->onDelay(
        0, [this, sentence]() { nmea_io_task_->set(sentence); });
    if (!response_semaphore_.take(1000)) {
      return false;
    }
    return true;
  }

 protected:
  sensesp::nmea0183::NMEA0183IOTask* nmea_io_task_;
  float offset_;  // Offset in radians
  AutonnicPATCWIMWVParser* response_parser_;
  sensesp::SemaphoreValue<bool> response_semaphore_;
};

inline const String ConfigSchema(const ReferenceAngleConfig& obj) {
  const char schema[] = R"({
      "type": "object",
      "properties": {
        "offset": { "title": "Offset", "type": "number", "displayMultiplier": 0.017453292519943295, "displayOffset": 0 }
      }
    })";
  return schema;
}

class WindDirectionDampingConfig : public sensesp::FileSystemSaveable,
                                   virtual public sensesp::Serializable {
 public:
  WindDirectionDampingConfig(sensesp::nmea0183::NMEA0183IOTask* nmea_io_task,
                             float damping_factor,
                             AutonnicPATCWIMWVParser* response_parser,
                             String config_path = "")
      : sensesp::FileSystemSaveable(config_path),
        sensesp::Serializable(),
        nmea_io_task_{nmea_io_task},
        damping_factor_{damping_factor},
        response_parser_{response_parser} {
    load();
    response_parser_->connect_to(&response_semaphore_);
  }

  inline virtual bool to_json(JsonObject& doc) override {
    doc["damping_factor"] = damping_factor_;
    return true;
  }

  inline virtual bool from_json(const JsonObject& config) override {
    String expected_keys[] = {"damping_factor"};
    for (auto& key : expected_keys) {
      if (!config[key].is<JsonVariant>()) {
        return false;
      }
    }
    damping_factor_ = config["damping_factor"];
    return true;
  }

  inline virtual bool save() override {
    // Save to local filesystem
    FileSystemSaveable::save();
    // Send the command to the device
    ESP_LOGD("WindDirectionDampingConfig", "Sending command: %f",
             damping_factor_);
    String sentence = AutonnicWindDirectionDampingSentence(damping_factor_);
    ESP_LOGD("WindDirectionDampingConfig", "Sending sentence: %s",
             sentence.c_str());
    response_semaphore_.clear();
    sensesp::event_loop()->onDelay(
        0, [this, sentence]() { nmea_io_task_->set(sentence); });
    if (!response_semaphore_.take(1000)) {
      return false;
    }

    return true;
  }

 protected:
  sensesp::nmea0183::NMEA0183IOTask* nmea_io_task_;
  float damping_factor_;  // Damping factor in percent
  AutonnicPATCWIMWVParser* response_parser_;
  sensesp::SemaphoreValue<bool> response_semaphore_;
};

const String ConfigSchema(const WindDirectionDampingConfig& obj) {
  const char schema[] = R"({
      "type": "object",
      "properties": {
        "damping_factor": { "title": "Damping Factor", "type": "number" }
      }
    })";
  return schema;
}

class WindSpeedDampingConfig : public sensesp::FileSystemSaveable,
                               virtual public sensesp::Serializable {
 public:
  WindSpeedDampingConfig(sensesp::nmea0183::NMEA0183IOTask* nmea_io_task,
                         float damping_factor, AutonnicPATCWIMWVParser* parser,
                         String config_path = "")
      : sensesp::FileSystemSaveable(config_path),
        sensesp::Serializable(),
        nmea_io_task_{nmea_io_task},
        damping_factor_{damping_factor},
        response_parser_{parser} {
    load();
    response_parser_->connect_to(&response_semaphore_);
  }

  inline virtual bool to_json(JsonObject& doc) override {
    doc["damping_factor"] = damping_factor_;
    return true;
  }

  inline virtual bool from_json(const JsonObject& config) override {
    String expected_keys[] = {"damping_factor"};
    for (auto& key : expected_keys) {
      if (!config[key].is<JsonVariant>()) {
        return false;
      }
    }
    damping_factor_ = config["damping_factor"];
    return true;
  }

  inline virtual bool load() override {
    // Autonnic A5120 does not support getting the configuration. Load
    // from local file storage.
    return FileSystemSaveable::load();
  }

  inline virtual bool save() override {
    // Save to local filesystem
    FileSystemSaveable::save();
    // Send the command to the device
    ESP_LOGD("WindSpeedDampingConfig", "Sending command: %f", damping_factor_);
    String sentence = AutonnicWindSpeedDampingSentence(damping_factor_);
    ESP_LOGD("WindSpeedDampingConfig", "Sending sentence: %s",
             sentence.c_str());
    response_semaphore_.clear();
    sensesp::event_loop()->onDelay(
        0, [this, sentence]() { nmea_io_task_->set(sentence); });
    if (!response_semaphore_.take(1000)) {
      return false;
    }
    return true;
  }

 protected:
  sensesp::nmea0183::NMEA0183IOTask* nmea_io_task_;
  float damping_factor_;  // Damping factor in percent
  AutonnicPATCWIMWVParser* response_parser_;
  sensesp::SemaphoreValue<bool> response_semaphore_;
};

inline const String ConfigSchema(const WindSpeedDampingConfig& obj) {
  const char schema[] = R"({
      "type": "object",
      "properties": {
        "damping_factor": { "title": "Damping Factor", "type": "number" }
      }
    })";
  return schema;
}

class WindOutputRepetitionRateConfig : public sensesp::FileSystemSaveable,
                                       virtual public sensesp::Serializable {
 public:
  WindOutputRepetitionRateConfig(
      sensesp::nmea0183::NMEA0183IOTask* nmea_io_task, float repetition_rate,
      AutonnicPATCWIMWVParser* response_parser, String config_path = "")
      : sensesp::FileSystemSaveable(config_path),
        sensesp::Serializable(),
        nmea_io_task_{nmea_io_task},
        repetition_rate_{repetition_rate},
        response_parser_{response_parser} {
    load();

    response_parser_->connect_to(&response_semaphore_);
  }

  inline virtual bool to_json(JsonObject& doc) override {
    doc["repetition_rate"] = repetition_rate_;
    return true;
  }

  inline virtual bool from_json(const JsonObject& config) override {
    String expected_keys[] = {"repetition_rate"};
    for (auto& key : expected_keys) {
      if (!config[key].is<JsonVariant>()) {
        return false;
      }
    }
    repetition_rate_ = config["repetition_rate"];

    return true;
  }

  inline virtual bool load() override {
    // Autonnic A5120 does not support getting the configuration. Load
    // from local file storage.
    return FileSystemSaveable::load();
  }

  inline virtual bool save() override {
    // Save to local filesystem
    FileSystemSaveable::save();
    // Send the command to the device
    String sentence = AutonnicMessageRepetitionRateSentence(repetition_rate_);
    ESP_LOGD("WindOutputRepetitionRate", "Sending sentence: %s",
             sentence.c_str());
    response_semaphore_.clear();
    nmea_io_task_->set(sentence);  // Queue the command
    // Wait until the response is received
    if (!response_semaphore_.take(5000)) {
      ESP_LOGE("WindOutputRepetitionRate", "No response received");
      return false;
    }
    ESP_LOGV("WindOutputRepetitionRate", "Response received");
    return true;
  }

 protected:
  sensesp::nmea0183::NMEA0183IOTask* nmea_io_task_;
  float repetition_rate_;  // Damping factor in percent
  AutonnicPATCWIMWVParser* response_parser_;
  sensesp::SemaphoreValue<bool> response_semaphore_;
};

inline const String ConfigSchema(const WindOutputRepetitionRateConfig& obj) {
  const char schema[] = R"({
      "type": "object",
      "properties": {
        "repetition_rate": { "title": "Message Repetition Rate", "type": "integer" }
      }
    })";
  return schema;
}

}  // namespace wind_interface

#endif  // AUTONNIC_WIND_SRC_AUTONNIC_CONFIG_H_
