#ifndef AUTONNIC_N2K_SRC_SENDER_N2K_SENDERS_H_
#define AUTONNIC_N2K_SRC_SENDER_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

#include <tuple>

#include "sensesp/system/configurable.h"
#include "sensesp/system/expiring_value.h"
#include "sensesp/system/lambda_consumer.h"

namespace sensesp {

/**
 * @brief Base class for NMEA 2000 senders.
 *
 */
class N2kSender : public Configurable {
 public:
  N2kSender(String config_path) : Configurable{config_path} {}

  virtual void enable() = 0;

  void disable() {
    if (this->sender_reaction_ != nullptr) {
      ReactESP::app->remove(this->sender_reaction_);
      this->sender_reaction_ = nullptr;
    }
  }

 protected:
  RepeatReaction* sender_reaction_ = nullptr;
};

class N2kWindDataSender : public N2kSender,
                          public ValueProducer<std::pair<double, double>> {
 public:
  N2kWindDataSender(String config_path, tN2kWindReference wind_reference,
                    tNMEA2000* nmea2000, bool enable = true)
      : N2kSender{config_path},
        wind_reference_{wind_reference},
        nmea2000_{nmea2000},
        repeat_interval_{100},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{5000}           // In ms. When the inputs expire.
  {
    wind_speed_ = ExpiringValue<double>(N2kDoubleNA, expiry_, N2kDoubleNA);
    wind_angle_ = ExpiringValue<double>(N2kDoubleNA, expiry_, N2kDoubleNA);
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    if (this->sender_reaction_ == nullptr) {
      this->sender_reaction_ =
          ReactESP::app->onRepeat(repeat_interval_, [this]() {
            tN2kMsg N2kMsg;
            // At the moment, the PGN is sent regardless of whether all the
            // values are invalid or not.
            SetN2kWindSpeed(N2kMsg, 255, this->wind_speed_.get(),
                            this->wind_angle_.get(), this->wind_reference_);
            this->nmea2000_->SendMsg(N2kMsg);
            std::pair<double, double> wind_data = std::make_pair(
                this->wind_speed_.get(), this->wind_angle_.get());
            this->emit(wind_data);
          });
    }
  }

  LambdaConsumer<double> wind_speed_consumer_{
      [this](double value) { this->wind_speed_.update(value); }};
  LambdaConsumer<double> wind_angle_consumer_{
      [this](double value) { this->wind_angle_.update(value); }};

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  tN2kWindReference wind_reference_;

  ExpiringValue<double> wind_angle_;
  ExpiringValue<double> wind_speed_;
};

}  // namespace sensesp

#endif  // AUTONNIC_N2K_SRC_SENDER_N2K_SENDERS_H_
