#ifndef AUTONNIC_N2K_SRC_SENDER_N2K_SENDERS_H_
#define AUTONNIC_N2K_SRC_SENDER_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

#include <tuple>

#include "sensesp/system/expiring_value.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/serializable.h"
#include "sensesp/transforms/repeat.h"

namespace wind_interface {

/**
 * @brief Base class for NMEA 2000 senders.
 *
 */
class N2kSender : public sensesp::Saveable, public sensesp::Serializable {
 public:
  N2kSender(String config_path)
      : sensesp::Saveable{config_path}, sensesp::Serializable() {}

  virtual void enable() = 0;

  void disable() {
    if (this->sender_event_ != nullptr) {
      sensesp::event_loop()->remove(this->sender_event_);
      this->sender_event_ = nullptr;
    }
  }

 protected:
  reactesp::RepeatEvent* sender_event_ = nullptr;
};

class N2kWindDataSender
    : public N2kSender,
      public sensesp::ValueProducer<std::pair<double, double>> {
 public:
  N2kWindDataSender(String config_path, tN2kWindReference wind_reference,
                    tNMEA2000* nmea2000, bool enable = true)
      : N2kSender{config_path},
        wind_reference_{wind_reference},
        nmea2000_{nmea2000},
        repeat_interval_{100},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{5000}           // In ms. When the inputs expire.
  {
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    if (this->sender_event_ == nullptr) {
      this->sender_event_ =
          sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
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

  sensesp::RepeatExpiring<double> wind_angle_{repeat_interval_, expiry_};
  sensesp::RepeatExpiring<double> wind_speed_{repeat_interval_, expiry_};

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  tN2kWindReference wind_reference_;
};

}  // namespace wind_interface

#endif  // AUTONNIC_N2K_SRC_SENDER_N2K_SENDERS_H_
