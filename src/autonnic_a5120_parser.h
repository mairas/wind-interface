#ifndef AUTONNIC_WIND_SRC_AUTONNIC_A5120_PARSER_H_
#define AUTONNIC_WIND_SRC_AUTONNIC_A5120_PARSER_H_

#include "sensesp.h"
#include "sensesp/system/valueconsumer.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

using namespace sensesp;

class AutonnicPATCWIMWVParser : public SentenceParser {
 public:
  AutonnicPATCWIMWVParser(NMEA0183 *nmea0183, ValueConsumer<bool> *result,
                          ValueConsumer<String> *result_string = nullptr)
      : SentenceParser(nmea0183),
        result_{result},
        result_string_{result_string} {}

  bool parse_fields(char *field_strings, int field_offsets[],
                    int num_fields) override final {
    // Example sentence: $PATC,WIMWV,ACK

    if (num_fields != 1) {
      return false;
    }

    if (strcmp(field_strings + field_offsets[0], "ACK") == 0) {
      set_consumer(result_, true);
    } else {
      set_consumer(result_, false);
    }
    set_consumer(result_string_, String(field_strings + field_offsets[0]));
  }

  virtual void emit(float wind_speed, float wind_angle) = 0;

 protected:
  ValueConsumer<bool> *result_;
  ValueConsumer<String> *result_string_;
};


#endif  // AUTONNIC_WIND_SRC_AUTONNIC_A5120_PARSER_H_
