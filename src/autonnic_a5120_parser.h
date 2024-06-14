#ifndef AUTONNIC_WIND_SRC_AUTONNIC_A5120_PARSER_H_
#define AUTONNIC_WIND_SRC_AUTONNIC_A5120_PARSER_H_

#include "sensesp.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/system/valueconsumer.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

using namespace sensesp;

class AutonnicPATCWIMWVParser : public SentenceParser {
 public:
  AutonnicPATCWIMWVParser(NMEA0183 *nmea0183) : SentenceParser(nmea0183) {
    ignore_checksum(true);
  }

  virtual const char *sentence_address() override final { return "PATC,WIMWV"; }

  bool parse_fields(char *field_strings, int field_offsets[],
                    int num_fields) override final {
    // Example sentence: $PATC,WIMWV,ACK

    for (int i = 0; i < num_fields; i++) {
      ESP_LOGD("AutonnicPATCWIMWVParser", "Field %d: %s", i,
               field_strings + field_offsets[i]);
    }


    if (num_fields != 1) {
      return false;
    }

    if (strcmp(field_strings + field_offsets[0], "ACK") == 0) {
      result_.set(true);
    } else {
      result_.set(false);
    }
    result_string_.set(String(field_strings + field_offsets[0]));

    ESP_LOGD("AutonnicPATCWIMWVParser", "Result: %d", result_.get());

    return true;
  }

  ObservableValue<bool> result_;
  ObservableValue<String> result_string_;
};

#endif  // AUTONNIC_WIND_SRC_AUTONNIC_A5120_PARSER_H_
