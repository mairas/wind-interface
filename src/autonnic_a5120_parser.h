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

  bool parse_fields(const char *field_strings, const int field_offsets[],
                    int num_fields) override final {
    // Example sentence: $PATC,WIMWV,ACK

    for (int i = 0; i < num_fields; i++) {
      ESP_LOGD("AutonnicPATCWIMWVParser", "Field %d: %s", i,
               field_strings + field_offsets[i]);
    }


    if (num_fields != 1) {
      return false;
    }

    bool result;

    if (strcmp(field_strings + field_offsets[0], "ACK") == 0) {
      result = true;
    } else {
      result = false;
    }

    emit(result);
    ESP_LOGD("AutonnicPATCWIMWVParser", "Result: %d", result);

    return true;
  }

};

#endif  // AUTONNIC_WIND_SRC_AUTONNIC_A5120_PARSER_H_
