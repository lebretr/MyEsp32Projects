// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Class of Zigbee Binary sensor endpoint inherited from common EP class */

#pragma once

#include "soc/soc_caps.h"
#include "sdkconfig.h"
#if CONFIG_ZB_ENABLED

#include "ZigbeeEP.h"
#include "ha/esp_zigbee_ha_standard.h"

//enum for bits set to check what analog cluster were added
enum zigbee_number_clusters {
  NUMBER_INPUT = 1,
  NUMBER_OUTPUT = 2
};

// typedef struct zigbee_number_cfg_s {
//   esp_zb_basic_cluster_cfg_t basic_cfg;
//   esp_zb_identify_cluster_cfg_t identify_cfg;
//   esp_zb_number_output_cluster_cfg_t number_output_cfg;
//   esp_zb_number_input_cluster_cfg_t number_input_cfg;
// } zigbee_number_cfg_t;

class ZigbeeNumber : public ZigbeeEP {
public:
  ZigbeeNumber(uint8_t endpoint);
  ~ZigbeeNumber() {}

  // Add binary cluster
  bool addBinaryInput();
  // bool addBinaryOutput();

  // Set the application type and description for the binary input
  bool setBinaryInputApplication(uint32_t application_type);  // Check esp_zigbee_zcl_binary_input.h for application type values
  bool setBinaryInputDescription(const char *description);

  // Set the application type and description for the binary output
  // bool setBinaryOutputApplication(uint32_t application_type);  // Check esp_zigbee_zcl_binary_output.h for application type values
  // bool setBinaryOutputDescription(const char *description);

  // Use to set a cb function to be called on binary output change
  // void onBinaryOutputChange(void (*callback)(bool binary_output)) {
  //   _on_binary_output_change = callback;
  // }

  // Set the binary input/output value
  bool setBinaryInput(bool input);
  // bool setBinaryOutput(bool output);

  // Get the Binary Output value
  // bool getBinaryOutput() {
  //   return _output_state;
  // }

  // Report Binary Input/Output value
  bool reportBinaryInput();
  // bool reportBinaryOutput();

private:
  // void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) override;

  // void (*_on_binary_output_change)(bool);
  // void binaryOutputChanged();

  // bool _output_state;
  uint8_t _binary_clusters;
};

#endif  // CONFIG_ZB_ENABLED