#pragma once

#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"
#include "esphome/core/hal.h"
#include "../elero.h" 

namespace esphome {
namespace elero {

class EleroCover : public cover::Cover, public Component {
 public:
  // --- ELERO STATUS CODES (Received from Blind) ---
  static const uint8_t ELERO_STATE_UNKNOWN = 0x00;
  static const uint8_t ELERO_STATE_TOP = 0x01;
  static const uint8_t ELERO_STATE_BOTTOM = 0x02;
  static const uint8_t ELERO_STATE_INTERMEDIATE = 0x03;
  static const uint8_t ELERO_STATE_TILT = 0x04;
  static const uint8_t ELERO_STATE_BLOCKING = 0x05;
  static const uint8_t ELERO_STATE_OVERHEATED = 0x06;
  static const uint8_t ELERO_STATE_TIMEOUT = 0x07;
  static const uint8_t ELERO_STATE_START_MOVING_UP = 0x08;
  static const uint8_t ELERO_STATE_START_MOVING_DOWN = 0x09;
  static const uint8_t ELERO_STATE_MOVING_UP = 0x0a;
  static const uint8_t ELERO_STATE_MOVING_DOWN = 0x0b;
  static const uint8_t ELERO_STATE_STOPPED = 0x0d;
  static const uint8_t ELERO_STATE_TOP_TILT = 0x0e;
  static const uint8_t ELERO_STATE_BOTTOM_TILT = 0x0f;
  static const uint8_t ELERO_STATE_OFF = 0x10;
  static const uint8_t ELERO_STATE_ON = 0x11;

  void setup() override;
  void loop() override;
  void dump_config() override;

  // Configuration setters
  void set_elero_parent(Elero *parent) { this->parent_ = parent; }
  void set_blind_address(uint32_t address) { this->blind_address_ = address; }
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  void set_remote_address(uint32_t address) { this->remote_address_ = address; }
  void set_open_duration(uint32_t duration) { this->open_duration_ = duration; }
  void set_close_duration(uint32_t duration) { this->close_duration_ = duration; }
  void set_tilt_duration(uint32_t duration) { this->tilt_duration_ = duration; }
  void set_payload_1(uint8_t payload) { this->payload_1_ = payload; }
  void set_payload_2(uint8_t payload) { this->payload_2_ = payload; }
  void set_pckinf_1(uint8_t pckinf) { this->pckinf_1_ = pckinf; }
  void set_pckinf_2(uint8_t pckinf) { this->pckinf_2_ = pckinf; }
  void set_hop(uint8_t hop) { this->hop_ = hop; }
  void set_command_up(uint8_t cmd) { this->command_up_ = cmd; }
  void set_command_down(uint8_t cmd) { this->command_down_ = cmd; }
  void set_command_stop(uint8_t cmd) { this->command_stop_ = cmd; }
  void set_command_check(uint8_t cmd) { this->command_check_ = cmd; }
  void set_command_tilt(uint8_t cmd) { this->command_tilt_ = cmd; }
  void set_poll_interval(uint32_t interval) { this->poll_interval_ = interval; }
  void set_supports_tilt(bool supports) { this->supports_tilt_ = supports; }

  // Methods required by parent Elero component
  uint32_t get_blind_address() const { return this->blind_address_; }
  uint32_t get_remote_address() const { return this->remote_address_; }
  uint8_t get_channel() const { return this->channel_; }
  
  void set_poll_offset(uint32_t offset) { this->poll_offset_ = offset; }
  void set_rx_state(uint8_t state); 

  // External event handler
  void on_command(uint8_t command, uint8_t channel, uint32_t blind_address);
  
  // Status Polling
  void poll_status(uint8_t retries);
  void start_poll_loop();
  void execute_poll_loop();
  void stop_poll_loop();

 protected:
  cover::CoverTraits get_traits() override;
  void control(const cover::CoverCall &call) override;

  // Helper functions
  void start_movement(cover::CoverOperation op);
  void stop_movement();
  void send_command(uint8_t command);
  void increase_counter();

  Elero *parent_{nullptr};
  uint32_t blind_address_{0};
  uint8_t channel_{1};
  uint32_t remote_address_{0};
  
  // Timing configuration
  uint32_t open_duration_{0};
  uint32_t close_duration_{0};
  uint32_t tilt_duration_{0};
  
  // Protocol details
  uint8_t payload_1_{0};
  uint8_t payload_2_{0};
  uint8_t pckinf_1_{0};
  uint8_t pckinf_2_{0};
  uint8_t hop_{0};
  uint8_t command_up_{0};
  uint8_t command_down_{0};
  uint8_t command_stop_{0};
  uint8_t command_check_{0};
  uint8_t command_tilt_{0};
  uint32_t poll_interval_{0};
  bool supports_tilt_{false};

  // Internal state
  uint32_t last_loop_time_{0};
  bool is_moving_{false};
  bool is_tilting_only_{false}; 
  uint8_t counter_{1};          
  uint32_t poll_offset_{0};     
  
  // Polling state
  uint32_t last_valid_status_time_{0};
  int poll_retries_left_{0};
  
  // Precise position tracking
  float exact_position_{0.0f};
  float exact_tilt_{0.0f};
};

} // namespace elero
} // namespace esphome