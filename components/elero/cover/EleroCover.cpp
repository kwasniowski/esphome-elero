#include "EleroCover.h"
#include "esphome/core/log.h"
#include <cstring>

namespace esphome {
namespace elero {

static const char *const TAG = "elero.cover";

void EleroCover::setup() {
  this->position = 0.0f;
  this->tilt = 0.0f;
  this->exact_position_ = 0.0f;
  this->exact_tilt_ = 0.0f;
  this->counter_ = 1;

  if (this->parent_ != nullptr) {
    this->parent_->register_cover(this);
  } else {
    ESP_LOGE(TAG, "Parent Elero component is missing during setup!");
  }
}

void EleroCover::dump_config() {
  LOG_COVER("", "Elero Cover", this);
  ESP_LOGCONFIG(TAG, "  Blind Address: 0x%06X", this->blind_address_);
  ESP_LOGCONFIG(TAG, "  Channel: %d", this->channel_);
  ESP_LOGCONFIG(TAG, "  Remote Address: 0x%06X", this->remote_address_);
  ESP_LOGCONFIG(TAG, "  Open Duration: %.1fs", this->open_duration_ / 1000.0f);
  ESP_LOGCONFIG(TAG, "  Close Duration: %.1fs", this->close_duration_ / 1000.0f);
  ESP_LOGCONFIG(TAG, "  Tilt Duration: %.1fs", this->tilt_duration_ / 1000.0f);
  ESP_LOGCONFIG(TAG, "  Supports Tilt: %s", YESNO(this->supports_tilt_));
}

cover::CoverTraits EleroCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  // Assumed state allows buttons to be active even if HA thinks limit is reached
  traits.set_is_assumed_state(true);
  
  if (this->supports_tilt_) {
    traits.set_supports_tilt(true);
  }
  return traits;
}

void EleroCover::loop() {
  if (!this->is_moving_) {
    return;
  }

  uint32_t now = millis();
  if (this->last_loop_time_ == 0) {
    this->last_loop_time_ = now;
    return;
  }

  uint32_t dt = now - this->last_loop_time_;
  this->last_loop_time_ = now;

  uint32_t travel_time_up = (this->open_duration_ > this->tilt_duration_) 
                            ? (this->open_duration_ - this->tilt_duration_) : 1;
  uint32_t travel_time_down = (this->close_duration_ > this->tilt_duration_) 
                              ? (this->close_duration_ - this->tilt_duration_) : 1;

  float tilt_step = (this->tilt_duration_ > 0) ? ((float)dt / (float)this->tilt_duration_) : 1.0f;
  float pos_step_up = (float)dt / (float)travel_time_up;
  float pos_step_down = (float)dt / (float)travel_time_down;

  if (this->current_operation == cover::COVER_OPERATION_OPENING) {
    if (this->exact_tilt_ < 1.0f) {
      this->exact_tilt_ += tilt_step;
      if (this->exact_tilt_ > 1.0f) this->exact_tilt_ = 1.0f;
    } 
    else if (!this->is_tilting_only_) {
      this->exact_position_ += pos_step_up;
      if (this->exact_position_ > 1.0f) this->exact_position_ = 1.0f;
    }
  } 
  else if (this->current_operation == cover::COVER_OPERATION_CLOSING) {
    if (this->exact_tilt_ > 0.0f) {
      this->exact_tilt_ -= tilt_step;
      if (this->exact_tilt_ < 0.0f) this->exact_tilt_ = 0.0f;
    } 
    else if (!this->is_tilting_only_) {
      this->exact_position_ -= pos_step_down;
      if (this->exact_position_ < 0.0f) this->exact_position_ = 0.0f;
    }
  }

  this->position = this->exact_position_;
  this->tilt = this->exact_tilt_;
  this->publish_state();

  if (!this->is_tilting_only_) {
    if (this->current_operation == cover::COVER_OPERATION_OPENING && 
        this->position >= 1.0f && this->tilt >= 1.0f) {
      this->stop_movement();
    }
    else if (this->current_operation == cover::COVER_OPERATION_CLOSING && 
             this->position <= 0.0f && this->tilt <= 0.0f) {
      this->stop_movement();
    }
  }
}

void EleroCover::control(const cover::CoverCall &call) {
  this->is_tilting_only_ = false;

  // 1. STOP
  if (call.get_stop()) {
    this->send_command(this->command_stop_);
    this->stop_movement();
    this->start_poll_loop(); 
    return;
  }

  // 2. POSITION
  if (call.get_position().has_value()) {
    float target_pos = *call.get_position();
    this->cancel_timeout("stop_timer");
    this->cancel_interval("check_pos");

    this->is_tilting_only_ = false;

    // FIX: Force movement if target is at limits (0.0 or 1.0)
    // This allows re-syncing if HA thinks we are at limit but we are not.
    bool force_up = (target_pos == 1.0f);
    bool force_down = (target_pos == 0.0f);

    if (force_up || target_pos > this->exact_position_ + 0.01f) {
      this->send_command(this->command_up_);
      this->start_movement(cover::COVER_OPERATION_OPENING);
      
      this->set_interval("check_pos", 100, [this, target_pos]() {
        if (this->exact_position_ >= target_pos) {
          this->send_command(this->command_stop_);
          this->stop_movement();
          this->cancel_interval("check_pos");
        }
      });
    } 
    else if (force_down || target_pos < this->exact_position_ - 0.01f) {
      this->send_command(this->command_down_);
      this->start_movement(cover::COVER_OPERATION_CLOSING);
      
      this->set_interval("check_pos", 100, [this, target_pos]() {
        if (this->exact_position_ <= target_pos) {
          this->send_command(this->command_stop_);
          this->stop_movement();
          this->cancel_interval("check_pos");
        }
      });
    }
    
    this->start_poll_loop(); 
    return;
  }

  // 3. TILT
  if (call.get_tilt().has_value() && !call.get_position().has_value()) {
    float target_tilt = *call.get_tilt();
    float diff = target_tilt - this->exact_tilt_;

    if (std::abs(diff) > 0.05f) {
      this->is_tilting_only_ = true;
      uint32_t move_time = (uint32_t)(std::abs(diff) * this->tilt_duration_);
      
      if (diff > 0) {
        this->send_command(this->command_up_);
        this->start_movement(cover::COVER_OPERATION_OPENING);
      } else {
        this->send_command(this->command_down_);
        this->start_movement(cover::COVER_OPERATION_CLOSING);
      }

      this->set_timeout("stop_timer", move_time, [this]() {
        this->send_command(this->command_stop_);
        this->stop_movement();
      });
      
      this->start_poll_loop();
    }
  }
}

// --- POLLING LOGIC ---

void EleroCover::poll_status(uint8_t retries) {
  this->start_poll_loop();
}

void EleroCover::start_poll_loop() {
  // Try for approx 90 seconds (45 * 2s)
  this->poll_retries_left_ = 45;
  this->cancel_timeout("poll_loop");
  
  // Start first poll after 1s delay
  this->set_timeout("poll_loop", 1000, [this]() {
    this->execute_poll_loop();
  });
}

void EleroCover::execute_poll_loop() {
  if (this->poll_retries_left_ <= 0) {
    ESP_LOGD(TAG, "Polling timeout reached, stopping.");
    return;
  }

  ESP_LOGD(TAG, "Polling blind 0x%06X, retries left: %d", this->blind_address_, this->poll_retries_left_);
  
  // Manually construct a clean CHECK command (payload 0x00 0x00)
  if (this->parent_ != nullptr) {
    this->increase_counter();
    t_elero_command cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.payload[0] = 0x00; // CLEAN PAYLOAD FOR CHECK
    cmd.payload[1] = 0x00; // CLEAN PAYLOAD FOR CHECK
    cmd.pck_inf[0] = this->pckinf_1_;
    cmd.pck_inf[1] = this->pckinf_2_;
    cmd.hop = this->hop_;
    cmd.channel = this->channel_;
    cmd.remote_addr = this->remote_address_;
    cmd.blind_addr = this->blind_address_;
    cmd.command = this->command_check_;
    cmd.counter = this->counter_;
    this->parent_->send_command(&cmd);
  }

  this->poll_retries_left_--;

  // Schedule next poll in 2 seconds
  this->set_timeout("poll_loop", 2000, [this]() {
    this->execute_poll_loop();
  });
}

void EleroCover::stop_poll_loop() {
  ESP_LOGD(TAG, "Stopping poll loop for blind 0x%06X", this->blind_address_);
  this->cancel_timeout("poll_loop");
  this->poll_retries_left_ = 0;
}

void EleroCover::on_command(uint8_t command, uint8_t channel, uint32_t blind_address) {
  if (this->channel_ != channel) return;
  if (this->blind_address_ != blind_address) return;

  if (this->is_tilting_only_) {
    if (command == this->command_up_ && this->current_operation == cover::COVER_OPERATION_OPENING) return;
    if (command == this->command_down_ && this->current_operation == cover::COVER_OPERATION_CLOSING) return;
  }

  this->cancel_timeout("stop_timer");
  this->cancel_interval("check_pos");
  this->is_tilting_only_ = false;

  if (command == this->command_up_) {
    this->start_movement(cover::COVER_OPERATION_OPENING);
    this->start_poll_loop();
  } 
  else if (command == this->command_down_) {
    this->start_movement(cover::COVER_OPERATION_CLOSING);
    this->start_poll_loop();
  } 
  else if (command == this->command_stop_) {
    this->stop_movement();
    this->start_poll_loop();
  }
}

void EleroCover::set_rx_state(uint8_t state) {
  ESP_LOGD(TAG, "Received status byte: 0x%02X for blind 0x%06X", state, this->blind_address_);
  this->last_valid_status_time_ = millis();

  switch (state) {
    case ELERO_STATE_TOP: // 0x01
      ESP_LOGD(TAG, "Status: Top Limit Reached");
      this->exact_position_ = 1.0f;
      this->exact_tilt_ = 1.0f;
      this->stop_movement();
      this->stop_poll_loop();
      break;

    case ELERO_STATE_BOTTOM: // 0x02
      ESP_LOGD(TAG, "Status: Bottom Limit Reached");
      this->exact_position_ = 0.0f;
      this->exact_tilt_ = 0.0f;
      this->stop_movement();
      this->stop_poll_loop();
      break;

    case ELERO_STATE_STOPPED: // 0x0D
    case ELERO_STATE_INTERMEDIATE: // 0x03
    case ELERO_STATE_TILT: // 0x04
    case ELERO_STATE_TOP_TILT: // 0x0E
    case ELERO_STATE_BOTTOM_TILT: // 0x0F
    case ELERO_STATE_BLOCKING: // 0x05
    case ELERO_STATE_OVERHEATED: // 0x06
    case ELERO_STATE_TIMEOUT: // 0x07
    default:
      // For any other state (Stopped, Error, Unknown), assume we stopped.
      ESP_LOGD(TAG, "Status: Stopped/Other (0x%02X)", state);
      this->stop_movement();
      this->stop_poll_loop();
      break;

    case ELERO_STATE_START_MOVING_UP: // 0x08
    case ELERO_STATE_MOVING_UP: // 0x0A
      ESP_LOGD(TAG, "Status: Moving UP");
      if (!this->is_moving_ || this->current_operation != cover::COVER_OPERATION_OPENING) {
        this->start_movement(cover::COVER_OPERATION_OPENING);
      }
      // Continue polling to catch when it stops
      break;

    case ELERO_STATE_START_MOVING_DOWN: // 0x09
    case ELERO_STATE_MOVING_DOWN: // 0x0B
      ESP_LOGD(TAG, "Status: Moving DOWN");
      if (!this->is_moving_ || this->current_operation != cover::COVER_OPERATION_CLOSING) {
        this->start_movement(cover::COVER_OPERATION_CLOSING);
      }
      // Continue polling to catch when it stops
      break;
  }
  
  this->position = this->exact_position_;
  this->tilt = this->exact_tilt_;
  this->publish_state();
}

void EleroCover::start_movement(cover::CoverOperation op) {
  this->current_operation = op;
  this->is_moving_ = true;
  this->last_loop_time_ = millis();
  this->publish_state();
}

void EleroCover::stop_movement() {
  this->is_moving_ = false;
  this->is_tilting_only_ = false;
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->last_loop_time_ = 0;
  
  if (this->exact_tilt_ > 1.0f) this->exact_tilt_ = 1.0f;
  if (this->exact_tilt_ < 0.0f) this->exact_tilt_ = 0.0f;
  if (this->exact_position_ > 1.0f) this->exact_position_ = 1.0f;
  if (this->exact_position_ < 0.0f) this->exact_position_ = 0.0f;

  this->position = this->exact_position_;
  this->tilt = this->exact_tilt_;
  this->publish_state();
}

void EleroCover::increase_counter() {
  if (this->counter_ == 0xFF) {
    this->counter_ = 1;
  } else {
    this->counter_++;
  }
}

void EleroCover::send_command(uint8_t command) {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Parent Elero component is missing!");
    return;
  }

  this->increase_counter();

  t_elero_command cmd;
  memset(&cmd, 0, sizeof(cmd));

  cmd.payload[0] = this->payload_1_;
  cmd.payload[1] = this->payload_2_;
  cmd.pck_inf[0] = this->pckinf_1_;
  cmd.pck_inf[1] = this->pckinf_2_;
  cmd.hop = this->hop_;
  cmd.channel = this->channel_;
  cmd.remote_addr = this->remote_address_;
  cmd.blind_addr = this->blind_address_;
  cmd.command = command;
  cmd.counter = this->counter_;

  ESP_LOGD(TAG, "Sending command: 0x%02X to blind: 0x%06X, Counter: %d", command, this->blind_address_, this->counter_);
  this->parent_->send_command(&cmd);
}

} // namespace elero
} // namespace esphome