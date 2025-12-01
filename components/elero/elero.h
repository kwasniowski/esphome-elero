#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/elero/cc1101.h"
#include <map>
#include <string>
#include <atomic>

namespace esphome {
namespace elero {

// Protocol constants
static const uint8_t ELERO_MAX_PACKET_SIZE = 57;
static const uint32_t ELERO_POLL_INTERVAL_MOVING = 2000;
static const uint32_t ELERO_DELAY_SEND_PACKETS = 50;
static const uint32_t ELERO_TIMEOUT_MOVEMENT = 120000;
static const uint8_t ELERO_SEND_RETRIES = 3;
static const uint8_t ELERO_SEND_PACKETS = 2;

// Structure for command data
typedef struct {
  uint8_t counter;
  uint32_t blind_addr;
  uint32_t remote_addr;
  uint8_t channel;
  uint8_t pck_inf[2];
  uint8_t hop;
  uint8_t payload[2];
  uint8_t command;
} t_elero_command;

// Forward declaration
class EleroCover;

class Elero : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                    spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ>,
                                    public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Interrupt handler
  static void interrupt(Elero *arg);
  void set_received();

  // Low level CC1101 methods
  void reset();
  void init();
  void write_reg(uint8_t addr, uint8_t data);
  void write_burst(uint8_t addr, uint8_t *data, uint8_t len);
  void write_cmd(uint8_t cmd);
  bool wait_rx();
  bool wait_tx();
  bool wait_tx_done();
  bool wait_idle();
  bool transmit();
  uint8_t read_reg(uint8_t addr);
  uint8_t read_status(uint8_t addr);
  void read_buf(uint8_t addr, uint8_t *buf, uint8_t len);
  void flush_and_rx();

  // Protocol logic
  void interpret_msg();
  void register_cover(EleroCover *cover);
  bool send_command(t_elero_command *cmd);
  
  // New function: Close all and reset status
  void close_and_reset_all();

  // Configuration setters
  void set_gdo0_pin(InternalGPIOPin *pin) { gdo0_pin_ = pin; }
  void set_freq0(uint8_t freq) { freq0_ = freq; }
  void set_freq1(uint8_t freq) { freq1_ = freq; }
  void set_freq2(uint8_t freq) { freq2_ = freq; }

 private:
  // Encryption / Decryption helpers
  uint8_t count_bits(uint8_t byte);
  void calc_parity(uint8_t* msg);
  void add_r20_to_nibbles(uint8_t* msg, uint8_t r20, uint8_t start, uint8_t length);
  void sub_r20_from_nibbles(uint8_t* msg, uint8_t r20, uint8_t start, uint8_t length);
  void xor_2byte_in_array_encode(uint8_t* msg, uint8_t xor0, uint8_t xor1);
  void xor_2byte_in_array_decode(uint8_t* msg, uint8_t xor0, uint8_t xor1);
  void encode_nibbles(uint8_t* msg);
  void decode_nibbles(uint8_t* msg, uint8_t len);
  void msg_decode(uint8_t *msg);
  void msg_encode(uint8_t* msg);
  std::string resolve_addr(uint32_t addr) const;
 
  // Internal members
  std::atomic<bool> received_{false};
  uint8_t msg_rx_[CC1101_FIFO_LENGTH];
  uint8_t msg_tx_[CC1101_FIFO_LENGTH];
  uint8_t freq0_{0x7a};
  uint8_t freq1_{0x71};
  uint8_t freq2_{0x21};
  InternalGPIOPin *gdo0_pin_{nullptr};
  ISRInternalGPIOPin gdo0_irq_pin_{nullptr};
  
  // Map to store registered covers
  std::map<uint32_t, EleroCover*> address_to_cover_mapping_;
};

}  // namespace elero
}  // namespace esphome