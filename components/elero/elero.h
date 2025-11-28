#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/elero/cc1101.h"

// All encryption/decryption structures copied from https://github.com/QuadCorei8085/elero_protocol/ (MIT)
// All remote handling based on code from https://github.com/stanleypa/eleropy (GPLv3)

namespace esphome {
namespace elero {

static const uint8_t ELERO_MAX_PACKET_SIZE = 57; // according to FCC documents

static const uint32_t ELERO_POLL_INTERVAL_MOVING = 2000;  // poll every two seconds while moving
static const uint32_t ELERO_DELAY_SEND_PACKETS = 50; // 50ms send delay between repeats
static const uint32_t ELERO_TIMEOUT_MOVEMENT = 120000; // poll for up to two minutes while moving

static const uint8_t ELERO_SEND_RETRIES = 3;
static const uint8_t ELERO_SEND_PACKETS = 2;

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

class EleroCover;

class Elero : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                    spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ>,
                                    public Component {
 public:
  void setup() override;
  void loop() override;

  static void interrupt(Elero *arg);
  void set_received();
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
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
  void interpret_msg();
  void register_cover(EleroCover *cover);
  bool send_command(t_elero_command *cmd);
  
  void set_gdo0_pin(InternalGPIOPin *pin) { gdo0_pin_ = pin; }
  void set_freq0(uint8_t freq) { freq0_ = freq; }
  void set_freq1(uint8_t freq) { freq1_ = freq; }
  void set_freq2(uint8_t freq) { freq2_ = freq; }

 private:
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
 
  std::atomic<bool> received_{false};
  uint8_t msg_rx_[CC1101_FIFO_LENGTH];
  uint8_t msg_tx_[CC1101_FIFO_LENGTH];
  uint8_t freq0_{0x7a};
  uint8_t freq1_{0x71};
  uint8_t freq2_{0x21};
  InternalGPIOPin *gdo0_pin_{nullptr};
  ISRInternalGPIOPin gdo0_irq_pin_{nullptr};
  std::map<uint32_t, EleroCover*> address_to_cover_mapping_;
};

}  // namespace elero
}  // namespace esphome

