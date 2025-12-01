#include "elero.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/components/elero/cover/EleroCover.h"

namespace esphome {
namespace elero {

static const char *TAG = "elero";
static const uint8_t flash_table_encode[] = {0x08, 0x02, 0x0d, 0x01, 0x0f, 0x0e, 0x07, 0x05, 0x09, 0x0c, 0x00, 0x0a, 0x03, 0x04, 0x0b, 0x06};
static const uint8_t flash_table_decode[] = {0x0a, 0x03, 0x01, 0x0c, 0x0d, 0x07, 0x0f, 0x06, 0x00, 0x08, 0x0b, 0x0e, 0x09, 0x02, 0x05, 0x04};

void Elero::loop() {
  if (this->received_.exchange(false)) {
    uint8_t len_status = this->read_status(CC1101_RXBYTES);
    uint8_t bytes_available = len_status & 0x7F;
    
    if (bytes_available > 0) {
      if (bytes_available > CC1101_FIFO_LENGTH) {
        ESP_LOGW(TAG, "RX FIFO overflow detected (%d bytes), flushing.", bytes_available);
        this->flush_and_rx();
        return;
      }

      this->read_buf(CC1101_RXFIFO, this->msg_rx_, bytes_available);

      if (this->msg_rx_[0] + 3 <= bytes_available) {
        this->interpret_msg();
      } else {
        ESP_LOGV(TAG, "Incomplete packet received.");
      }
    }

    if (len_status & 0x80) {
      ESP_LOGV(TAG, "CC1101 RX Overflow, flushing.");
      this->flush_and_rx();
    }
  }
}

IRAM_ATTR void Elero::interrupt(Elero *arg) {
  arg->set_received();
}

IRAM_ATTR void Elero::set_received() {
  this->received_ = true;
}

void Elero::dump_config() {
  ESP_LOGCONFIG(TAG, "Elero Config:");
  ESP_LOGCONFIG(TAG, "  Freq0: 0x%02X", this->freq0_);
  ESP_LOGCONFIG(TAG, "  Freq1: 0x%02X", this->freq1_);
  ESP_LOGCONFIG(TAG, "  Freq2: 0x%02X", this->freq2_);
}

void Elero::setup() {
  ESP_LOGI(TAG, "Setting up Elero Component...");
  this->spi_setup();
  this->gdo0_pin_->setup();
  this->gdo0_irq_pin_ = this->gdo0_pin_->to_isr();
  this->gdo0_pin_->attach_interrupt(Elero::interrupt, this, gpio::INTERRUPT_FALLING_EDGE);
  this->reset();
  this->init();
}

void Elero::flush_and_rx() {
  this->write_cmd(CC1101_SIDLE);
  this->wait_idle();
  this->write_cmd(CC1101_SFRX);
  this->write_cmd(CC1101_SFTX);
  this->write_cmd(CC1101_SRX);
  this->received_ = false;
}

void Elero::reset() {
  this->enable();
  this->write_byte(CC1101_SRES);
  delay_microseconds_safe(50);
  this->write_byte(CC1101_SIDLE);
  delay_microseconds_safe(50);
  this->disable();
}

void Elero::init() {
  uint8_t patable_data[] = {0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0};

  this->write_reg(CC1101_FSCTRL1, 0x08);
  this->write_reg(CC1101_FSCTRL0, 0x00);
  this->write_reg(CC1101_FREQ2, this->freq2_);
  this->write_reg(CC1101_FREQ1, this->freq1_);
  this->write_reg(CC1101_FREQ0, this->freq0_);
  this->write_reg(CC1101_MDMCFG4, 0x7B);
  this->write_reg(CC1101_MDMCFG3, 0x83);
  this->write_reg(CC1101_MDMCFG2, 0x13); 
  this->write_reg(CC1101_MDMCFG1, 0x52);
  this->write_reg(CC1101_MDMCFG0, 0xF8);
  this->write_reg(CC1101_CHANNR, 0x00);
  this->write_reg(CC1101_DEVIATN, 0x43);
  this->write_reg(CC1101_FREND1, 0xB6);
  this->write_reg(CC1101_FREND0, 0x10);
  this->write_reg(CC1101_MCSM0, 0x18);
  this->write_reg(CC1101_MCSM1, 0x3F);
  this->write_reg(CC1101_FOCCFG, 0x1D);
  this->write_reg(CC1101_BSCFG, 0x1F);
  this->write_reg(CC1101_AGCCTRL2, 0xC7);
  this->write_reg(CC1101_AGCCTRL1, 0x00);
  this->write_reg(CC1101_AGCCTRL0, 0xB2);
  this->write_reg(CC1101_FSCAL3, 0xEA);
  this->write_reg(CC1101_FSCAL2, 0x2A);
  this->write_reg(CC1101_FSCAL1, 0x00);
  this->write_reg(CC1101_FSCAL0, 0x1F);
  this->write_reg(CC1101_FSTEST, 0x59);
  this->write_reg(CC1101_TEST2, 0x81);
  this->write_reg(CC1101_TEST1, 0x35);
  this->write_reg(CC1101_TEST0, 0x09);
  this->write_reg(CC1101_IOCFG0, 0x06);
  this->write_reg(CC1101_PKTCTRL1, 0x8C);  
  this->write_reg(CC1101_PKTCTRL0, 0x45);
  this->write_reg(CC1101_ADDR, 0x00);
  this->write_reg(CC1101_PKTLEN, 0x3C);
  this->write_reg(CC1101_SYNC1, 0xD3);
  this->write_reg(CC1101_SYNC0, 0x91);
  this->write_burst(CC1101_PATABLE, patable_data, 8);

  this->write_cmd(CC1101_SRX);
  this->wait_rx();
}

void Elero::write_reg(uint8_t addr, uint8_t data) {
  this->enable();
  this->write_byte(addr);
  this->write_byte(data);
  this->disable();
  delay_microseconds_safe(15);
}

void Elero::write_burst(uint8_t addr, uint8_t *data, uint8_t len) {
  this->enable();
  this->write_byte(addr | CC1101_WRITE_BURST);
  for(int i=0; i<len; i++)
    this->write_byte(data[i]);
  this->disable();
  delay_microseconds_safe(15);
}

void Elero::write_cmd(uint8_t cmd) {
  this->enable();
  this->write_byte(cmd);
  this->disable();
  delay_microseconds_safe(15);
}

bool Elero::wait_rx() {
  uint8_t timeout = 200;
  while ((this->read_status(CC1101_MARCSTATE) != CC1101_MARCSTATE_RX) && (--timeout != 0)) {
    delay_microseconds_safe(200);
  }
  if(timeout > 0) return true;
  ESP_LOGE(TAG, "Timed out waiting for RX");
  return false;
}

bool Elero::wait_idle() {
  uint8_t timeout = 200;
  while ((this->read_status(CC1101_MARCSTATE) != CC1101_MARCSTATE_IDLE) && (--timeout != 0)) {
    delay_microseconds_safe(200);
  }
  if(timeout > 0) return true;
  ESP_LOGE(TAG, "Timed out waiting for Idle");
  return false;
}

bool Elero::wait_tx() {
  uint8_t timeout = 200;
  while ((this->read_status(CC1101_MARCSTATE) != CC1101_MARCSTATE_TX) && (--timeout != 0)) {
    delay_microseconds_safe(200);
  }
  if(timeout > 0) return true;
  ESP_LOGE(TAG, "Timed out waiting for TX");
  return false;
}

bool Elero::wait_tx_done() {
  uint8_t timeout = 200;
  while((!this->received_) && (--timeout != 0)) {
    delay_microseconds_safe(200);
  }
  if(timeout > 0) return true;
  ESP_LOGE(TAG, "Timed out waiting for TX Done");
  return false;
}

bool Elero::transmit() {
  this->write_cmd(CC1101_SRX);
  if(!this->wait_rx()) return false;

  this->write_burst(CC1101_TXFIFO, this->msg_tx_, this->msg_tx_[0] + 1);
  this->write_cmd(CC1101_STX);

  if(!this->wait_tx()) {
    this->flush_and_rx();
    return false;
  }
  if(!this->wait_tx_done()) {
    this->flush_and_rx();
    return false;
  }

  uint8_t bytes = this->read_status(CC1101_TXBYTES) & 0x7f;
  if(bytes != 0) {
    ESP_LOGE(TAG, "Error transferring, %d bytes left in buffer", bytes);
    this->flush_and_rx();
    return false;
  }
  return true;
}

uint8_t Elero::read_reg(uint8_t addr) {
  uint8_t data;
  this->enable();
  this->write_byte(addr);
  data = this->read_byte();
  return data;
}

uint8_t Elero::read_status(uint8_t addr) {
  uint8_t data;
  this->enable();
  this->write_byte(addr | CC1101_READ_BURST);
  data = this->read_byte();
  this->disable();
  delay_microseconds_safe(15);
  return data;
}

void Elero::read_buf(uint8_t addr, uint8_t *buf, uint8_t len) {
  this->enable();
  this->write_byte(addr | CC1101_READ_BURST);
  for(uint8_t i=0; i<len; i++)
    buf[i] = this->read_byte();
  this->disable();
  delay_microseconds_safe(15);
}

// --- Encryption / Decryption Logic ---
uint8_t Elero::count_bits(uint8_t byte) {
  uint8_t ones = 0;
  uint8_t mask = 1;
  for(int i = 0; i < 8; i++) {
    if(mask & byte) ones += 1;
    mask <<= 1;
  }
  return ones & 0x01;
}

void Elero::calc_parity(uint8_t* msg) {
  uint8_t p = 0;
  for(int i = 0; i < 4; i++) {
    uint8_t a = count_bits(msg[0 + i*2]);
    uint8_t b = count_bits(msg[1 + i*2]);
    p |= a ^ b;
    p <<= 1;
  }
  msg[7] = (p << 3);
}

void Elero::add_r20_to_nibbles(uint8_t* msg, uint8_t r20, uint8_t start, uint8_t length) {
  for(int i = 0; i < 8; i++) {
    uint8_t d = msg[i];
    uint8_t ln = (d + r20) & 0x0F;
    uint8_t hn = ((d & 0xF0) + (r20 & 0xF0)) & 0xFF;
    msg[i] = hn | ln;
    r20 = (r20 - 0x22) & 0xFF;
  }
}

void Elero::sub_r20_from_nibbles(uint8_t* msg, uint8_t r20, uint8_t start, uint8_t length) {
  for(int i = start; i < length; i++) {
    uint8_t d = msg[i];
    uint8_t ln = (d - r20) & 0x0F;
    uint8_t hn = ((d & 0xF0) - (r20 & 0xF0)) & 0xFF;
    msg[i] = hn | ln;
    r20 = (r20 - 0x22) & 0xFF;
  }
}

void Elero::xor_2byte_in_array_encode(uint8_t* msg, uint8_t xor0, uint8_t xor1) {
  for(int i = 1; i < 4; i++) {
    msg[i*2 + 0] = msg[i*2 + 0] ^ xor0;
    msg[i*2 + 1] = msg[i*2 + 1] ^ xor1;
  }
}

void Elero::xor_2byte_in_array_decode(uint8_t* msg, uint8_t xor0, uint8_t xor1) {
  for(int i = 0; i < 4; i++) {
    msg[i*2 + 0] = msg[i*2 + 0] ^ xor0;
    msg[i*2 + 1] = msg[i*2 + 1] ^ xor1;
  }
}

void Elero::encode_nibbles(uint8_t* msg) {
  for(int i = 0; i < 8; i++) {
    uint8_t nh = (msg[i] >> 4) & 0x0F;
    uint8_t nl = msg[i] & 0x0F;
    uint8_t dh = flash_table_encode[nh];
    uint8_t dl = flash_table_encode[nl];
    msg[i] = ((dh << 4) & 0xFF) | ((dl) & 0xFF);
  }
}

void Elero::decode_nibbles(uint8_t* msg, uint8_t len) {
  for(int i = 0; i < len; i++) {
    uint8_t nh = (msg[i] >> 4) & 0x0F;
    uint8_t nl = msg[i] & 0x0F;
    uint8_t dh = flash_table_decode[nh];
    uint8_t dl = flash_table_decode[nl];
    msg[i] = ((dh << 4) & 0xFF) | ((dl) & 0xFF);
  }
}

void Elero::msg_decode(uint8_t *msg) {
  decode_nibbles(msg, 8);
  sub_r20_from_nibbles(msg, 0xFE, 0, 2);
  xor_2byte_in_array_decode(msg, msg[0], msg[1]);
  sub_r20_from_nibbles(msg, 0xBA, 2, 8);
}

void Elero::msg_encode(uint8_t* msg) {
  uint8_t xor0 = msg[0];
  uint8_t xor1 = msg[1];
  calc_parity(msg);
  add_r20_to_nibbles(msg, 0xFE, 0, 8);
  xor_2byte_in_array_encode(msg, xor0, xor1);
  encode_nibbles(msg);
}

std::string Elero::resolve_addr(uint32_t addr) const {
  if (auto it{address_to_cover_mapping_.find(addr)}; it != address_to_cover_mapping_.cend()) {
    return it->second->get_name();
  }
  char buff[10];
  snprintf(buff, sizeof(buff), "0x%06X", addr);
  return std::string(buff);
}

void Elero::interpret_msg() {
  uint8_t length = this->msg_rx_[0];
  if(length > ELERO_MAX_PACKET_SIZE) return;

  uint8_t cnt = this->msg_rx_[1];
  uint8_t typ = this->msg_rx_[2];
  uint8_t hop = this->msg_rx_[4];
  uint8_t syst = this->msg_rx_[5];
  uint8_t chl = this->msg_rx_[6];
  uint32_t src = ((uint32_t)this->msg_rx_[7] << 16) | ((uint32_t)this->msg_rx_[8] << 8) | (this->msg_rx_[9]);
  uint32_t bwd = ((uint32_t)this->msg_rx_[10] << 16) | ((uint32_t)this->msg_rx_[11] << 8) | (this->msg_rx_[12]);
  uint32_t fwd = ((uint32_t)this->msg_rx_[13] << 16) | ((uint32_t)this->msg_rx_[14] << 8) | (this->msg_rx_[15]);
  uint32_t dst = (typ > 0x60) ? ((uint32_t)this->msg_rx_[17] << 16) | ((uint32_t)this->msg_rx_[18] << 8) | (this->msg_rx_[19]) : this->msg_rx_[17];
  uint8_t dests_len = (typ > 0x60) ? this->msg_rx_[16] * 3 : this->msg_rx_[16];
  if(dests_len + 15 > CC1101_FIFO_LENGTH) return;

  uint8_t crc = this->msg_rx_[length + 2] >> 7;
  uint8_t lqi = this->msg_rx_[length + 2] & 0x7f;
  float rssi;
  if(this->msg_rx_[length+1] > 127)
    rssi = (float)((this->msg_rx_[length+1]-256)/2-74);
  else
    rssi = (float)((this->msg_rx_[length+1])/2-74);

  uint8_t payload1 = this->msg_rx_[17 + dests_len];
  uint8_t payload2 = this->msg_rx_[18 + dests_len];

  uint8_t *payload = &this->msg_rx_[19 + dests_len];
  msg_decode(payload);

  ESP_LOGD(TAG, "Msg: Len=%d Cnt=%d Typ=0x%02X Src=0x%06X Bwd=%s Fwd=%s Hop=0x%02X Syst=0x%02x Channel=%02d #Dest=%02d Dest=%s rssi=%2.1f, lqi=%2d, crc=%2d Payload=[0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]", 
    length, cnt, typ, src, resolve_addr(bwd).c_str(), resolve_addr(fwd).c_str(), 
    hop, syst, chl, dests_len, resolve_addr(dst).c_str(), rssi, lqi, crc,
    payload1, payload2, payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7]
  );

  // 1. Handle Status Messages (Blind -> ESP)
  auto search = this->address_to_cover_mapping_.find(src);
  if(search != this->address_to_cover_mapping_.end()) {
    // Status byte is usually at payload[6] for status messages
    // 0xCA = Status Message, 0xC9 = Info/Response
    if (typ == 0xCA || typ == 0xC9) {
        ESP_LOGI(TAG, "External update for blind: %s, Status: 0x%02X", search->second->get_name().c_str(), payload[6]);
        search->second->set_rx_state(payload[6]);
    }
  } 
  // 2. Handle Remote Commands (Remote Spy: Remote -> Blind)
  else {
    bool remote_found = false;
    for (auto const& [addr, cover] : this->address_to_cover_mapping_) {
        // Check if SRC matches the remote address configured for this blind
        if (cover->get_remote_address() == src) {
            // Check if CHANNEL matches (or if message is broadcast channel 0)
            if (cover->get_channel() == chl || chl == 0) {
                ESP_LOGI(TAG, "Spy: Remote 0x%06X (Ch %d) command detected for blind %s (Ch %d). Polling...", 
                         src, chl, cover->get_name().c_str(), cover->get_channel());
                cover->poll_status(3);
                remote_found = true;
            }
        }
    }
    
    if (!remote_found) {
         ESP_LOGD(TAG, "Src=0x%06X (Ch %d) not registered as blind or matching remote, ignoring.", src, chl);
    }
  }
}

void Elero::register_cover(EleroCover *cover) {
  uint32_t address = cover->get_blind_address();
  if(this->address_to_cover_mapping_.find(address) != this->address_to_cover_mapping_.end()) {
    ESP_LOGE(TAG, "Blind 0x%06X already registered!", address);
    return;
  }
  this->address_to_cover_mapping_.insert({address, cover});
  cover->set_poll_offset((this->address_to_cover_mapping_.size() - 1) * 5000);
}

bool Elero::send_command(t_elero_command *cmd) {
  uint16_t code = (0x00 - (cmd->counter * 0x708f)) & 0xffff;
  this->msg_tx_[0] = 0x1d; 
  this->msg_tx_[1] = cmd->counter;
  this->msg_tx_[2] = cmd->pck_inf[0];
  this->msg_tx_[3] = cmd->pck_inf[1];
  this->msg_tx_[4] = cmd->hop;
  this->msg_tx_[5] = 0x01;
  this->msg_tx_[6] = cmd->channel;
  this->msg_tx_[7] = ((cmd->remote_addr >> 16) & 0xff);
  this->msg_tx_[8] = ((cmd->remote_addr >> 8) & 0xff);
  this->msg_tx_[9] = ((cmd->remote_addr) & 0xff);
  this->msg_tx_[10] = ((cmd->remote_addr >> 16) & 0xff);
  this->msg_tx_[11] = ((cmd->remote_addr >> 8) & 0xff);
  this->msg_tx_[12] = ((cmd->remote_addr) & 0xff);
  this->msg_tx_[13] = ((cmd->remote_addr >> 16) & 0xff);
  this->msg_tx_[14] = ((cmd->remote_addr >> 8) & 0xff);
  this->msg_tx_[15] = ((cmd->remote_addr) & 0xff);
  this->msg_tx_[16] = 0x01;
  this->msg_tx_[17] = ((cmd->blind_addr >> 16) & 0xff);
  this->msg_tx_[18] = ((cmd->blind_addr >> 8) & 0xff);
  this->msg_tx_[19] = ((cmd->blind_addr) & 0xff);
  this->msg_tx_[20] = cmd->payload[0];
  this->msg_tx_[21] = cmd->payload[1];
  this->msg_tx_[22] = ((code >> 8) & 0xff);
  this->msg_tx_[23] = (code & 0xff);
  this->msg_tx_[24] = cmd->command;
  
  // Zero padding
  for (size_t i = 25; i <= this->msg_tx_[0]; i++) {
    this->msg_tx_[i] = 0x00;
  }
  
  msg_encode(&this->msg_tx_[22]);
  return transmit();
}

// --- NEW FUNCTION: Close and Reset Status ---
void Elero::close_and_reset_all() {
  ESP_LOGI(TAG, "Executing Close and Reset All Covers...");
  
  for (auto const& [addr, cover] : this->address_to_cover_mapping_) {
    ESP_LOGI(TAG, "Resetting cover: %s (0x%06X)", cover->get_name().c_str(), addr);
    
    // 1. Send DOWN command via Radio
    t_elero_command cmd;
    memset(&cmd, 0, sizeof(cmd));
    
    cmd.blind_addr = addr;
    cmd.remote_addr = cover->get_remote_address(); 
    cmd.channel = 1; 
    cmd.command = 0x40; // ELERO_CMD_DOWN
    cmd.counter = 1; 
    cmd.pck_inf[0] = 0x6A; 
    cmd.pck_inf[1] = 0x00;
    cmd.hop = 0x0A;
    cmd.payload[0] = 0x00;
    cmd.payload[1] = 0x04; 

    this->send_command(&cmd);

    // 2. Force internal state to 0.0 (Closed)
    // 0x02 is typically "Bottom Limit Reached" in Elero protocol
    cover->set_rx_state(0x02); 
  }
}

}  // namespace elero
}  // namespace esphome