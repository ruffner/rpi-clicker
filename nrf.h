/* Library for the nRF24L01+ module connected to the RaspberryPi

   Matt Ruffner 2014
 */

#ifndef NRF_H
#define NRF_H

#include <bcm2835.h>
#include "nRF24L01.h"

#define NRF_RATE_SLOW 0
#define NRF_RATE_MED  1
#define NRF_RATE_FAST 2

#define STD_CONFIG ( (1<<EN_CRC) | (1<<CRCO) )

#define CE_PIN RPI_GPIO_P1_22

class nrf {
 public:
  nrf(char chan, int pl_len, char * tx, char * rx);
  nrf(char chan, int pl_len);
  ~nrf();

  void send(char * data, int len);
  void receive(char * data);
  void listen(char * data);
  bool listen(char * data, int time_out_millis);
  char status();
  void flush_rx();

  void set_channel(char channel);
  void set_rx_addr(char * addr);
  void set_tx_addr(char * addr);
  void set_data_rate(int choice);

 private:
  char channel;
  uint8_t payload_length;
  char * tx_addr;
  char * rx_addr;
  uint8_t addr_len;

  uint8_t PTX;

  void ce_high();
  void ce_low();

  void config_register(char reg, char value);
  void read_register(char reg, char * value, int len);
  void write_register(char reg, char * value, int len);
  char get_status();
  bool rx_fifo_empty();
  bool data_ready();
  void power_up_rx();
  void power_up_tx();
  void get_data(char * data);

};

#endif
