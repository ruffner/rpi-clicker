/* Library for the nRF24L01+ module connected to the RaspberryPi

   Matt Ruffner 2014
 */

#include <sys/time.h>
#include <bcm2835.h>
#include "nRF24L01.h"
#include "nrf.h"

nrf::nrf(char chan, int pl_len) : nrf(chan, pl_len, new char[3], new char[3]) {}

nrf::nrf(char chn, int pl_len, char * tx, char * rx) {
  channel = (char)chn;
  PTX = 0;
  payload_length = (uint8_t)pl_len;
  addr_len = 3;
  tx_addr = tx;
  rx_addr = rx;

  bcm2835_init();
  
  bcm2835_gpio_fsel(CE_PIN, BCM2835_GPIO_FSEL_OUTP);

  ce_low();
 
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

  config_register(EN_AA, 0x00);              // disable shockburst
  config_register(SETUP_AW, 0x01);        // 3 byte address
  config_register(RF_CH, channel);             // channel 53
  config_register(RF_SETUP, 0x06);  // 250Kbps - max  power
  config_register(SETUP_RETR, 0x00);
  write_register(RX_ADDR_P0, rx_addr, addr_len);
  write_register(TX_ADDR, tx_addr, addr_len);
  config_register(RX_PW_P0, payload_length);

  power_up_rx();
  flush_rx();
}

nrf::~nrf() {
  bcm2835_spi_end();
  bcm2835_close();
}

void nrf::send(char * values, int len) {
  uint8_t status;
  status = get_status();
  while (PTX) {
    status = get_status();
    if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
      PTX = 0;
      break;
    }
  }
  ce_low();
  power_up_tx();
  bcm2835_spi_transfer( FLUSH_TX ); // flush tx fifo
  char data[len+1];
  data[0] = W_TX_PAYLOAD;
  for(int i = 1; i < len+1; i++)
    data[i] = values[i-1];
  bcm2835_spi_writenb(data, len+1);
  ce_high(); // Start transmission
}

void nrf::listen(char * data) {
  power_up_rx();
  while(!data_ready());
  get_data(data);
}

bool nrf::listen(char * data, int timeout_millis) {
  power_up_rx();
  struct timeval begin, now, diff;
  gettimeofday(&begin, 0);
  do {
    gettimeofday(&now, 0);
    timersub(&now, &begin, &diff);
  } while(((diff.tv_sec * 1000) + (diff.tv_usec / 1000) < timeout_millis) && !data_ready());
  if(!data_ready())
    return false;
  get_data(data);
  return true;
}

void nrf::set_data_rate(int choice) {
  switch (choice) {
  case NRF_RATE_SLOW:
    config_register(RF_SETUP, 0x26);
    break;
  case NRF_RATE_MED:
    config_register(RF_SETUP, 0x06);
    break;
  case NRF_RATE_FAST:
    config_register(RF_SETUP, 0x0E);
    break;
  }
}

char nrf::status() {
  return get_status();
}

void nrf::set_channel(char chan) {
  channel = chan;
}

void nrf::set_rx_addr(char * addr) {
  rx_addr = addr;
  write_register(RX_ADDR_P0, rx_addr, addr_len);
}

void nrf::set_tx_addr(char * addr) {
  ce_low();
  tx_addr = addr;
  write_register(TX_ADDR, tx_addr, addr_len);
}

void nrf::ce_high() {
  bcm2835_gpio_write(CE_PIN, HIGH);
}

void nrf::ce_low() {
  bcm2835_gpio_write(CE_PIN, LOW);
}

void nrf::config_register(char reg, char value) {
  char data[2] = {(char)(W_REGISTER | (REGISTER_MASK & reg)), value};
  bcm2835_spi_writenb(data, sizeof(data));
}

void nrf::read_register(char reg, char * value, int len) {
  char data[len+1];
  data[0] = R_REGISTER | (REGISTER_MASK & reg);
  for(int i = 1; i < len+1; i++)
    data[i] = value[i-1];
  bcm2835_spi_transfern(data, len+1);
  for(int i = 0; i < len; i++)
    value[i] = data[i+1];
}

void nrf::write_register(char reg, char * value, int len) {
  char data[len+1];
  data[0] = W_REGISTER | (REGISTER_MASK & reg);
  for(int i = 1; i < len+1; i++)
    data[i] = value[i-1];
  bcm2835_spi_writenb(data, len+1);
}

char nrf::get_status() {
  char rv = bcm2835_spi_transfer(0x00); // nop to get status
  return rv;
}

bool nrf::rx_fifo_empty() {
  char status;
  read_register(FIFO_STATUS, &status, 1);
  return (status & (1 << RX_EMPTY));
}

bool nrf::data_ready() {
  char status = get_status();
  if ( status & (1 << RX_DR) ) return 1;
  return !rx_fifo_empty();
}

void nrf::flush_rx() {
  bcm2835_spi_transfer( FLUSH_RX );
}

void nrf::power_up_rx() {
  PTX = 0;
  ce_low();
  config_register(CONFIG, STD_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) );
  ce_high();
  config_register(STATUS, (1 << TX_DS) | (1 << MAX_RT)); 
}

void nrf::power_up_tx() {
  PTX = 1;
  config_register(CONFIG, STD_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}

void nrf::get_data(char * value) {
  char data[payload_length+1];
  data[0] = R_RX_PAYLOAD;
  for(int i = 1; i < payload_length+1; i++)
    data[i] = value[i-1];
  bcm2835_spi_transfern(data, payload_length+1);
  for(int i = 0; i < payload_length; i++)
    value[i] = data[i+1];

  config_register(STATUS, (1<<RX_DR));    // Reset status register
}
