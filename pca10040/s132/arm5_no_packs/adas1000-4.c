// Includes;
#include "adas1000-4.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "ble_eeg.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "custom_board.h"
//#include "compiler_abstraction.h"
//#include "nrf.h"
#//include <stdio.h>

uint8_t ADAS1000_4_DEFAULT_REGS[] = {0x00};

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0);
static volatile bool spi_xfer_done;

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const *p_event,
    void *p_context) {
  spi_xfer_done = true;
}

void adas_spi_init(void) {
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
  //SCLK = 1MHz is right speed because fCLK = (1/2)*SCLK, and fMOD = fCLK/4, and fMOD MUST BE 128kHz. Do the math.
  spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
  spi_config.irq_priority = APP_IRQ_PRIORITY_HIGHEST; //APP_IRQ_PRIORITY_HIGHEST;
  spi_config.mode = NRF_DRV_SPI_MODE_3;               //CPOL = 1 (Active Low); CPHA = TRAILING (1)
  spi_config.miso_pin = ADAS1000_4_SDO;               //Master (nrf) in, slave (adas) out
  spi_config.sck_pin = ADAS1000_4_SCLK;
  spi_config.mosi_pin = ADAS1000_4_SDI;
  spi_config.ss_pin = ADAS1000_4_CS;
  spi_config.orc = 0x55;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
  NRF_LOG_INFO(" SPI Initialized..\r\n");
}

void adas_spi_uninit(void) {
  nrf_drv_spi_uninit(&spi);
  NRF_LOG_INFO(" SPI Disabled! \r\n");
}

void adas_powerdown(void) {
  //Bring ~PD Low to Power Down:
  //NOTE that the registers are not affected by a power down.
  nrf_gpio_pin_clear(ADAS1000_4_PWDN);
  NRF_LOG_INFO(" ADAS1000-4 Powered Down. \r\n");
}

void adas_powerup(void) {
  // ~PD to Power Up/
  nrf_gpio_pin_set(ADAS1000_4_PWDN);
  // Wait 2ms:
  nrf_delay_ms(2);
  NRF_LOG_INFO(" ADAS1000-4 Powered Up. \r\n");
}

void adas_reset_regs(void) {
  //Resets Registers to Startup Config
  nrf_gpio_pin_clear(ADAS1000_4_RESET);
  //WAIT 1.5 ms:
  nrf_delay_ms(2);
  //Bring pin back up:
  nrf_gpio_pin_set(ADAS1000_4_RESET);
  NRF_LOG_INFO(" ADAS1000-4 Registers Reset. \r\n");
}

void adas_write_register(uint8_t addr, uint32_t value) {
  // TX DATA [REG ADDR, BITS 23:16, BITS 15:8, BITS 7:0]
  uint8_t tx_data[4] = {0x80 + addr, (uint8_t)(value & 0xFF0000) >> 16, (uint8_t)(value & 0x00FF00) >> 8, (uint8_t)(value & 0x0000FF) >> 0};
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, 4, NULL, NULL));
}

void adas_read_register(uint8_t addr, uint32_t *value) {
  uint8_t tx_data[4] = {addr, 0, 0, 0};
  uint8_t rx_data[4] = {0, 0, 0, 0};
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, 4, rx_data, 4));
  *value = ((uint32_t)rx_data[1] << 16) + ((uint32_t)rx_data[2] << 8) + ((uint32_t)rx_data[3] << 0);
}

void adas_write_default_registers(void) {
  uint8_t num_registers = 1;
  bool write_opcode[] = {true, true, true, false}; 
  uint8_t register_addresses[] = {0x05, 0x0A, 0x01};
  uint32_t registers_values[] = {0x00E0000B, 0x001F9600, 0x00E004AE};
  uint8_t i = 0;
  for (i = 0; i < num_registers; i++) {
    adas_write_register(register_addresses[i], registers_values[i]);
  }
  NRF_LOG_INFO(" ADAS1000-4 Default Registers Written. \r\n");
}

void adas_begin_frame_transactions(void) {
  //Begins data transactions by reading FRAMES register (read 0x40):
  //Read from 0x40:
  uint32_t response = 0x00000000;
  adas_read_register(0x40, &response);
  NRF_LOG_INFO(" ADAS1000-4 Issuing FRAMES Read, Response: 0x00%X%X%X. \r\n", (uint8_t)(response & 0xFF0000) >> 16, (uint8_t)(response & 0x00FF00) >> 8, (uint8_t)(response & 0x0000FF) >> 0);
}