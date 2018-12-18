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

uint32_t ADAS1000_4_DEFAULT_REGS[] = {0x85A0000A, 0x8A1F8E03, 0x81A004C6, 0x83002019, 0x84000000, 0x87000000, 0x8E000000, 0x8F000000, 0x40000000};

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
  spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
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

void adas_write_register(uint32_t value) {
  // TX DATA [REG ADDR, BITS 23:16, BITS 15:8, BITS 7:0]
  uint8_t tx_data[4];
  tx_data[0] = (uint8_t)((value >> 24) & 0xFF);
  tx_data[1] = (uint8_t)((value >> 16) & 0xFF);
  tx_data[2] = (uint8_t)((value >> 8) & 0xFF);
  tx_data[3] = (uint8_t)(value & 0xFF);
  spi_xfer_done = false;
  NRF_LOG_HEXDUMP_DEBUG(tx_data, 4);
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, 4, NULL, NULL));
  while (!spi_xfer_done) {
    __WFE();
  }
}

void adas_write_register_array(uint8_t *value) {
  spi_xfer_done = false;
  NRF_LOG_HEXDUMP_DEBUG(value, 4);
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, value, 4, NULL, NULL));
  while (!spi_xfer_done) {
    __WFE();
  }
}

void adas_write_registers_new(ble_eeg_t *p_eeg, uint8_t *new_register_values) {
  uint8_t num_registers = 9;
  // Copy default regs to update current config on BLE side
  memcpy_fast(&p_eeg->adas1000_4_current_configuration[0], &new_register_values[0], num_registers * sizeof(uint32_t));
  uint8_t i;
  adas_write_default_registers(p_eeg, true);
}

void adas_write_default_registers(ble_eeg_t *p_eeg, bool new_config) {
  uint8_t num_registers = 9;
  // Copy default regs to update current config on BLE side
  if (!new_config) {
    memcpy_fast(&p_eeg->adas1000_4_current_configuration[0], &ADAS1000_4_DEFAULT_REGS[0], num_registers * sizeof(uint32_t));
  }
  // Cycle through and write each of the words to the respective registers
  uint8_t i;
  for (i = 0; i < num_registers; i++) {
      adas_write_register(p_eeg->adas1000_4_current_configuration[i]);
  }
  NRF_LOG_INFO(" ADAS1000-4 Default Registers Written. \r\n");
}

void adas_read_register(uint8_t addr, uint32_t *value) {
  uint8_t tx_data[4];
  memset(tx_data, 0, 4);
  tx_data[0] = (uint8_t)addr;
  uint8_t rx_data[4];
  memset(rx_data, 0, 4);
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, 4, rx_data, 4));
  while (!spi_xfer_done) {
    __WFE();
  }
  *value = ((uint32_t)rx_data[0] << 24) + ((uint32_t)rx_data[1] << 16) + ((uint32_t)rx_data[2] << 8) + ((uint32_t)rx_data[3] << 0);
}

void adas_read_frames(uint8_t number_frames, ble_eeg_t *p_eeg) {
    uint8_t rx_data[4 * number_frames];
    memset(rx_data, 0, 4 * number_frames);
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, rx_data, 4 * number_frames, rx_data, 4 * number_frames));
    while (!spi_xfer_done) {
      __WFE();
    }
    //NRF_LOG_HEXDUMP_DEBUG(rx_data, 4*number_frames);
    // Add frames to ECG Ch Data (24-bit)
    memcpy_fast(&p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count], &rx_data[5], 3);
    memcpy_fast(&p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count], &rx_data[17], 3);
    memcpy_fast(&p_eeg->eeg_ch3_buffer[p_eeg->eeg_ch1_count], &rx_data[21], 3);
    p_eeg->eeg_ch1_count += 3;
    //memcpy_fast(&p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count], &rx_data[0], 4 * number_frames);
    //p_eeg->eeg_ch1_count += (4*number_frames);
}
