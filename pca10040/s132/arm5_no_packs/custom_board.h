#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H


#if defined (ADAS1000_4_BOARD_V1)

#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

// Define Pins:
// 1. TPS63051
#define TPS63051_EN 19
#define TPS63051_ILIM0 20
#define TPS63051_PG 21
// 2. Clk:
#define CLK_DSC1001_STANDBY 11
// 3. ADAS1000-4
#define ADAS1000_4_CS 12
#define ADAS1000_4_DRDY 13
#define ADAS1000_4_SDI 14
#define ADAS1000_4_SCLK 15
#define ADAS1000_4_SDO 16
#define ADAS1000_4_PWDN 17
#define ADAS1000_4_RESET 18
// 4. TPS62746 Load Switch (Low Priority)
#define TPS62746_LOAD_SW_CTRL 4
#define TPS62746_AIN_BATT_V 3 //AIN1

#endif //< defined (ADAS1000_4_BOARD_V1)


#endif //< #ifndef CUSTOM_BOARD_H