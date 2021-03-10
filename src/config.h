#ifndef CONFIG_H_
#define CONFIG_H_

#include "em_acmp.h"

#define CONFIG_AD_NCHANS 2
extern volatile int gl_ad_sampling_rate; // Per sample, not scan
extern volatile double gl_ad_scan_rate; // Per scan

extern volatile int fastcharge; // Per scan
extern volatile bool gl_comp_ref_64_ladder;

int AD_config(void);

void aux_sensor_config(const ADC_SingleInput_TypeDef);

void comp_config(const uint32_t *const trig_levels, const ACMP_Channel_TypeDef *const pos_sel1);

void trigg_ref_set(const uint32_t value);

void trigg_ref_reset(void);

void preamp_config(const uint16_t *const preamp_status);

void preamp_reset();

void preamp_set_status(const uint16_t *const preamp_status);

int battery_charge_status(const uint16_t battery_voltage, const int16_t logger_temperature);

void uart0_config(void);

void uart1_config(void);

void rtc_config(void);

void burtc_config();

void wdog_config();

void GPIO_setup(void);

void dust_mote_reset(void);

void efm32_reset(void);

void dust_mote_setup(void);

void dust_mote_sleep(void);

void external_hard_reset(void);

void charge_mode(int);

#endif
