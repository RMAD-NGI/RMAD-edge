/*
 * aux_data.h
 *
 *  Created on: 29. juli 2020
 *      Author: KVIYME
 */

#ifndef AUX_DATA_H_
#define AUX_DATA_H_




void get_aux_and_report(void);

void read_single_aux_sample(void);

void read_scan_aux_sample(void);

void get_new_aux_sample(void);

void send_aux_data(void);

int convertToCelsius(sample_t);

#endif /* AUX_DATA_H_ */
