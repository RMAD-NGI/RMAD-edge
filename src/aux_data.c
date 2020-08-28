/*
 * aux_data.c
 *
 *  Created on: 29. juli 2020
 *      Author: KVIYME
 */
#include <assert.h>
#include <math.h>

#include "process.h"
#include "em_adc.h"

#include "aux_data.h"
#include "rmad.h"
#include "config.h"
#include "dust_com.h"
#include "adjustable_params.h"
#include "process.h"

volatile static int charging = 1;

volatile static int num_aux_samples;

volatile static int current_aux_scan;
volatile static int current_aux_sample;



volatile ADC_SingleInput_TypeDef adc_ch_list_preamp_off[] = {adcSingleInputCh7,adcSingleInputCh0}; //adcSingleInpCh7 = solar, must be first
volatile ADC_SingleInput_TypeDef adc_ch_list_system[] = {adcSingleInputTemp,adcSingleInputVDDDiv3};
volatile ADC_SingleInput_TypeDef adc_ch_list_preamp_on[] = {}; //adcSingleInputCh0,adcSingleInputCh1

static int num_preamp_off = sizeof(adc_ch_list_preamp_off);
static int num_system = sizeof(adc_ch_list_system);
static int num_preamp_on = sizeof(adc_ch_list_preamp_on);

sample_t aux_sample[12];
//scan_t aux_scan;

static int aux_ad_sampling_rate;

static int trigg1_status = 0;
static int trigg2_status = 0;



void get_aux_and_report(void){  //starting point for generating aux_data packet

	//int sleep_mode_recording;

	if(gl_mote_online_check){

		//ADC_IntDisable(ADC0, ADC_IF_SINGLE);
	    //ADC_Reset(ADC0);

        trigg1_status = (ACMP0 -> STATUS) & 2;
        trigg2_status = (ACMP0 -> STATUS) & 2;

		gl_num_aux = 0;
		//aux_sample[0] = 0;

	    aux_sensor_config(adcSingleInputCh7);  // dummy reading on solar channel channel, trown away as the first reading often fails.

	    gl_get_aux_data = true;
	    gl_sleep_mode_recording = 1;
	    //sleep_mode_recording = 1;

	    ADC_Start(ADC0, adcStartSingle);

	    gl_aux_data_is_scan = false;


	//} else {

	//	sleep_mode_recording = 3;

	}

	//return sleep_mode_recording;
}


void read_single_aux_sample(void){   // called from ADC interupt


	const sample_t sample = (sample_t)ADC_DataSingleGet(ADC0);
	aux_sample[gl_num_aux] = sample;

	ADC_IntDisable(ADC0, ADC_IF_SINGLE);
    ADC_Reset(ADC0);

    gl_aux_data_recived = true;
    gl_num_aux = gl_num_aux + 1;

}


void read_scan_aux_sample(void){  // called from ADC interupt

	//static int aux_num_sample = 0;
	static int aux_num_scan= 0;

	//static double sum_a=0;
	//static double sum_b=0;

	static double sum_a = 0.0, sum_sqr_a = 0.0, sum_offset_a = 0.0;
	static double sum_b = 0.0, sum_sqr_b = 0.0, sum_offset_b = 0.0;

	static scan_t aux_scan;
	static int aux_scan_sample_a[400];
	static int aux_scan_sample_b[400];

    const unsigned int ch = (ADC0 ->STATUS & 50331648) >> 24;// 2 bits, don't want to keep checking for overflow...
    sample_t sample = (sample_t)ADC_DataScanGet(ADC0);
    aux_scan[ch] = sample;

    //++aux_num_sample;


    if (ch == CONFIG_AD_NCHANS-1) {

    	aux_scan_sample_a[aux_num_scan] = aux_scan[0];
    	sum_offset_a += aux_scan[0];
    	aux_scan_sample_b[aux_num_scan] = aux_scan[1];
    	sum_offset_b += aux_scan[1];

		sum_a += aux_scan[0] - 0x7FFF;
		sum_sqr_a += (aux_scan[0]- 0x7FFF) * (aux_scan[0] - 0x7FFF);

		sum_b += aux_scan[1] - 0x7FFF;
		sum_sqr_b += (aux_scan[1]- 0x7FFF) * (aux_scan[1] - 0x7FFF);

    	++aux_num_scan;

    }


    if (aux_num_scan == 400){

	   	ADC_Reset(ADC0);

	   	int sample_mean_a = sum_offset_a / aux_num_scan;
	   	int sample_mean_b = sum_offset_b / aux_num_scan;

	   	float sdv_a = sqrt((sum_sqr_a - sum_a*sum_a/aux_num_scan) / (aux_num_scan - 1));
	   	float sdv_b = sqrt((sum_sqr_b - sum_b*sum_b/aux_num_scan) / (aux_num_scan - 1));

	    //aux_sample[num_preamp_off + num_system + num_preamp_on + 1] = aux_scan_sample_a[0];
	   	//aux_sample[num_preamp_off + num_system + num_preamp_on + 2] = aux_scan_sample_b[0];

	   	aux_sample[num_preamp_off + num_system + num_preamp_on + 1] = sample_mean_a;
	   	aux_sample[num_preamp_off + num_system + num_preamp_on + 2] = sample_mean_b;

	   	aux_sample[num_preamp_off + num_system + num_preamp_on + 3] = sdv_a;
	   	aux_sample[num_preamp_off + num_system + num_preamp_on + 4] = sdv_b;

	   	//aux_sample[num_preamp_off + num_system + num_preamp_on + 6] = std_deviance(aux_scan_sample_a, aux_num_scan, sample_mean_a);

	    gl_aux_data_recived = true;
	    gl_num_aux = gl_num_aux + 1;
	    //aux_num_sample = 0;
	    aux_num_scan = 0;

	    //sum_a=0;
	    //sum_b=0;
	    sum_a = 0.0;
	    sum_sqr_a = 0.0;
	    sum_offset_a = 0.0;

	    sum_b = 0.0;
	    sum_sqr_b = 0.0;
	    sum_offset_b = 0.0;


    }

    ADC_IntClear(ADC0, ADC_IF_SCAN);
    //ADC_IntEnable(ADC0, ADC_IF_SCAN);

}


void get_new_aux_sample(void){  //called from main

	//int sleep_mode_recording;

	if (gl_num_aux <= num_preamp_off){

		aux_sensor_config(adc_ch_list_preamp_off[gl_num_aux-1]);
		ADC_Start(ADC0, adcStartSingle);

		gl_aux_data_is_scan = false;

		//sleep_mode_recording = 1;

	} else if (gl_num_aux <= (num_preamp_off + num_system)){


		aux_sensor_config(adc_ch_list_system[gl_num_aux - num_preamp_off-1]);
		ADC_Start(ADC0, adcStartSingle);

		gl_aux_data_is_scan = false;

		//sleep_mode_recording = 1;

			if(gl_num_aux == num_preamp_off+1){

			ACMP_IntDisable(ACMP0, ACMP_IF_EDGE);  // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv
			ACMP_IntDisable(ACMP1, ACMP_IF_EDGE);  // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv

			DAC0 -> CH0DATA = 2048 + gl_adjustable_params->comp_trig_levels[0];
			DAC0 -> CH1DATA = 2048;
			preamp_set_status(gl_adjustable_params->preamp_logging);

		    ACMP_IntClear(ACMP0, ACMP_IF_EDGE); // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv
		    ACMP_IntEnable(ACMP0, ACMP_IF_EDGE); // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv
		    ACMP_IntClear(ACMP1, ACMP_IF_EDGE); // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv
		    ACMP_IntEnable(ACMP1, ACMP_IF_EDGE); // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv

		}

	} else if (gl_num_aux <= (num_preamp_off + num_system + num_preamp_on)){

		    aux_sensor_config(adc_ch_list_preamp_on[gl_num_aux - num_preamp_off - num_system - 1]);
		    ADC_Start(ADC0, adcStartSingle);

		    gl_aux_data_is_scan = false;

		    //sleep_mode_recording = 1;

	}else if (gl_num_aux == (num_preamp_off + num_system + num_preamp_on + 1)){

			//ADC_Reset(ADC0);

		    aux_ad_sampling_rate = AD_config();
		    ADC_Start(ADC0, adcStartScan);

			//aux_sensor_config_low(adc_ch_list_preamp_on[0]);
			//ADC_Start(ADC0, adcStartSingle);

		    //gl_sleep_mode_recording = 1;

		    gl_aux_data_is_scan = true;

		    //aux_num_sample = 0;

	} else {


		ACMP_IntDisable(ACMP0, ACMP_IF_EDGE);  // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv
		ACMP_IntDisable(ACMP1, ACMP_IF_EDGE);  // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv

		DAC0 -> CH1DATA = 0;
		DAC0 -> CH0DATA = gl_adjustable_params->comp_trig_levels[0];
		preamp_set_status(gl_adjustable_params->preamp_cmp_trigg);

	    ACMP_IntClear(ACMP0, ACMP_IF_EDGE); // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv
	    ACMP_IntEnable(ACMP0, ACMP_IF_EDGE); // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv
	    ACMP_IntClear(ACMP1, ACMP_IF_EDGE); // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv
	    ACMP_IntEnable(ACMP1, ACMP_IF_EDGE); // lagt til for å test om rebootloopen skyldes trigg før mote er aktiv

	    send_aux_data();

	    ADC_Reset(ADC0);

	    gl_get_aux_data = false;
	    gl_aux_data_recived = false;

	    gl_num_aux = 0;
	    gl_sleep_mode_recording = 3;
	    //sleep_mode_recording = 3;

	}

	//return sleep_mode_recording;
}




void send_aux_data() {
    EFM32_PACK_START(1); // Actually a no-op for GNU it seems, for GNU we use __attribute__ ((__packed__))
    // More info: See https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html#Type-Attributes
    typedef struct
        __attribute__ ((__packed__)) {
            // NOT USED: uint8_t     life_phase; // Indicates where we are in the lifetime of the system: startup/status check requested/etc
    		uint8_t system;
            uint8_t revition;                   // Non-zero iff an out socket is open. Does not access the mote, so may not catch if the mote died "recently".
            uint8_t state;                 // True iff battery is charging

            uint16_t vdd3;
            int16_t temperature;

            uint8_t signed_number;

            //uint16_t ai4;
            uint16_t ai5;
            uint16_t ai6;
            //uint16_t ai7;
            //uint16_t ai8;
            //uint16_t ai9;
            uint16_t ai10;
            uint16_t ai11;
            uint16_t ai12;
            uint16_t ai13;
            //uint16_t ai14;
            //uint16_t ai15;



        } system_status_t;
        EFM32_PACK_END(); // Actually a no-op for GNU it seems

        //int sw_vertion = 1;
        //int hw_revision = 1;

        system_status_t status;

        status.system =  HW_CONFIGURATION * 16 + SW_VERTION;
        status.revition = HW_REVITION * 16 + HW_RADIO;

        //status.system = sw_vertion + hw_revision * 16;

        status.vdd3 = aux_sample[4]; //29360; //
        status.temperature = convertToCelsius(aux_sample[3]); //2200; //

        //int charging = battery_charge_status(status.vdd3, status.temperature); //statusbit endret til charge enable - PC7 - i hw4 og hw5
        //int charging = 0; //!GPIO_PinInGet(gpioPortD, 8);
        //int connected = (gl_socket_id >= 0) ? 1 : 0;

        int fastcharge = 0;

        status.state = charging +  2*fastcharge + 2*trigg2_status + 4*trigg1_status;

        status.signed_number = 0;

        //status.ai4 = aux_sample[0];
        status.ai5 = aux_sample[1];
        status.ai6 = aux_sample[2];
        //status.ai7 = aux_sample[3];
        //status.ai8 = 0; //aux_sample[4];
        //status.ai9 = 0; //aux_sample[5];
        status.ai10 = aux_sample[5];
        status.ai11 = aux_sample[6];
        status.ai12 = aux_sample[7];
        status.ai13 = aux_sample[8];
        //status.ai14 = aux_sample[10];
        //status.ai15 = aux_sample[11];


        if (gl_socket_id > 0)	//satt inn for og redusere trafikk mellom efm32 og dust when mote lost
        {
        	dust_tx_msg_data(OMSG_AUX, &status, sizeof(status));
        }

        charging = battery_charge_status(status.vdd3, status.temperature); //statusbit endret til charge enable - PC7 - i hw4 og hw5 - flyttet hit for at sendt bit skal representere status ved avlesning av batterispenning

    }



int convertToCelsius(sample_t adcSample)
{
  int16_t temp;

  /* Factory calibration temperature from device information page. */
  float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
                             >> _DEVINFO_CAL_TEMP_SHIFT);

  float cal_value_0 = (float)((DEVINFO->ADC0CAL2
                               & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
                              >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  /* Temperature gradient (from datasheet) */
  float t_grad = -6.3;

  temp = 100*(cal_temp_0 - ((cal_value_0 - adcSample/8) / t_grad));

  return temp;
}
