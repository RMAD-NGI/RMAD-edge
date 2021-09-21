#include <math.h>

#include "em_gpio.h"
#include "em_adc.h"
#include "em_dac.h"
#include "em_acmp.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_rtc.h"
#include "em_wdog.h"
//#include "em_burtc.h"
#include "rtcdrv.h"
#include "config.h"
#include "dust_com.h"
#include "rmad.h"

/*#include "segmentlcd.h" only for debugging in this function, shall be removed*/
volatile int gl_ad_sampling_rate;
volatile double gl_ad_scan_rate; // Per scan



volatile bool charging_enable = true;
volatile bool prev_low_batt_volt =false;

volatile int fastcharge = 0;

volatile bool gl_comp_ref_64_ladder;


void GPIO_setup(void)
{

	GPIO_PinModeSet(gpioPortC,7,gpioModeWiredAnd,1); /*charge enable - mcp73811   1=on 0=off*/

    if (HW_REVITION >= 6)
    {
    	GPIO_PinModeSet(gpioPortF,12,gpioModeWiredAnd,1); /*hard reset smartmeshIP mote   1=off 0=on*/

    	GPIO_PinModeSet(gpioPortD,8,gpioModeInput,0); //set external reset pin as input when not in use

    	//GPIO_PinModeSet(gpioPortD,8,gpioModeWiredAnd,1); /*external hard reset     1=off 0=on*/

    	GPIO_PinModeSet(gpioPortF,11,gpioModeWiredAndPullUp,0); /*fast charge    1=on 0=off*/
    	//GPIO_PinModeSet(gpioPortC,7,gpioModeWiredAndPullUp,1); /*charge enable - mcp73811   1=on 0=off*/

    	//charge_mode(0);
    //} else{


    }

}

void charge_mode(charge_mode)
{
	if (charge_mode > 0)
	{
		GPIO_PinModeSet(gpioPortF,11,gpioModeWiredAndPullUp,1); //fast charge 450mA
		if(gl_debug_on_UART1)printf("\ncharge_mode() - fast charge enabled");

	}else{

		GPIO_PinModeSet(gpioPortF,11,gpioModeWiredAndPullUp,0); //slow charge 85mA
		if(gl_debug_on_UART1)printf("\ncharge_mode() - fast charge disabled");
	}

	fastcharge = charge_mode;

}

void efm32_reset(void){

    if(gl_debug_on_UART1)if(gl_debug_on_UART1)printf("\n\n***calling NVIC_SystemReset() for hw6***");
    wait(10);
    NVIC_SystemReset();  // a terrible hack to make stop a full halt at this point - need to find a propper solution

}

void dust_mote_reset(void)
{

	if (HW_REVITION>= 6)
	        {

				if(gl_debug_on_UART1)printf("\nSmartMesh IP mote request - Hard Reset");
	        	GPIO_PinModeSet(gpioPortF,12,gpioModeWiredAnd,0); /*hard reset smartmeshIP mote   1=off 0=on*/
	        	wait(500);
	        	GPIO_PinModeSet(gpioPortF,12,gpioModeWiredAnd,1); /*hard reset smartmeshIP mote   1=off 0=on*/

	        }else{

	        	if(gl_debug_on_UART1)printf("\nSmartMesh IP mote request - Soft Reset");
	        	DUST_SEND({0x08, 0x00, 0x00}); // dust mote soft reset

	        }
}

void dust_mote_setup(void){

	DUST_SEND({0x02, 0x04, 0x00, 0x0d}); // get mac adress

	DUST_SEND({0x01, 0x05, 0x00, 0x24, 0x01}); // set autojoin

	DUST_SEND({0x01, 0x05, 0x00, 0x06, 0xFF}); // Set joinDutyCycle to 100%

	//DUST_SEND({0x02, 0x04, 0x00, 0x06}); // get joinDutyCycle%

	//if (HW_REVITION>= 6){

			//DUST_SEND({0x01, 0x02, 0x00, 0x03, 0xFF, 0xFF}); // set networkID = FFFF can join all networkIDs

			DUST_SEND({0x01, 0x06, 0x00, 0x03, 0x07, 0xBD}); // set networkID = 1981

	//}


	//if (SW_VERTION == 0){

	//    	DUST_SEND({0x01, 0x02, 0x02, 0x06, 0xFF}); // Set joinDutyCycle to 100%

	//}else{

	//    	DUST_SEND({0x01, 0x02, 0x02, 0x06, 0x18}); // Set joinDutyCycle to 10%

	//}

    wait(500);

}

void dust_mote_sleep(void)
{

	if(gl_debug_on_UART1)printf("\nSmartMesh IP mote request - Deep Sleep");
	DUST_SEND({0x07, 0x00, 0x00}); // disconnect mote
	wait(1000);
	DUST_SEND({0x09, 0x00, 0x00}); // send mote to sleep

}

void external_hard_reset(void)
{
	GPIO_PinModeSet(gpioPortD,8,gpioModeWiredAnd,0); /*external hard reset*/
	wait(500);
	GPIO_PinModeSet(gpioPortD,8,gpioModeInput,0); /*clear external reset*/
}


int AD_config(void)
{

	/* Base the ADC configuration on the default setup. */

	ADC_Init_TypeDef ainit;
	ADC_InitScan_TypeDef gInit;

	/* Initialize timebases */
	ainit.ovsRateSel = adcOvsRateSel512;  // På ett tidspungt halvert sampligraten seg, og som en quick fix endret vi denne parameteren til fra 1024 til 512
	//ainit.ovsRateSel = adcOvsRateSel1024;
	ainit.warmUpMode = adcWarmupKeepADCWarm;
	ainit.prescale = 0;
	ainit.timebase = ADC_TimebaseCalc(0);
	ainit.tailgate = 0;
	ainit.lpfMode = adcLPFilterBypass;

	/* Set input for scan */
	gInit.prsSel = 0;
	gInit.prsEnable = false;
	gInit.acqTime = adcAcqTime4;
	gInit.reference = adcRef2V5;

	// If changing the following, also change CONFIG_AD_NCHANS
	if(CONFIG_AD_NCHANS==1) gInit.input = ADC_SCANCTRL_INPUTMASK_CH0;
	if(CONFIG_AD_NCHANS==2) gInit.input = ADC_SCANCTRL_INPUTMASK_CH0 + ADC_SCANCTRL_INPUTMASK_CH1;
	if(CONFIG_AD_NCHANS==3) gInit.input = ADC_SCANCTRL_INPUTMASK_CH0 + ADC_SCANCTRL_INPUTMASK_CH1 + ADC_SCANCTRL_INPUTMASK_CH2;
	if(CONFIG_AD_NCHANS==4) gInit.input = ADC_SCANCTRL_INPUTMASK_CH0 + ADC_SCANCTRL_INPUTMASK_CH1 + ADC_SCANCTRL_INPUTMASK_CH2 + ADC_SCANCTRL_INPUTMASK_CH3;

	gInit.resolution = adcResOVS;
	gInit.leftAdjust = 0;
	gInit.diff = false;
	gInit.rep = true;

	ADC_Init(ADC0, &ainit);
	ADC_InitScan(ADC0, &gInit);


	// Presumably correct, but what is the "+ 12"?
	gl_ad_sampling_rate = (CMU_ClockFreqGet(cmuClock_ADC0)/(ainit.prescale+1)) / ((pow(2,gInit.acqTime) + 12) * pow(2,(ainit.ovsRateSel+1))); // (/2) enda en quick fiks som ikke er bra, det er åpenbart noe feil ved utregningen av samplingraten.

	//if(gl_debug_on_UART1)printf("\nAD_config() - gl_ad_sampling_rate(sps*ch) = %d", gl_ad_sampling_rate);

	gl_ad_sampling_rate = 854; //fixed sampligrate for two channel (RMAD-Railway), as the above calculation gives wrong results

	//if(gl_debug_on_UART1)printf("\nAD_config() - gl_ad_sampling_rate(sps*ch) = %d", gl_ad_sampling_rate);

	gl_ad_scan_rate = (double)gl_ad_sampling_rate / CONFIG_AD_NCHANS;

	ADC_IntClear(ADC0, ADC_IF_SCAN);
	ADC_IntEnable(ADC0, ADC_IF_SCAN);
	NVIC_EnableIRQ(ADC0_IRQn);
	NVIC_SetPriority(ADC0_IRQn, 4);

	return gl_ad_sampling_rate;
}

void aux_sensor_config(const ADC_SingleInput_TypeDef channel)
{

  ADC_Init_TypeDef dinit;
  ADC_InitSingle_TypeDef sInit;

  dinit.warmUpMode = adcWarmupNormal;
  dinit.ovsRateSel = adcOvsRateSel4096;
  dinit.lpfMode = adcLPFilterBypass;
  dinit.timebase = ADC_TimebaseCalc(0);
  dinit.prescale = 0;
  dinit.tailgate = 0;

  sInit.reference = adcRef2V5;
  sInit.resolution = adcResOVS;
  sInit.acqTime = adcAcqTime256;
  sInit.leftAdjust = 0;
  sInit.input = channel;
  sInit.diff = false;


  sInit.rep = false;
  sInit.prsSel = 0;
  sInit.prsEnable = false;

  ADC_Init(ADC0, &dinit);
  ADC_InitSingle(ADC0, &sInit);

  /* Setup interrupt generation on completed conversion. */
  ADC_IntClear(ADC0, ADC_IF_SINGLE);
  ADC_IntEnable(ADC0, ADC_IF_SINGLE);
  NVIC_EnableIRQ(ADC0_IRQn);
  NVIC_SetPriority(ADC0_IRQn, 4);

}


void preamp_config(const uint16_t *const preamp_status)
{

	int test = preamp_status;

	GPIO_PinModeSet(gpioPortA,12,gpioModePushPull,(test & 1)); //added to test preamp 0 on/off
    GPIO_PinModeSet(gpioPortA,13,gpioModePushPull,(test & 2)); //added to test preamp 1 on/off
    GPIO_PinModeSet(gpioPortC,4,gpioModePushPull,(test & 4)); //added to test preamp 2 on/off
    GPIO_PinModeSet(gpioPortC,5,gpioModePushPull,(test & 8)); //added to test preamp 3 on/off

}

void preamp_reset()
{

	GPIO_PinOutClear(gpioPortA,12); //preamp 0 off
    GPIO_PinOutClear(gpioPortA,13); //preamp 1 off
    GPIO_PinOutClear(gpioPortC,4); //preamp 2 off
    GPIO_PinOutClear(gpioPortC,5); //preamp 3 off

}

void preamp_set_status(const uint16_t *const preamp_status)
{

	int test = preamp_status;

	if(test & 1)
	{
			GPIO_PinOutSet(gpioPortA,12); // preamp 0 on
		}else{
			GPIO_PinOutClear(gpioPortA,12); // preamp 0 off
		}

	if(test & 2)
		{
			GPIO_PinOutSet(gpioPortA,13); // preamp 1 on
		}else{
			GPIO_PinOutClear(gpioPortA,13); // preamp 1 off
		}

	if(test & 4)
		{
			GPIO_PinOutSet(gpioPortC,4); // preamp 2 on
		}else{
			GPIO_PinOutClear(gpioPortC,4); // preamp 2 off
		}

	if(test & 8)
		{
			GPIO_PinOutSet(gpioPortC,5); // preamp 3 on
		}else{
			GPIO_PinOutClear(gpioPortC,5); // preamp 3 off
		}

}

int battery_charge_status(uint16_t battery_voltage, int16_t logger_temperature)
{

	int batt_4_15 = 30234 + 436;  // fully charged at 4.2V
	int batt_4_05 = 29360;
	int batt_3_70 = 26301 - 875;  // 3.6V appears to be a better value for recovery
	int batt_3_50 = 24554; // 3.5V limit to enter deep sleep. Have experienced total discharge with limit set to 3.3V

	//if (HW_REVITION>= 6)
	//        {

	//			batt_4_15 = 30234;
	//			batt_4_05 = 29360;
	//			batt_3_70 = 26301 - 875;  // 3.6V appears to be a better value for recovery
	//			batt_3_50 = 24554;  // 3.5V limit to enter deep sleep. Have experienced total discharge with limit set to 3.3V

	//}


	if(gl_debug_on_UART1)printf("\nbattery_charge_status() - battery_voltage(bin) = %d", battery_voltage);

	//lading av batteri med hysteresis
	if (battery_voltage > batt_4_15) // batterispenning høyere enn 4.15V
		{
			if(gl_debug_on_UART1)printf("\nbattery_charge_status() - disable charging as voltage > 4.20V");
			charging_enable = false;
		}
	else if (battery_voltage < batt_4_05) // batterispenning lavere enn 4.05V
		{
			if(gl_debug_on_UART1)printf("\nbattery_charge_status() - enable charging as voltage < 4.05V");
			charging_enable = true;
		}

	//mote og efm32 i deep sleep/em4 ved ekstrm lav spenning - hard reboot når batteri er OK
	if (battery_voltage < batt_3_50) // batterispenning lavere enn 3.5V - 24554
		{

			if(!prev_low_batt_volt){

				prev_low_batt_volt = true;
				if(gl_debug_on_UART1)printf("\nbattery_charge_status() - low battery - requesting sleep mode on next iteration");

			}else{

				if(gl_debug_on_UART1)printf("\nbattery_charge_status() - requesting sleep mode as battery voltage < 3.50V");

				gl_mote_sleep = true;

				dust_mote_sleep();

				ACMP_IntDisable(ACMP0, ACMP_IF_EDGE);
				ACMP_IntDisable(ACMP1, ACMP_IF_EDGE);
				ACMP_Reset(ACMP0);
				DAC_Reset(DAC0);
				ADC_Reset(ADC0);

				preamp_reset();

				dust_close_any_sockets();
			}

		}
	else if (gl_mote_sleep & battery_voltage > batt_3_70) // batterispenning høyere enn 3.7V - 26301
		{

			prev_low_batt_volt =false;

			if(gl_debug_on_UART1)printf("\nbattery_charge_status() - system reset as battery voltage > 3.7V");

			if (HW_REVITION >= 6)
			{
				//dust_mote_reset();
				efm32_reset();

			}else{
				external_hard_reset();
			}

		}
	else {

			prev_low_batt_volt =false;

		}
	//setter ladestatus pin
	if (5000 < logger_temperature) //temperatur høyere enn 50 celsius
		{

			GPIO_PinModeSet(gpioPortC,7,gpioModeWiredAnd,0); /*charge enable - mcp73811   1=on 0=off*/
			return 0;
		}
	else if (1000 > logger_temperature)  //temperatur lavere enn 10 celcius
		{

			GPIO_PinModeSet(gpioPortC,7,gpioModeWiredAnd,0); /*charge enable - mcp73811   1=on 0=off*/
			return 0;
		}
	else if (charging_enable == false) //batteriet er i utladingsyklus 4.15 -> 4.05
		{
			GPIO_PinModeSet(gpioPortC,7,gpioModeWiredAnd,0); /*charge enable - mcp73811   1=on 0=off*/
			return 0;
		}
	else
		{
			GPIO_PinModeSet(gpioPortC,7,gpioModeWiredAnd,1); /*charge enable - mcp73811   1=on 0=off*/
			return 1;
		}

}

void single_comp_config(ACMP_TypeDef *const acmp, const ACMP_Channel_TypeDef neg_sel, const ACMP_Channel_TypeDef pos_sel, const unsigned int vddLevel)
{
	ACMP_Init_TypeDef acmpInit = ACMP_INIT_DEFAULT;

	acmpInit.enable = true;
	acmpInit.lowPowerReferenceEnabled = false;
	acmpInit.hysteresisLevel = acmpHysteresisLevel0;
	acmpInit.warmTime = acmpWarmTime512;
	acmpInit.interruptOnRisingEdge = true;
	acmpInit.vddLevel = vddLevel;

	//GPIO_PinModeSet(gpioPortE, gpio_pin,gpioModePushPull,1);
	//ACMP_GPIOSetup(acmp,1,true,false);

	ACMP_Init(acmp, &acmpInit);
	ACMP_ChannelSet(acmp, neg_sel, pos_sel); // acmpChannel0);

	ACMP_Enable(acmp);
	ACMP_IntClear(acmp,3);

}

void comp_config(const uint32_t *const trig_levels, const ACMP_Channel_TypeDef *const pos_sels)
{


	if(gl_comp_ref_64_ladder){

		single_comp_config(ACMP0, acmpChannelVDD, pos_sels[0], 1);

	}else{

		DAC_Init_TypeDef dacInit = DAC_INIT_DEFAULT;
		DAC_InitChannel_TypeDef dchInit = DAC_INITCHANNEL_DEFAULT;

		dacInit.reference = dacRef2V5;
		dacInit.outMode = dacOutputPinADC;
		dacInit.convMode = dacConvModeContinuous;

		dchInit.enable = true;

		DAC_Init(DAC0, &dacInit);
		DAC_InitChannel(DAC0, &dchInit, 0);
		DAC_InitChannel(DAC0, &dchInit, 1);

		DAC0 -> CH0DATA = trig_levels[0];
		DAC0 -> CH1DATA = 0;

		//GPIO_PinModeSet(gpioPortB,12,gpioModePushPull,0);
		//GPIO_PinModeSet(gpioPortB,12,gpioModeWiredAnd,0);
		single_comp_config(ACMP0, acmpChannelDAC0Ch0, pos_sels[0], 1);
		//single_comp_config(ACMP1, acmpChannelDAC0Ch0, pos_sels[1], 3);

	}
}

void trigg_ref_set(const uint32_t value)
{

	DAC_Init_TypeDef dacInit = DAC_INIT_DEFAULT;
	DAC_InitChannel_TypeDef dchInit = DAC_INITCHANNEL_DEFAULT;

	dacInit.reference = dacRef2V5;
	dacInit.outMode = dacOutputPin;
	dacInit.convMode = dacConvModeContinuous;
	dchInit.enable = true;

	DAC_Init(DAC0, &dacInit);
	DAC_InitChannel(DAC0, &dchInit, 1);

	DAC0 -> CH1DATA = value;

}

void trigg_ref_reset(void){

	//trigg_ref_set(0);
	DAC_Reset(DAC0);
	GPIO_PinModeSet(gpioPortB,12,gpioModeWiredAnd,0); //testing GPIO out as preamp ref during trigg
}

void uart0_config(void)
{

    /* Initialize uart0 for communication with dust networks*/

    USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

    uartInit.enable = usartEnable ;
    uartInit.parity = usartNoParity;
    uartInit.stopbits = usartStopbits1;
    uartInit.databits = usartDatabits8;
    uartInit.baudrate = 115200;

    /* Initialize Rx and Tx*/
    GPIO_PinModeSet(gpioPortE,0,gpioModeWiredAndPullUp,0); /*Tx*/
    GPIO_PinModeSet(gpioPortE,1,gpioModeInput,0); /*Rx*/

    GPIO_PinModeSet(gpioPortC,8,gpioModeInput,0); /*Tx RTS*/
    GPIO_PinModeSet(gpioPortC,9,gpioModeWiredAndPullUp,1); /*Tx CTR*/

    GPIO_PinModeSet(gpioPortA,14,gpioModeWiredAndPullUp,1); /*nTime*/

    USART_InitAsync(UART0,&uartInit);

    /* Module UART0 is configured to location 1 */
    UART0->ROUTE = (UART0->ROUTE & ~_UART_ROUTE_LOCATION_MASK) | UART_ROUTE_LOCATION_LOC1;
    /* Enable signals TX, RX */
    UART0->ROUTE |= UART_ROUTE_TXPEN | UART_ROUTE_RXPEN;

    GPIO_IntConfig(gpioPortC,8,true,true,false);


}


void uart1_config(void)
{


    /* Initialize uart1 for communication with XBee-868 PRO */
    USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

    uartInit.enable = usartEnable ;
    uartInit.parity = usartNoParity;
    uartInit.stopbits = usartStopbits1;
    uartInit.databits = usartDatabits8;
    uartInit.baudrate = 115200;


    //GPIO_PinModeSet(gpioPortB,9,gpioModePushPull ,0); /*Tx*/
    //GPIO_PinModeSet(gpioPortB,10,gpioModeInput,1); /*Rx*/


    USART_InitAsync(UART1,&uartInit);

    /* Module UART0 is configured to location 1 */
    UART1->ROUTE = (UART1->ROUTE & ~_UART_ROUTE_LOCATION_MASK) | UART_ROUTE_LOCATION_LOC2;
    /* Enable signals TX, RX */
    UART1->ROUTE |= UART_ROUTE_TXPEN | UART_ROUTE_RXPEN;


}

void rtc_config(void)
{
	// Changed 20140729
	RTCDRV_Setup(cmuSelect_LFRCO, cmuClkDiv_32); // Written off rtcdrv.c
	// Original code:
//    RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;
//
//	rtcInit.comp0Top = true;
//	rtcInit.debugRun = true;
//	rtcInit.enable = false;
//
//	RTC_Init(&rtcInit);
//
}

// May depend on setup of clocks done in burtc_config() (not tested).
void wdog_config()
{
	// Adapted from doc:
	WDOG_Init_TypeDef init = {
		true,               /* Start watchdog when init done */                                      \
		false,              /* WDOG not counting during debug halt */                                \
		true,               /* WDOG counting when in EM2 */                                      \
		true,               /* WDOG counting when in EM3 */                                      \
		false,              /* EM4 can be entered */                                                 \
		false,              /* Do not block disabling LFRCO/LFXO in CMU */                           \
		false,              /* Do not lock WDOG configuration (if locked, reset needed to unlock) */ \
		wdogClkSelULFRCO,   /* Select 1kHZ WDOG oscillator */                                        \
		wdogPeriod_256k     /* Set longest possible timeout period */                                \
    };

	WDOG_Init(&init);
}

