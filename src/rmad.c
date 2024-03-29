
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "em_device.h"
#include "em_chip.h"

#include "em_emu.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "em_dac.h"
#include "em_acmp.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_rtc.h"
#include "em_wdog.h"
#include "em_int.h"
#include "em_common.h" // for EFM32_MIN(), needed at least for debugging
#include "segmentlcd.h"
#include "ngi_rtcdrv.h"
#include "wait_burtc.h"

#include "dust_com.h"
#include "config.h"
#include "flash.h"
#include "process.h"
#include "packet.h"
#include "adjustable_params.h"
#include "debug_printf.h"
#include "rmad.h"
#include "aux_data.h"

#include "retargetserial.h"


// Some interesting parameters you may play around with:
#define IF_IS_SAFE // See coment in main(). Pretty sure it is OK to define.
#define RESET_MOTE_AT_START // OK defined and undefined, but probably best defined. Otherwise the socket bind likely fails and we get a reset anyway, after some time.
#define MOTE_REPORT_MIN_INTERVAL 900 // seconds. Too often => Perhaps lots of battery use (on mote). Also used for system check and reporting.
#define MOTE_CHECK_MIN_INTERVAL 259200 // seconds. Too often => Perhaps lots of battery use (on mote). Also used for system check and reporting.
#define MIN_WAIT_MS_EM  1  // 1 ms, > 0 => be sure not to (almost ?) busy-loop. I am guessing 0 is also OK.
// Seemed like we did, maybe because of some strange unknown interrupts,
// or maybe UART interrupts (but didn't break on breakpoint?), or maybe some bug.
// UPDATE: The bug seems to have been in process_ringbuf_entries(), the while loop, now fixed,
// and setting this to e.g. 10 ms is too much for the AD interrupts, or shouldn't be, but ran slowly.
// UPDATE 2: Runs slowly because it takes time to send commands or maybe rather wait for ack.
// It goes quickly e.g. when starting AD, many interrupts, slowly when AD is turned off.
#define MAX_WAIT_MS_EM1 1000 // 100 ms in EM1, MUST BE MUCH LOWER THAN 256 SECONDS! Watchdog!
#define MAX_WAIT_MS_EM2 1000 // 60 seconds, MUST BE MUCH LOWER THAN 256 SECONDS! Watchdog!
#define MAX_WAIT_MS_EM3 1000 // 60 seconds, MUST BE MUCH LOWER THAN 256 SECONDS! Watchdog!
// #define DEBUG_AD_DATA // Strictly for debugging. Anyway disabled outside DEBUG mode
// End parameters

#define MIN(x0, x1) ((x0) < (x1) ? (x0) : (x1))

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[1])) // OK X

/*
 ****** Principles for sleep (EM) modes and how to avoid infinite sleeps/bad responses.
 *
 * The strategy is to use a "basic algorithm" that is almost 100% safe, then close
 * the remainder by never sleeping forever even when no interrupt comes, even in EM3, by using BURTC.
 *
 * Initial attempt at basic algorithm:
 *   USART/GPIO wanted sleep level are define by the GPIO ISR, which is called on both start and end transmit.
 *   COMP/ADC wanted sleep level is defined by the COMP0 ISR and by the ADC ISR when finished reading samples.
 *   The minimum sleep level of the two could then be used, however:
 *   In both cases there is a vulnerability if the final ISR call (GPIO/ADC) (setting sleep level 3, though that is not the point)
 *   after the handling of recording_start/end and messages (notifications) is finished and before the sleep is entered.
 *   Depending on whether the interrupt happens before or after the calculation of the minimum sleep level,
 *   EM1 or EM3 is entered, and the final interrupt was already performed before that, so
 *   nothing wakes up the sleep.
 *
 *   It seems exceedingly difficult to fix this in the basic algorithm, but using BURTC, in addition to the basic algorithm,
 *   we can set an interrupt to wake up after a short period. The period can be very short, e.g. 0.1 seconds or less,
 *   for EM1, thus giving good performance in the EM1 case.
 *   However, probably more likely the final interrupt happened before the calculation
 *   of the minimum sleep mode, so maybe giving a minimum of 3. Having a very short BURTC wakeup
 *   for EM3 could be quite a drain on the power supply, we need a long timeout here (last resort),
 *   and make sure our basic algorithm almost never needs the last resort.
 *   Solution: Put a loop around the heavy processing, continuing depending on whether
 *  recording_start/end have been turned on or a message is available in the packet buffer or arrived while processing was ongoing. Something like:
 *     do {
 *       < handle recording >
 *       message_received = false;
 *       message_found = <handle next message> // not necessarily the most recent
 *       // Calculate sleep_mode inside loop, both to minimize "critical region"
 *       // and to get sleep_mode == 1 even if "worst case" end-IRQ(s) (which are no longer so bad) appear(s) in the critical region.
 *       sleep_mode = min(gl_sleep_mode_recording, gl_sleep_mode_uart);
 *     } while (recording_stop || message_found || message_received); // message_received might be true if new message just came in
 *     < Critical region
 *       Actually extending from somewhere in the calc of while-termination-expr above
 *       until sleep is entered).
 *       If we (e.g.? Any other possible scenario?) sleep_mode == 3,
 *       and we here get a GPIO start interrupt, all incoming bytes, and GPIO end interrupt,
 *       all before entering sleep mode, we are in the
 *       worst case scenario where we may have to wait up to e.g. 10 seconds in EM3
 *       before servicing the message. It may be less than (e.g.) 10 seconds if a new interrupt comes during sleep.
 *       If we put anything time-consuming here, the risk of this increases, and even a COMP interrupt
 *       might cause problems if all AD interrupts afterwards are received before sleep is entered.
 *     >
 *     switch (sleep_mode) {
 *       1: EM1 max e.g. 0.1 seconds
 *       3: EM3 max e.g. 10 seconds
 *     }
 *
 * The worst case scenario above seems almost impossible to get into, but if it does,
 * BURTC or any other type of interrupt capable of exiting EM3, will eventually get us out of EM3 anyway,
 * after max. e.g. ca. 10 seconds.
 */

/****** Principles for communication with the radio
 * Input from the mote is read exclusively by UART0_RX_IRQHandler (and GPIO_ODD_IRQHandler).
 * GPIO_ODD_IRQHandler interprets the input as packages and puts them in a ring-buffer
 * of packets (including acknowledgements). When the buffer is full, the oldest package is overwritten.
 *
 * Ack'ing of incoming packets is done by the machinery in GPIO_ODD_IRQHandler,
 * while waiting for acks from the mote as needed (it is up to you) is your responsibility when writing new code.
 *
 * Any non-IRQ-called routine is allowed to write
 * to the mote, and also to block
 * waiting for ack or another packet to arrive, by calling wait_for_packet().
 * It is, however, not recommended to block for longer than necessary, since this may delay
 * servicing of incoming requests, etc.
 *
 * Used to be: The only thing IRQ routines are allowed to write, is ack's.
 * Upgraded to: ISRs (IRQ routines) may send any packets/requests. They must, however,
 * take extreme care if they wait for the ACK from the mote, the GPIO/UART ISR's must have higher (>=? Not enough I guess) prio
 * so that ack may be read.
 * Also, they must take extreme care not to destroy non-ISR code which assume e.g. that
 * gl_packet_last_sent_command is not changed from an ISR.
 * In practice: Be extremely careful if writing anything except ACKs from an ISR! Analyze the code to be sure! Or don't do it!
 *              Non-ISR code is allowed to assume that ISRs only send ACKs!
 * As of 20140912 ISRs only send ACKs as far as I know. I pledge to update this line if that changes.
 *
 * Why use an input queue instead of just a single input packet buffer like before?
 * We want to be able to wait for e.g. an ack after sending a packet. That was already possible,
 * due to the relevant IRQ routine checking whether a packet was an ack or another type of packet.
 * In effect almost a two-packet buffer as long as one packet was an ack and one was not (though
 * the ack was not kept cleanly as a packet, so for accessing return info in ack one had
 * to read a lower-level buffer. Relevant at least for socketID).
 * However, receiving more than one notification while waiting for ack would lead to all but the last to be
 * overwritten and thus discarded. This becomes relevant when we want to make pretty sure that
 * e.g. "timeIndication" and "receive" notifications are not discarded. Discarding a "receive" might cause
 * the microcontroller to miss a command to start logging!
 * Secondly (less important) the ack-handling, including using information from an ack (socketID), hopefully will be cleaner.
 */


volatile bool gl_debug_on_UART1 = true;

// Prefix "gl_" => global
// FYI In the ARM inplementation we use, 'sig_atomic_t' is 'int' as far as I can see.
volatile sig_atomic_t gl_sleep_mode_recording = 3;
volatile static sig_atomic_t gl_sleep_mode_uart = 3;
volatile static int gl_prev_received_packet_id = -1;
volatile static int gl_current_sample = 0;
volatile static int gl_current_scan = 0;
volatile int gl_num_samples = 0;
volatile int gl_num_aux = 0;


volatile static int gl_num_sample_skip = 0; //for uttesting av forkasting av de f�rste samples under tigging

volatile static unsigned char gl_RX_in[DUST_HDLC_MAX];
volatile static int gl_RX_index;

volatile static bool gl_event_recived = false;

volatile static bool gl_recording_start = false;
volatile bool gl_recording_stop = false;
volatile bool gl_recording_running = false;

volatile bool gl_wait_for_timestamp = false;

volatile bool gl_get_aux_data = false;
volatile bool gl_aux_data_is_scan = false;

volatile bool gl_aux_data_recived = false;
//sample_t aux_sample[6];

volatile static int sleep_mode;

volatile bool gl_mote_sleep = false;
volatile bool gl_transport_mode = false;

volatile bool gl_comp_ref_64_ladder = false;

volatile int aux_iteration = 0;

bool logging_is_running() {
    return gl_recording_running;
}

void ADC0_IRQHandler(void) {


if (gl_get_aux_data == true) {

	if (!gl_aux_data_is_scan){

		read_single_aux_sample();

	} else {

		read_scan_aux_sample();

	}

} else {


    static const sample_t default_value = 0;

    static scan_t scan;

    if (gl_current_sample == 0) {

    	//if(gl_debug_on_UART1)printf("\n\nADC0_IRQHandler() scan started");

        process_ringbuf_add(); // BEFORE GPIO_PinOutSet()!
        GPIO_PinOutSet(gpioPortA, 14); //clear nTime

        memset(scan, default_value, sizeof(scan));
    }

    const unsigned int ch = (ADC0 ->STATUS & 50331648) >> 24;// 2 bits, don't want to keep checking for overflow...

    const sample_t sample = (sample_t)ADC_DataScanGet(ADC0);

    if (gl_num_sample_skip>0)
    {

    	--gl_num_sample_skip; //bruker comp_trigg_levels(2) til � definere antall samples som skal droppes i starten av ett trigg/d�db�nd

    }else{
    	/*store ADC data to memory from this point*/
    	scan[ch] = sample;
    	// Samples should arrive in channel order. It is possible to lose samples.
    	// We assume that a scan is finished when we get a sample from the last channel.
    	if (ch == CONFIG_AD_NCHANS - 1) {
    		process_ringbuf_add_scan(scan);
    		++gl_current_scan;
    		memset(scan, default_value, sizeof(scan));
    	}
    }

    if (gl_current_scan <= process_log_until()) {
        ++gl_current_sample;
        gl_sleep_mode_recording = 1; // Superfluous I guess

    } else {



    	GPIO_PinOutClear(gpioPortA, 14); //get nTime end of dataset

    	preamp_set_status(gl_adjustable_params->preamp_cmp_trigg);

        ADC_Reset(ADC0);

        comp_config(gl_adjustable_params->comp_trig_levels, gl_adjustable_params->comp_pos_sels);

        if(gl_comp_ref_64_ladder){

        	trigg_ref_reset();

        }

        process_ringbuf_lock_traceset();

        if(gl_debug_on_UART1)printf("\n\nADC0_IRQHandler() scan completed");

        gl_num_samples = gl_current_scan - 1;   //calculates the number of samples in a dataset
        //gl_recording_stop = true;
        gl_recording_running = false;
        gl_current_sample = 0;
        gl_current_scan = 0;
        gl_sleep_mode_recording = 3;
        gl_wait_for_timestamp = true;

        wait(10);
        GPIO_PinOutSet(gpioPortA, 14); //clear nTime end of dataset

    }

    ADC_IntClear(ADC0, ADC_IF_SCAN);
}
}

void ACMP0_IRQHandler(void) {
    // Why introduce gl_recording_running?
    // At least because this function may be called from dust_handle_notification(),
    // but we may also use it e.g. to check whether it is safe to reset the mote.
    // Kind of makes the disabling of ACMP0 at the bottom of this function unneccessary I guess (with slight rewrite of AD ISR), but the
    // code is there, and might be good for efficiency.

    if (gl_recording_running)return;

    ACMP_IntDisable(ACMP0, ACMP_IF_EDGE);
    if(CONFIG_AD_NCHANS>=3) ACMP_IntDisable(ACMP1, ACMP_IF_EDGE);


    if(gl_comp_ref_64_ladder){

    	trigg_ref_set(2600);

    }else{

    	//DAC0 -> CH1DATA = 2048;
    	DAC0 -> CH1DATA = 2600;

    }


    preamp_set_status(gl_adjustable_params->preamp_logging);

    GPIO_PinOutClear(gpioPortA, 14); //get nTime

    ADC_Reset(ADC0);
    gl_get_aux_data = false;
    gl_aux_data_recived = false;
    gl_num_aux = 0;

    const uint32_t *const trig_levels_temp = gl_adjustable_params->comp_trig_levels;
    gl_num_sample_skip = trig_levels_temp[1] * CONFIG_AD_NCHANS;

    // According to our experience, this is needed here, not just at start of program.
    // Maybe because calls to ADC_Init and ADC_InitScan are needed.
    gl_ad_sampling_rate = AD_config();

    //wait(5);
	if(gl_debug_on_UART1)printf("\nACMP0_IRQHandler() - ad_sampling_rate = %d", gl_ad_sampling_rate/CONFIG_AD_NCHANS);

    ADC_Start(ADC0, adcStartScan);

    gl_recording_start = true;
    gl_recording_running = true;
    gl_sleep_mode_recording = 1;

}


/* Not needed/used it seems, I guess ACMP0_IRQHandler() is called both for ACMP0 and ACMP1.
 * See following link for more information, and also Yme tested.
 * http://community.silabs.com/t5/32-Bit-Discussion/missing-ACMP1-IRQn/td-p/119452
 */
//void ACMP1_IRQHandler(void) {
//    ACMP0_IRQHandler();
//}

void get_uart0_bytes() {
#if 1 // Most recent code
    // If we would have overflowed RX_in, we just throw away the bytes and bet
    // that the hardware flow control, crc-checking etc will synchronize back.
    while (UART0 ->STATUS & USART_STATUS_RXDATAV) {
        unsigned char in = USART_RxDataGet(UART0 );
        if (gl_RX_index < sizeof(gl_RX_in)) {
            gl_RX_in[gl_RX_index] = in;
            ++gl_RX_index;
        }
    }
#else
    // If using this, comment out call from GPIO_ODD_IRQHANDLER
    unsigned char in = USART_Rx(UART0);
    if (gl_RX_index < sizeof(RX_in)) {
        gl_RX_in[gl_RX_index] = in;
        ++gl_RX_index;
    }
#endif
}

void UART0_RX_IRQHandler(void) {
    // Is this ISR safe?
    // 1. It does not clear the interrupt flag, could it be called again and again...?
    // 2. It always reads one character only. What if there are several incoming characters
    //    before this ISR is called? Will the ISR be re-issued?

    // Now trying fix of these issues/questions.
    get_uart0_bytes();

    USART_IntClear(UART0, USART_IF_RXDATAV);

}

// An odd GPIO port changes.
void GPIO_EVEN_IRQHandler(void) {
#define USE_ORIGINAL
    if (GPIO_PinInGet(gpioPortC, 8) == false) {

        gl_sleep_mode_uart = 1;
#ifdef USE_ORIGINAL

        gl_RX_index = 0;
#endif

        GPIO_PinOutClear(gpioPortC, 9);

    } else {
        // Will hold both full escaped HLDC packet and then unescaped API header and payload.
        unsigned char RX_packet[DUST_HDLC_MAX];

        GPIO_PinOutSet(gpioPortC, 9);
        get_uart0_bytes(); // Get any remaining bytes in the fifo.
        // We need to disable interrupts since GPIO interrupt has a lower priority,
        // than UART0_RX_IrqHandler():
        INT_Disable();
#ifdef USE_ORIGINAL // Original code, we trust hardware flow
        const int RX_packet_length = gl_RX_index;
        (void) memcpy(RX_packet, gl_RX_in, RX_packet_length);
#else
        int RX_packet_length;
        int istart, barrier = 0, k = 0;
        for (k = 0; k < gl_RX_index; ++k) {
            if (gl_RX_in[k] == 126) {
                if (++barrier == 1)
                istart = k;
                else if (barrier == 2) {
                    RX_packet_length = k - istart + 1;
                    (void)memcpy(RX_packet, gl_RX_in+istart, RX_packet_length);
                    gl_RX_index = gl_RX_index-k-1;
                    (void)memcpy(gl_RX_in, gl_RX_in+k+1, gl_RX_index);
                    break;
                }
            }
        }

        if (barrier == 0) {
            RX_packet_length = gl_RX_index = 0;
        }
        else if (barrier == 1) {
            gl_RX_index -= istart;
            (void)memcpy(gl_RX_in, gl_RX_in+istart, gl_RX_index);
            RX_packet_length = 0;
        }


#endif
        INT_Enable();

//		//IV: Safe to just add to UART1 and never read the other end?
//		/*output to uart1 for debugging*/
//		USART_Tx(UART1, 0xaa);
//		for (int n = 0; n < RX_length; n++) {
//			USART_Tx(UART1, RX_packet[n]);
//		}
//
//		USART_Tx(UART1, 0xff);
//		USART_Tx(UART1, 0x00);
//		USART_Tx(UART1, 0xff);
//		/*END output to uart1 for debugging*/

#ifndef NDEBUG
        const int max_bytes = 9;
        PRINTF("Received packet [");
        for (int k = 0; k < EFM32_MIN(max_bytes, RX_packet_length); ++k)
            PRINT("%d,", RX_packet[k]);
        PRINT("%s]\n", (RX_packet_length > max_bytes ? "..." : ""));
        if (RX_packet[1] == 37)
            PRINTF("txDone\n");
#endif

        const int RX_length = dust_get_packet(RX_packet, RX_packet_length);

        if (RX_length >= 3) {
            const bool is_ack = (RX_packet[2] & 0x01) != 0;
            const int received_packet_id = (RX_packet[2] & 0x02) >> 1;
            const bool is_sync = (RX_packet[2] & 0x08) != 0;
            if (is_ack) {
            	//if(gl_debug_on_UART1)printf("Received ack [%lx,%lx,%lx,%lx,%lx,%lx,%lx,%lx,%lx]\n", RX_packet[0], RX_packet[1],RX_packet[2], RX_packet[3], RX_packet[4],RX_packet[5], RX_packet[6], RX_packet[7], RX_packet[8]);

                //ack_recived = true;
                packet_register(RX_packet, RX_length);

                //if(gl_debug_on_UART1)printf("Packet length: [%d]\n", RX_packet_length);

                if(RX_packet_length>=13)if(RX_packet[4]==1 | RX_packet[4]==13)if(gl_debug_on_UART1)printf("Mac Address: 0%lx%lx0%lx0%lx0%lx%lx%lx%lx\n", RX_packet[5],RX_packet[6], RX_packet[7], RX_packet[8], RX_packet[9],RX_packet[10], RX_packet[11], RX_packet[12]);

            } else {
                //PRINTF("Received notification [%d,%d,%d]\n", RX_packet[0],
                  //      RX_packet[1], RX_packet[2]);
                // gl_radio_output_is_busy is replaced by INT_Disable().
//				if (!gl_radio_output_is_busy)
                dust_send_ack(RX_packet);

                // For retransmits we only send and ack (above), we don't re-register it as incoming,
                // except when the sync flag is set, since we then cannot see the difference between a newly booted
                // mote and a retransmit. A pity, since the main problem seems to be retransmit of packets
                // with sync set (or is the mote booting "thousands of times"?).
                // If we start this program with an already-running mote,
                // we should accept any initial packet_id even if sync is not on,
                // so we gove the initial gl_prev... the value -1
                if (is_sync
                        || received_packet_id != gl_prev_received_packet_id) {
                    gl_prev_received_packet_id = received_packet_id;
//
//					for (int n = 0; n < RX_length; n++) {
//						event_pack[n] = RX_packet[n];
//					}
//					event_pack_length = RX_length;
                    gl_event_recived = true;
                    packet_register(RX_packet, RX_length);
                }

            }
        } else {
            //PRINTF("Packet is invalid\n");
        }

        gl_sleep_mode_uart = 3;
    }
    GPIO_IntClear(256);//8192
}






    /**************************************************************************//**
     * @brief  Main function
     *****************************************************************************/

    int main(void) {
        /*
         * Regarding padding in structs:
         * Some places in the code we assume that structs have no padding.
         * This is done where we use structs as input or output parameters for IO from/to radio.
         * For this purpose, we use the gcc-specific "__attribute__((__packed__))" (or equivalently "attribute((packed))" I guess),
         * and for "completeness" also EFM32_PACK_START/EFM32_PACK_END.
         * We use this also for flash headers (perhaps simpler to interpret FLASH in future if padding changes in a new compiler),
         * and for cached flash header info (save RAM).
         * For radio IO we assume the packed attribute not only packs fields together,
         * but also that there is no trailing padding. The latter is important when checking
         * that packets are of the right size by comparing with sizeof(<struct>),
         * when sending packets with data after a struct (e.g. OMSG_DATA), and maybe
         * even when sending packets with nothing after the struct (depending
         * on the receiver).
         * It has, however, been difficult to interpret the gcc documentation in a way that
         * guarantees no trailing padding. However,
         *   a) See Ambroz Bizjak's answer in http://stackoverflow.com/questions/7957363/effects-of-attribute-packed-on-nested-array-of-structures
         *      Seems to know what he talks about, and also in comments to his answer mtalexan has tested it, it seems.
         *      Note that i found at least one other person claiming the opposite about trailing padding in gcc.
         *      Also note (on the positive side) that I don't think we use any pointers to packed members
         *      (as of 20140921), and anyway "Cortex-M3 and M4 allow unaligned access by default"
         *      (http://stackoverflow.com/questions/18269181/unaligned-access-causes-error-on-arm-cortex-m4)
         *   b) Just below we add a test of our own.
         *
         *   One place in the code we have already coded around the question about *trailing* padding
         *   by using offsetof(<struct>, <last member>) + sizeof(<last member>) (function send_data() using data_desc_t).
         *   I won't change that code back for the time being.
         *
         *   If we change to another compiler than gcc I guess we will get
         *   compilation errors due to "__attribute__((__packed__))".
         *
         *   Here is the packing test:
         */
        { // Scope so we don't "use up" variable e, in case we wish to use it later.
            struct __attribute__((__packed__)) {
                unsigned char a;
                uint16_t b;
                uint32_t c;
                double d;
            } e;
            _Static_assert(sizeof(e) == 1+2+4+sizeof(double), "Packing not working as assumed!");
        }


        /* Chip errata */
        CHIP_Init();

        DEBUG_INIT;
        //PRINTF("geophone.c starting\n");

        //RETARGET_SerialInit();
        //RETARGET_SerialCrLf(1);

        if(gl_debug_on_UART1){

            RETARGET_SerialInit();
            RETARGET_SerialCrLf(1);
            printf("\n\n***RMAD-EDGE EFM32GG boot***\n");
            printf("\nRMAD-Software version = %d", SW_VERTION);
            printf("\nRMAD-Hardware revision = %d", HW_REVITION);
            printf("\nRMAD-Number of AD channels = %d\n", CONFIG_AD_NCHANS);

        }



        /* Enable peripheral clocks */
        CMU_ClockEnable(cmuClock_HFPER, true);
        CMU_ClockEnable(cmuClock_ADC0, true);
        CMU_ClockEnable(cmuClock_ACMP0, true);
        if(CONFIG_AD_NCHANS>=3) CMU_ClockEnable(cmuClock_ACMP1, true);
        CMU_ClockEnable(cmuClock_DAC0, true);
        CMU_ClockEnable(cmuClock_GPIO, true);
        CMU_ClockEnable(cmuClock_UART0, true);
        //CMU_ClockEnable(cmuClock_UART1, true);
        CMU_ClockEnable(cmuClock_RTC, true);

        CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);

        // Initialize adjustable parameters stored in FLASH between runs. Run this before using any parameter.
        adjustable_params_init();

        preamp_config(gl_adjustable_params->preamp_cmp_trigg);

        GPIO_setup(); //setting up battery charge and external reset pint

        // charge_mode(1);  //testing av hurtiglading, fj�rnes n�r test er ferig

        uart0_config();
        //uart1_config();

        if(gl_comp_ref_64_ladder) trigg_ref_reset();

        comp_config(gl_adjustable_params->comp_trig_levels,gl_adjustable_params->comp_pos_sels);

        //AD_config();
        rtc_config(); // Simply calls RTCDRV_Setup().
        ADC_Reset(ADC0); // Make sure the AD is not running (only an issue during debugging, right?). Configuration is done in UART ISR.

        //if(gl_debug_on_UART1)printf("\nprocess_config() starting");
        process_config(); // Also handles the NAND FLASH
        //if(gl_debug_on_UART1)printf("\nprocess_config() finished");

        // BURTC, see main_gg_stk.c and others in AN0041
        // Everything put in burtc_config.
        // Grouping all calls together makes it simpler?
        burtc_config();

        wdog_config(); // May depend on setup of clocks done in burtc_config() (not tested).
        //WDOG_lock(); // Comment back in when debugged?

        //SegmentLCD_Init(false);

     //   ACMP_IntEnable(ACMP0, ACMP_IF_EDGE);
        NVIC_EnableIRQ(ACMP0_IRQn);
        NVIC_SetPriority(ACMP0_IRQn, 2);
     //   ACMP_IntEnable(ACMP1, ACMP_IF_EDGE);


     //   ACMP_IntDisable(ACMP0, ACMP_IF_EDGE);  // lagt til for � test om rebootloopen skyldes trigg f�r mote er aktiv
     //   ACMP_IntDisable(ACMP1, ACMP_IF_EDGE);  // lagt til for � test om rebootloopen skyldes trigg f�r mote er aktiv

        // http://community.silabs.com/t5/32-Bit-Discussion/missing-ACMP1-IRQn/td-p/119452   :
        // NVIC_EnableIRQ(ACMP1_IRQn);
        // NVIC_SetPriority(ACMP1_IRQn, 2);

        // Do we need this? Probably not.
        // Or maybe yes, see http://embeddedgurus.com/state-space/2014/02/cutting-through-the-confusion-with-arm-cortex-m-interrupt-priorities/:
        // "default priority of zero".
        // Should have lower prio (ie higher number) than UART and GPIO, since
        // in rare circumstances it may send packets to mote (could be changed if
        // we skip waiting for ack's).
        NVIC_SetPriority(ADC0_IRQn, 4);

        USART_IntClear(UART0, USART_IF_RXDATAV);
        USART_IntEnable(UART0, USART_IF_RXDATAV);
        NVIC_EnableIRQ(UART0_RX_IRQn);
        NVIC_SetPriority(UART0_RX_IRQn, 1);

        GPIO_IntClear(256); //8192
        GPIO_IntEnable(256); //8192
        NVIC_EnableIRQ(GPIO_EVEN_IRQn);
        NVIC_SetPriority(GPIO_EVEN_IRQn, 3);

        // Read any bytes in the input buffer.
        // The test code in get_uart0_bytes() should not be enabled.
        get_uart0_bytes();
        gl_RX_index = 0;



        dust_mote_setup();  //program mote with propper network config, only requiered one time but propably does not hurt on every reboot


        dust_mote_reset(); //hard or soft reset of mote, dependent on HW_REVISION



        float secs_since_mote_report = 1000; // Not exact. Used to avoid checking mote too often.
        float secs_since_mote_check = 0; // Not exact. Used to avoid checking mote too often.
        float secs_since_last_sleep = 0; // Not exact. Used to avoid checking mote too often.

        seq_num_t search_seq_num_start = 0;
        int sleep_mode;
        bool notification_found = true; // Set to true initially, at least #ifdef IF_IS_SAFE
        packet_with_meta_t packet;
        while (true) {

            /*realtime signal prosessing of ADC data (from memory) from this point
             * We loop until we have treated all incoming events, while still giving good
             * priority to recording_start/recodring_stop.
             */


        	//outer loop
            do {

                if (gl_recording_start) {

                    gl_dont_wait_for_acks = true;
                    //dust_tx_msg_data(OMSG_LOGGINGSTARTED, "", 0);
                    gl_recording_start = false;
                }

    			if (HW_REVITION >= 6)
    			{

    				process_W29N01HV_hybrid_ringbuf_entries();
    				//process_W29N01HV_ringbuf_entries();
    				//process_ringbuf_entries();

    			}else{

    				process_ringbuf_entries();

    			}




                if (gl_recording_stop) {

                	//if(gl_debug_on_UART1)printf("\nMain() gl_recording_stoped");

                    gl_dont_wait_for_acks = false;

                    const event_t lastevent = process_latest_flash_event();

                    dust_tx_msg_data(OMSG_LOGGINGSTOPPED, &lastevent, sizeof(lastevent));

                    gl_recording_stop = false;

                    ACMP_IntClear(ACMP0, ACMP_IF_EDGE);
                    ACMP_IntEnable(ACMP0, ACMP_IF_EDGE);
                    if(CONFIG_AD_NCHANS>=3) ACMP_IntClear(ACMP1, ACMP_IF_EDGE);
                    if(CONFIG_AD_NCHANS>=3) ACMP_IntEnable(ACMP1, ACMP_IF_EDGE);
                }



                // The following if-check is not necessary, it is only to improve efficiency.
                // In fact I had to think a bit on whether it is safe, but should be.
                // First time in the program execution it is obviously safe.
                // The first time in all later do-while loops are also safe, because
                // at exit of the previous do-while loop, every finished incoming notification has been processed,
                // so gl_event received <=> some unprocessed notification.
                // Loop-iteration after the first in each do-while loop:
                // If we do not enter the if-statement, we did not get a new event while searching for/processing the
                // previous notification and in fact the packet buffer had no more
                // available notifications already the previous round, so there can be no available notifications.
                // Thus it must be safe to skip.
                //

                if (gl_event_recived || notification_found) {

                    gl_event_recived = false;
                    notification_found = packet_search(&packet, .min_seq_num = search_seq_num_start, .response = false) == 0;

                    if (notification_found) {
                        search_seq_num_start = packet.seq_num + 1;

                        /*handle radio (dust network) data
                         * Any lower-level function actually implementing any possibly long-lasting handling should consider feeding the watchdog.
                         */
                        dust_handle_notification(&packet);
                    }

                } else {

                    notification_found = false;

                }


                // This code is a way to allow us to run more or less time-consuming tasks
                // only if the system is likely headed towards EM3, i. e. not too busy,
                // without jeopardizing the almost 100% safe "basic algorithm" (made 100% by BURTC).
                // Maybe not important to recalculate sleep_mode.
                int likely_sleep_mode = MIN(gl_sleep_mode_recording, gl_sleep_mode_uart);
                const bool repeat_outer_loop_likely = gl_recording_stop || notification_found || gl_event_recived;


                if ((secs_since_mote_check >= MOTE_CHECK_MIN_INTERVAL) && !gl_mote_sleep && !gl_transport_mode){

                	dust_check_and_possibly_reset_mote();
                	secs_since_mote_check = 0;

                }


                if ((secs_since_mote_report >= MOTE_REPORT_MIN_INTERVAL) && likely_sleep_mode == 3 && !repeat_outer_loop_likely && !gl_transport_mode) {
                    // Perform code which should not be performed when the system is busy.

                	get_aux_and_report();
                	//gl_sleep_mode_recording = get_aux_and_report();

                	//if(gl_debug_on_UART1)printf("\n\nmain() aux_send on interval, iteration = %d", aux_iteration);

                	if(aux_iteration == 4){ // reduce joinDutyCycle after 24 timer from boot

                		DUST_SEND({0x01, 0x02, 0x02, 0x06, 0x18}); // Set joinDutyCycle to 10%

                	}

                	aux_iteration++;
                	secs_since_mote_report = 0;

                    //sleep_mode = MIN(gl_sleep_mode_recording, gl_sleep_mode_uart); // Maybe not necessary, but OK.

                }


                if (gl_aux_data_recived == true){

                	get_new_aux_sample();
                	//gl_sleep_mode_recording = get_new_aux_sample();

                	//sleep_mode = MIN(gl_sleep_mode_recording, gl_sleep_mode_uart); // Maybe not necessary, but OK.

                }


            } while (gl_recording_stop || notification_found || gl_event_recived);


            //inner loop
            //sleep_mode = 0; //never sleep
            sleep_mode = MIN(gl_sleep_mode_recording, gl_sleep_mode_uart);
            switch (sleep_mode) {
            case 0:

                break;
            case 1:

            	secs_since_last_sleep = wait_from_to_burtc(MIN_WAIT_MS_EM, MAX_WAIT_MS_EM1, 1) / 1000.0;
                break;

            case 2:
            	secs_since_last_sleep = wait_from_to_burtc(MIN_WAIT_MS_EM, MAX_WAIT_MS_EM2, 2) / 1000.0;
                break;

            case 3:

            	secs_since_last_sleep = wait_from_to_burtc(MIN_WAIT_MS_EM, MAX_WAIT_MS_EM3, 3) / 1000.0;
                break;
            }

            secs_since_mote_report += secs_since_last_sleep;
            secs_since_mote_check += secs_since_last_sleep;
            secs_since_last_sleep = 0;

            WDOG_Feed();

        }
    }
