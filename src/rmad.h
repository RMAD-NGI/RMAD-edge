
#ifndef GEOPHONE_H_
#define GEOPHONE_H_



#include "process.h"
#include <signal.h> // Get sig_atomic_t


#define HW_CONFIGURATION 3

//1 = 4 ch single ended 215Hz - geophones (not yet implmented)
//2 = aux only - sensor node (not yet implmented)
//3 = 2 ch single ended 425Hz - railway
//4 = 2 ch differential 425Hz - infrasound (not yet implmented)

#define SW_VERTION 0

//0 = reserved for development and debugging
//5 = last offisal release for RMAD-railway and RMAD-geophone, RMAD-railway, RMAD-infrasound
//6 = intermediate test vertion for RMAD-EDGE

#define HW_REVITION 6

//4 = white development board with EFM32 starter kit,
//5 = red production board with EFM32 starter kit
//6 = black production board with EFM32 chip

#define HW_RADIO 0

//0 = SmartMeshIP
//1 = Digi NB-IoT (not yet implmented)

extern volatile bool gl_debug_on_UART1;

extern volatile int gl_num_samples;
extern volatile bool gl_wait_for_timestamp;
extern volatile bool gl_recording_stop;

extern volatile int gl_sleep_mode_recording;
extern volatile bool gl_get_aux_data;
extern volatile int gl_num_aux;
extern volatile bool gl_aux_data_recived;

extern volatile bool gl_aux_data_is_scan;

extern volatile bool gl_mote_online_check;

extern volatile bool gl_mote_sleep; //mote has been sent to deep sleep due to low battery voltage
extern volatile bool gl_transport_mode; //logger has been set to transport mode, requieres hard reset to exit

//extern sample_t aux_sample[6];



bool logging_is_running();




#endif /* GEOPHONE_H_ */
