
/* This code is sued for starting continous ranging on SRF02 sensor
 * 
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */

 
 
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <math.h>

#include "redpitaya/rp.h"
#include "_kiss_fft_guts.h"
#include "kiss_fft.h"
#include "kiss_fftr.h"



#define I2C_SLAVE_FORCE 		   0x0706
#define I2C_SLAVE    			   0x0703    /* Change slave address            */
#define I2C_FUNCS    			   0x0705    /* Get the adapter functionality */
#define I2C_RDWR    			   0x0707    /* Combined R/W transfer (one stop only)*/


#define	V_SONIC_WAVE				343.2		// [m/s] Schallgeschwindigkeit in Luft


// global constant variables
const char 		params[3]				= {'D','C','?'};
const uint32_t	ADC_DECIMATION			= 64;					// [-]
const uint32_t	ADC_SAMPLE_RATE			= 125000000;			// [Hz]
const uint32_t	ADC_TIME_PER_SAMPLE		= 8;					// [ns]
const int32_t	ADC_DELAY_OFFSET		= -50;					// [samples]				--> this value was evaluated empiricaly
const uint32_t	SONIC_SPEED_CM_per_SEC	= (uint32_t)(V_SONIC_WAVE * 100);				// [cm/s] --> 343,2 m/s * 100 = 34.200 cm/s
const uint32_t	MIN_DISTANCE_CM			= 30;					// [cm] 


// global variables
volatile uint8_t		param_decimation= 0;
volatile kiss_fftr_cfg	fft_cfg;


// für 30 cm: (2 * 30 cm) / 34.320 cm/s = 0,1748*10^-3 s = 1,748 ms 

/*
*
*	LD_LIBRARY_PATH=/opt/redpitaya/lib ./iic
*
*/
static void init_adc(void);
static int8_t translate_argument( char **argument);


int main(int argc, char **argv)
{
	
	// variable for IIC
	int						address		= 0x70;					// Address of the SRF08 shifted right 1 bit
	unsigned char			buf[10];							// Buffer for data being read/ written on the i2c bus
	char					*fileName	= "/dev/i2c-0";			// Name of the port we will be using	
	int fd;														// File descriptor
	unsigned int 			n			= 0;					// count of repeating meassurements
	rp_acq_trig_state_t 	state;
	
	
	// variables for measurement
	uint32_t			adc_wait_us			= ( ( ADC_BUFFER_SIZE * (	ADC_TIME_PER_SAMPLE * ADC_DECIMATION )) / 1000 );
	uint32_t			adc_delay_sample	= (( (MIN_DISTANCE_CM * 2 * ( ADC_SAMPLE_RATE / ADC_DECIMATION )) / SONIC_SPEED_CM_per_SEC ) + ADC_DELAY_OFFSET );
	uint32_t			data_buf_size		= (uint32_t)ADC_BUFFER_SIZE;		// 16.384
	uint32_t			data_cnt_write		= data_buf_size;
	uint32_t			trig_pos			= 0;
	float				*data_buf			= (float *)calloc( data_buf_size, sizeof(float));
	kiss_fft_scalar		*fft_buf_in			= (int32_t *)calloc( data_buf_size, sizeof(int32_t));
	kiss_fft_cpx		*fft_buf_out		= (kiss_fft_cpx *)calloc( data_buf_size, sizeof(kiss_fft_cpx));
	FILE				*datei				= NULL;
	//double				distance			= 0.0;
	//double				distance_faktor		= (double)( ( (double)SONIC_SPEED_CM_per_SEC / 2.0) / (double)( ADC_SAMPLE_RATE / ADC_DECIMATION ) ) ;
	char				FILE_NAME[]				= "/tmp/ram/data_n.txt";
	char 				*p;
	
	// variables for FFT
	fft_cfg = kiss_fftr_alloc(data_buf_size,0,NULL,NULL);
	
	
	if( fft_cfg == NULL || fft_buf_in == NULL || fft_buf_out == NULL ){
		return(-2);
	}
	
	
	// Arguments will be evaluated later
	if( argc > 1){
		translate_argument(argv);
	}
	else{
		// if no arguments are present, the default
		// values will be chosen
	}
	
	
	if ((fd = open(fileName, O_RDWR)) < 0) {						// Open port for reading and writing
		printf("Cannot open i2c port\n");
		exit(1);
	}

	if (ioctl(fd, I2C_SLAVE_FORCE, address) < 0) {					// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}
	
	
	
	// pointer p is used to name the output file:
	// "/tmp/ram/data_n.txt" where n will be: 0-9
	p	= (char *)(FILE_NAME + 14);
	
	
	// save header information into the file
	// for later meta-data Info for data evaluation
		/*fprintf(datei, "f_abtast: %i Hz, d_s: %.3f µm, s_min: %i cm, data_recording_time: %i µs\n\n",\
			(uint32_t)(ADC_SAMPLE_RATE / ADC_DECIMATION),\
			(double)(1e6 / (ADC_SAMPLE_RATE / ADC_DECIMATION)),\
			(uint32_t)MIN_DISTANCE_CM,\
			(uint32_t)adc_wait_us); */
	
	
	while( n < 5)
	{
		uint64_t	timeout_cnt;
		//float		data_tmp;
		//bool 		b_min;
		//uint16_t	minimum;
		init_adc();
		
		
		// rename "/tmp/ram/data_n.txt" to "/tmp/ram/data_p.txt"
		*p	= '0' + n++;
		
		
		// open the file for data saving
		datei = fopen(FILE_NAME,"w+");
		
		
		// proof if file exists
		if(datei == NULL){
			printf("Error at file opening - %s\n", FILE_NAME);
			return(-1);
		}
		
		
		
		// output only for debugging
		// printf("adc delay samples: %u\n", adc_delay_sample);
		// printf("adc wait after trigger: %u\n", adc_wait_us);
		
		buf[0] = 0;
		buf[1] = 81;
		
		
		// Set internal trigger delay
		rp_AcqSetTriggerDelay( adc_delay_sample );
		
		
		// "reset" the state of the trigger BEFORE starting the measurment
		state = RP_TRIG_STATE_WAITING;
		
		
		// I²C programming - start ultrasonic
		if ((write(fd, buf, 2)) != 2) {
			printf("Error writing to i2c slave\n");
			exit(1);
		}
		
		// ADC is started and waits for trigger
		rp_AcqStart();
		
		timeout_cnt = 1e5;
		
		// stay in loop while trigger is not activated
		while( state == RP_TRIG_STATE_WAITING && ( (--timeout_cnt) > 0 ) ){
			rp_AcqGetTriggerState(&state);
		}
		
		
		// wait until buffer is full @ 1,953 Msps (8,388 ms)
		// after trigger was activated
		usleep(adc_wait_us);
		rp_AcqStop();
		
		
		// we need the position of the pointer at trigger moment
		rp_AcqGetWritePointerAtTrig( &trig_pos );
		
		
		// get data out of ADC buffer.
		// data is read out from the point of triggering PLUS
		// the delay time, which the ultrasonic needs to exceed 60 cm (30cm distance from object)
		rp_AcqGetDataV(RP_CH_1, (trig_pos + adc_delay_sample ), &data_buf_size, data_buf );
		
		
		
		
		// ###################################
		// Aufruf FFT
		// ###################################

		
		// search the maximum
		// needed for normalization
		for( uint16_t ix=0; ix < data_cnt_write; ix++){
			fft_buf_in[ix] = (int32_t)( data_buf[ix] * 1000 );
		}
		
		kiss_fftr( fft_cfg, fft_buf_in, fft_buf_out );
		
		for( uint16_t ix=0; ix < data_cnt_write; ix++){
			//fprintf(datei, "%i\t%i\n",(fft_buf_out[ix].r), (fft_buf_out[ix].i ) );
			fprintf(datei, "%i\n", (int16_t)( data_buf[ix] * 1000) );
			
		}
		
		
		
		
		// if all n-measurements are ready, the file can be closed
		fclose(datei);
		
		// show the name of the produced file
		printf("\"%s\"\n", FILE_NAME);
		
		// calculate the distance of the measured object
		/*distance = distance_faktor * minimum + (double)( MIN_DISTANCE_CM );
		distance = (double)( (uint32_t)( distance * 10.00 ) / 10.00 );
		if( distance > 30.0){
			printf("%.2f cm\n", distance);
		}
		else{
			printf("Kein Objekt vorhanden!\n");
		}*/
		
		usleep( 500000 );
		
	}
	
	
	
	
	free(fft_buf_in);
	free(fft_buf_out);
	free(data_buf);
	rp_Release();


    return EXIT_SUCCESS;
}



static void init_adc(void)
{
	rp_Init();
	rp_DpinSetState( RP_DIO0_P, RP_LOW);							// DIO0 set to low
	rp_DpinSetDirection( RP_DIO0_P, RP_IN);							// DIO0 as input
	//rp_AcqSetDecimation( RP_DEC_64 );								// Decimation 64 --> 64 * 8 ns = 512 ns / sample
	rp_AcqSetSamplingRate( RP_SMP_1_953M );							// Sample rate 1.953Msps; Buffer time length 8.388ms; Decimation 64
	rp_AcqSetTriggerSrc( RP_TRIG_SRC_EXT_PE );						// Trigger set to external trigger positive edge (DIO0_P pin)
	rp_AcqSetArmKeep(true);
	rp_AcqStop();
}



static int8_t translate_argument( char **argument)
{
	int8_t ret = -1;
	char *p = (char *)argument[1];
	
	if( *p == '-' ){
		
		// ein Parameter in Form von "-x" wurde angegeben
		if( (*(++p) >= '0') && (*p <= '9') ){
			( *p -= 'a' - 'A' );
		}
		
		for( uint8_t ix=0; ix < sizeof(params)/sizeof(char); ix++){
			if( *p == params[ix] ){
				ret = ix;
				break;
			}
		}
		
		switch( *p ){
			case 'C':
				printf("c: %s\n",argument[1]);
				break;
			case 'D':
				printf("d: %s\n",argument[1]);
				break;
			default:
				printf("Error in parameter %s --> Possible parameters are:\n", argument[1]);
				for( uint8_t ix=0; ix < sizeof(params)/sizeof(char); ix++){
					printf("\t-%c\n", (char)params[ix]);
				}
			break;
		}
	}
	return(ret);
}


