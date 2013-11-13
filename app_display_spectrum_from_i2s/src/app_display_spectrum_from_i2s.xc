
#include <platform.h>
#include <xs1.h>
#include <xs1_su.h>
#include <math.h>
//#define XSCOPE_DEBUG
#ifdef XSCOPE_DEBUG
#include <xscope.h>
#endif //XSCOPE_DEBUG

#include "i2s_master.h"
#include "app_global.h"
#include "ports.h"

#include "lcd.h"
#include "sdram.h"
#include "display_controller.h"
#include "fft.h"
#include "level_meter.h"

on tile[0] : lcd_ports lcdports = {
  XS1_PORT_1I, XS1_PORT_1L, XS1_PORT_16B, XS1_PORT_1J, XS1_PORT_1K, XS1_CLKBLK_1 };
on tile[0] : sdram_ports sdramports = {
  XS1_PORT_16A, XS1_PORT_1B, XS1_PORT_1G, XS1_PORT_1C, XS1_PORT_1F, XS1_CLKBLK_2 };

//#define SAMP_FREQ 40000		// sampling frequency for ADC inputs
#define FFT_POINTS 256	// Number of signal samples chosen for FFT computation. It is double the level meter bands.
#define FFT_SINE sine_256	// Sine wave selected for FFT computation
#define LEV_METER_BANDS FFT_POINTS/4 	// Number of FFT points to be displayed
#define DIV_FACTOR	512 //To factor the quantizayion in order to display in the LCD screen; default: 1

#define LOG_SPEC 0	// Set to 1 if log spectrum is taken; 0 otherwise
#if LOG_SPEC
#define MAX_FFT 70		// FFT limit on the display
#else
#define MAX_FFT 100
#endif

#define FFT_FULL_USE 0	// FFT computation in full use
#if (!FFT_FULL_USE)
#define FFT_UPDATE_RATE 10 	// Number of times FFT computation is done in a second
#endif

void audio_hw_init(unsigned);
void audio_hw_config(unsigned samFreq);

#ifdef XSCOPE_DEBUG
void xscope_user_init(void) {
      xscope_register(2,
                      XSCOPE_CONTINUOUS, "ADC-DAC",
                      XSCOPE_UINT, "adc-dac",
                      XSCOPE_CONTINUOUS, "M_S",
                      XSCOPE_UINT, "mag_spectrum");
}

void output_data_adc_dac(unsigned int data_value_1) {
  xscope_int(0, data_value_1);
}

void output_data_mag_spec(unsigned int data_value_1) {
  xscope_int(1, data_value_1);
}
#endif //XSCOPE_DEBUG

void magnitude_spectrum(unsigned sig1[], unsigned sig2[], unsigned magSpectrum[])
{
	int  im[FFT_POINTS];
	int sig[FFT_POINTS];

	// Mixing signals
	for (int i=0; i<FFT_POINTS; i++)
	{
		//sig1[i] += sig2[i];
		sig[i] = sig1[i] + sig2[i];
		im[i] = 0;
	}
#if 0
	// FFT
	fftTwiddle(sig1, im, FFT_POINTS);
	fftForward(sig1, im, FFT_POINTS, FFT_SINE);

	// Magnitude spectrum
	for (int i=0; i<FFT_POINTS; i++){
		magSpectrum[i] = sig1[i]*sig1[i] + im[i]*im[i];
#if LOG_SPEC
		magSpectrum[i] = (magSpectrum[i]>0)? 10*log(magSpectrum[i]):0;
#endif
#else
		// FFT
		fftTwiddle(sig, im, FFT_POINTS);
		fftForward(sig, im, FFT_POINTS, FFT_SINE);

		// Magnitude spectrum
		for (int i=0; i<FFT_POINTS; i++){
			magSpectrum[i] = sig[i]*sig[i] + im[i]*im[i];
	#if LOG_SPEC
			magSpectrum[i] = (magSpectrum[i]>0)? 10*log(magSpectrum[i]):0;
	#endif
#endif
	}

}


enum command {GET_SIG};
interface app_sigSamp_interface {
	void get_signal(unsigned cmd, unsigned sig1[], unsigned sig2[]);
};

void app(chanend c_dc, interface app_sigSamp_interface client c)
{
  unsigned frBufIndex=0, frBuf[2];
  unsigned sig1[FFT_POINTS], sig2[FFT_POINTS];
  unsigned magSpec[FFT_POINTS];
  unsigned plotTime;
  timer t;
  unsigned max_mag_spec_val = 0;

   // Create frame buffers
  frBuf[0] = display_controller_register_image(c_dc, LCD_ROW_WORDS, LCD_HEIGHT);
  frBuf[1] = display_controller_register_image(c_dc, LCD_ROW_WORDS, LCD_HEIGHT);
  display_controller_frame_buffer_init(c_dc, frBuf[0]);

  // Get signal segment from ADC and Display spectrum periodically
  t :> plotTime;
  while (1){

	  frBufIndex = 1-frBufIndex;

	  // Get signal samples from ADC
	  c.get_signal(GET_SIG, sig1, sig2);

	  // Take magnitude spectrum of mixed signal and display it
	  magnitude_spectrum(sig1, sig2, magSpec);
	  magSpec[0] = 0;	// Set DC component to 0

#ifdef XSCOPE_DEBUG
	  for (int i=0; i<FFT_POINTS; i++) {
		//if (magSpec[i] > 0)
		output_data_mag_spec(magSpec[i]);
	  }
#endif //XSCOPE_DEBUG
	  for (int i=0; i<FFT_POINTS; i++) {
		if (max_mag_spec_val < magSpec[i])
		  max_mag_spec_val = magSpec[i];
	  }
	  level_meter(c_dc, frBuf[frBufIndex], magSpec, LEV_METER_BANDS, max_mag_spec_val/DIV_FACTOR);
	  //level_meter(c_dc, frBuf[frBufIndex], magSpec, LEV_METER_BANDS, max_mag_spec_val);
#if (!FFT_FULL_USE)
	  t when timerafter(plotTime+(XS1_TIMER_HZ/FFT_UPDATE_RATE)):> plotTime;
#endif
	  display_controller_frame_buffer_commit(c_dc,frBuf[frBufIndex]);
  }

}

#pragma select handler
void signal_sampler(streaming chanend c_data, interface app_sigSamp_interface server c)
{
  unsigned circBuf[I2S_MASTER_NUM_CHANS_ADC][FFT_POINTS];
  unsigned circBufPtr=0;
  /* Audio sample buffers */
  unsigned sampsAdc[I2S_MASTER_NUM_CHANS_ADC];
  unsigned sampsDac[I2S_MASTER_NUM_CHANS_DAC];
  unsigned buf_indexer = 0;

  /* Samples init */
  for (int i = 0; i < I2S_MASTER_NUM_CHANS_ADC; i++)  {
      sampsAdc[i] = 0;
  }
  for (int i = 0; i < I2S_MASTER_NUM_CHANS_DAC; i++)  {
      sampsDac[i] = 0;
  }

  while (1) {
	  select {
		  // Send signal samples
		  case c.get_signal(unsigned cmd, unsigned sig1[], unsigned sig2[]):
			  for (int i=0; i<FFT_POINTS; i++){
				  sig1[i] = circBuf[0][circBufPtr];
				  sig2[i] = circBuf[1][circBufPtr];
				  circBufPtr = (circBufPtr+1)%FFT_POINTS;
			  }
		  break;

		  default:
	        /* Receive ADC samples from audio thread */
#pragma loop unroll
	        for (int i = 0; i < I2S_MASTER_NUM_CHANS_ADC; i++) {
	          c_data :> sampsAdc[i];
	        }

#pragma loop unroll
	        /* Send out DAC samples */
	        for (int i = 0; i < I2S_MASTER_NUM_CHANS_DAC; i++) {
	          c_data <: sampsDac[i];
	        }

	        /* Do some processing - currently just loop back ADC to DAC on all channels */
	        for(int i = 0; i < I2S_MASTER_NUM_CHANS_DAC; i++, circBufPtr = (circBufPtr+1)%FFT_POINTS)  {
	          if(i < I2S_MASTER_NUM_CHANS_ADC) {
		        sampsDac[i] = sampsAdc[i];
		        circBuf[buf_indexer][circBufPtr] = sampsAdc[i]/(DIV_FACTOR*DIV_FACTOR);
		        buf_indexer++;
		        buf_indexer = buf_indexer%I2S_MASTER_NUM_CHANS_ADC;
#ifdef XSCOPE_DEBUG
		        output_data_adc_dac(sampsDac[i]);
#endif //XSCOPE_DEBUG
	          }
	        }
		  break;
	  }
  }
}

int main(){
	chan c_dc, c_lcd, c_sdram;
	streaming chan c_data;
	interface app_sigSamp_interface c;

	par {
		on tile[0]: app(c_dc, c);
		on tile[0]: display_controller(c_dc,c_lcd,c_sdram);
		on tile[0]: lcd_server(c_lcd,lcdports);
		on tile[0]: sdram_server(c_sdram,sdramports);
		on tile[0]: signal_sampler(c_data, c);

        on tile[1] :
        {
            unsigned mclk_bclk_div = MCLK_FREQ/(SAMP_FREQ * 64);
            audio_hw_init(mclk_bclk_div);
            audio_hw_config(SAMP_FREQ);
            i2s_master(i2s_resources, c_data, mclk_bclk_div);
        }
        //on tile[1] : loopback(c_data);
	}

	return 0;
}

