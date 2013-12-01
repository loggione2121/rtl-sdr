// Oeta

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <unistd.h>

#include <pthread.h>
#include <libusb.h>
#include <sys/time.h>
#include "rtl-sdr.h"

// GUI
#include <cairo.h>
#include <gdk/gdkkeysyms.h>
#include <gtk/gtk.h>
#include "kiss_fft.h"

#include <wiringPi.h>

#include "oeta.h"


static pthread_t demod_thread;
static pthread_mutex_t data_ready;  /* locked when no fresh data available */
static pthread_mutex_t data_write;  /* locked when r/w buffer */
static int do_exit = 0;
static rtlsdr_dev_t *dev = NULL;
static int lcm_post[17] = {1,1,1,3,1,5,3,7,1,9,5,11,3,13,7,15,1};

static int *atan_lut = NULL;
static int atan_lut_size = 131072; /* 512 KB */
static int atan_lut_coef = 8;

// GUI
static gint width = SCREEN_WIDTH;
static gint height = SCREEN_HEIGHT;
static gboolean update_text = FALSE;
static gboolean set_center_freq = FALSE;
static gboolean menu_req = FALSE;
static gint menu_item = 0;
static gboolean hf_mode = FALSE;

float scale;
int yzero = 0;
int text_margin = 0;
uint32_t samp_rate = SAMPLE_RATE_DEFAULT;
int fft_size = SCREEN_WIDTH;
kiss_fft_cfg  fft_cfg;
kiss_fft_cpx *fft_in;
kiss_fft_cpx *fft_out;
float *log_pwr_fft; // dbFS relative to 1.0

uint32_t capture_rate = 1008000;

struct fm_state
{
	int      now_r, now_j;
	int      pre_r, pre_j;
	int      prev_index;
	int      downsample;    /* min 1, max 256 */
	int      post_downsample;
	int      output_scale;
	int      squelch_level, conseq_squelch, squelch_hits, terminate_on_squelch;
	int      exit_flag;
	uint8_t  buf[MAXIMUM_BUF_LENGTH];
	uint32_t buf_len;
	int      signal[MAXIMUM_BUF_LENGTH];  /* 16 bit signed i/q pairs */
	int16_t  signal2[MAXIMUM_BUF_LENGTH]; /* signal has lowpass, signal2 has demod */
	int      signal_len;
	int      signal2_len;
	FILE     *file;
	uint32_t freq;
	uint32_t freq_offset;
	uint32_t sample_rate;
	int      output_rate;
	int      fir_enable;
	int      fir[256];  /* fir_len == downsample */
	int      fir_sum;
	int      custom_atan;
	int      deemph, deemph_a;
	int      now_lpr;
	int      prev_lpr_index;
	int      dc_block, dc_avg;
	void     (*mode_demod)(struct fm_state*);
	char	*mode_name;
	uint32_t step;
	int	 gain;
};

struct fm_state fm;

// Encoder
struct encoder
{
    int pin_a;
    int pin_b;
    volatile long value;
    volatile int lastEncoded;
    int divider;
} freq_encoder;

unsigned int last_interrupt_time = 0;

int gain_array[GAIN_ARRAY_SIZE] = {0, 15, 40, 65, 90, 115, 140, 165, 190, 215, 240, 290, 340, 420};
int gain_array_ptr = GAIN_ARRAY_INITIAL;
int step_array[STEP_ARRAY_SIZE] = {5000, 6250, 8330, 10000, 12500, 20000, 25000, 30000, 50000, 100000, 125000, 250000, 500000, 1000000};
int step_array_ptr = STEP_ARRAY_INITIAL;

void *set_center_freq_thread() {

	//int capture_freq, capture_rate;
	
	while(!do_exit) {
		usleep(DEBOUNCE_TIME_MILLI);
		if (set_center_freq) {
			//capture_rate = fm.downsample * fm.sample_rate;
			//capture_freq = fm.freq + capture_rate / 4;					
			//rtlsdr_set_center_freq(dev, capture_freq);
			rtlsdr_set_center_freq(dev, fm.freq);
			set_center_freq = FALSE;
		}
	}
	return NULL;
}

unsigned int millis (void)
{
  struct timeval tv ;
  uint64_t now ;

  gettimeofday(&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

  return (uint32_t)(now) ;
}

static void update_sample_rate()
{
        int r;

        fm.downsample = (1000000 / fm.sample_rate) + 1;
        capture_rate = fm.downsample * fm.sample_rate;
        
	fm.output_scale = (1<<15) / (128 * fm.downsample);
        if (fm.output_scale < 1) {
                fm.output_scale = 1;}
        fm.output_scale = 1;

        // Set the sample rate
        fprintf(stderr, "Capture rate %u Hz.\n", capture_rate);

        if (fm.output_rate > 0) {
                fprintf(stderr, "Output rate: %u Hz.\n", fm.output_rate);
        } else {
                fprintf(stderr, "Output rate: %u Hz.\n", fm.sample_rate/fm.post_downsample);}
        
	r = rtlsdr_set_sample_rate(dev, (uint32_t)capture_rate);
        if (r < 0) {
                fprintf(stderr, "WARNING: Failed to set sample rate.\n");}

}

void ISR_menu_select()
{

	unsigned int interrupt_time = millis();

   	if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME_MILLI) // Debounce 
   	{
        	if (!menu_req) {

                	menu_req = !menu_req;
                	menu_item = 0;
        	} else {
	
        	        if (menu_item > (MENU_ITEM_MAX - 1)) {
                	        menu_req = !menu_req;
                	} else {
                        	menu_item += 1;
                	}
        	}
	}
	last_interrupt_time = interrupt_time;
}

void ISR_hf_select()
{

        unsigned int interrupt_time = millis();

        if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME_MILLI) // Debounce
        {

		hf_mode = !hf_mode;

     		if (hf_mode) {
                	fm.freq = FREQ_INITIAL_HF;
			fm.sample_rate = SAMPLE_RATE_DEFAULT;
			update_sample_rate();
  	                fm.freq_offset = UPCONVERTER_CRYSTAL;
        	        fm.mode_demod = &am_demod;
                	fm.mode_name = "AM";
			
                	rtlsdr_set_tuner_gain_mode(dev, GAIN_MANUAL);
                	gain_array_ptr = GAIN_AM_PTR;
                	rtlsdr_set_tuner_gain(dev, gain_array[gain_array_ptr]);
        	} else {
                	fm.freq = FREQ_INITIAL_NORM;
			fm.sample_rate = SAMPLE_RATE_WFM;
			update_sample_rate();
                	fm.freq_offset = 0;
                	fm.mode_demod = &fm_demod;
                	fm.mode_name = "WFM";
			
                	gain_array_ptr = GAIN_AUTO_PTR;
                	rtlsdr_set_tuner_gain_mode(dev, GAIN_AUTO);
        	}
		
		update_text = TRUE;
		set_center_freq = TRUE;		
        }
	last_interrupt_time = interrupt_time;

}


void ISR_frequency_encoder()
{

	usleep(DEBOUNCE_TIME_MILLI);

	int MSB = digitalRead(freq_encoder.pin_a);
        int LSB = digitalRead(freq_encoder.pin_b);

	int encoded = (MSB << 1) | LSB;
        int sum = (freq_encoder.lastEncoded << 2) | encoded;

	freq_encoder.lastEncoded = encoded;

        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {

		if (freq_encoder.divider > FREQ_DIVIDER - 1) {
		if (menu_req ) {
			if (menu_item == MENU_MODE) {
	
				switch (fm.mode_name[0])
				{
					case 'L':
						fm.mode_name = "USB";
						fm.mode_demod = &usb_demod;
						update_text = TRUE;
						break;
					case 'U':
						fm.mode_name = "AM";
						fm.mode_demod = &am_demod;
						update_text = TRUE;
						break;
					case 'A':
						fm.mode_name = "WFM";
						fm.mode_demod = &fm_demod;
						update_text = TRUE;
						break;
					case 'W':
						fm.mode_name = "LSB";
						fm.mode_demod = &lsb_demod;
						update_text = TRUE;
						break;
					default:
						update_text = FALSE;
						break;
				}
	
			} else if (menu_item == MENU_STEP) {

				if (step_array_ptr > 0) {
					step_array_ptr--;
					fm.step = step_array[step_array_ptr];
				}
			} else if (menu_item == MENU_GAIN) {
				if (gain_array_ptr > 0) {
					gain_array_ptr--;
					fm.gain = gain_array[gain_array_ptr];

					if (gain_array_ptr == 0) {
						rtlsdr_set_tuner_gain_mode(dev, GAIN_AUTO);
					} else {
						rtlsdr_set_tuner_gain_mode(dev, GAIN_MANUAL);
						rtlsdr_set_tuner_gain(dev, gain_array[gain_array_ptr]);
					}	
				}
			}
        	} else {
			if (fm.freq_offset == 0) {
				if (fm.freq > (FREQ_MIN - OFFSET_CORRECTION + fm.step)) {
					fm.freq -= fm.step;
					set_center_freq = TRUE;
				}
			} else {
				if (fm.freq > (UPCONVERTER_CRYSTAL - OFFSET_CORRECTION + fm.step)) {
					fm.freq -= fm.step;
					set_center_freq = TRUE;
				}
			}
 		}

		
		freq_encoder.divider = 0;
		} else {
		freq_encoder.divider++;
		}
		freq_encoder.value--;
	}
        
	if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {

		if (freq_encoder.divider > FREQ_DIVIDER - 1) {
		if (menu_req) {
			if (menu_item == MENU_MODE) {

				switch (fm.mode_name[0])
				{
					case 'W':
						fm.mode_name = "AM";
						fm.mode_demod = &am_demod;
						update_text = TRUE;
						break;
					case 'A':
						fm.mode_name = "USB";
						fm.mode_demod = &usb_demod;
						update_text = TRUE;			
						break;
					case 'U':
						fm.mode_name = "LSB";
						fm.mode_demod = &lsb_demod;
						update_text = TRUE;
						break;
					case 'L':
						fm.mode_name = "WFM";
						fm.mode_demod = &fm_demod;
						update_text = TRUE;
						break;
					default:
						update_text = FALSE;
						break;
				}
	
			} else if (menu_item == MENU_STEP) {

                                if (step_array_ptr < STEP_ARRAY_SIZE - 1) {
                                        step_array_ptr++;
                                	fm.step = step_array[step_array_ptr];
				}
			} else if (menu_item == MENU_GAIN) {
				if (gain_array_ptr < GAIN_ARRAY_SIZE - 1) {
					gain_array_ptr++;
					fm.gain = gain_array[gain_array_ptr];

                                        rtlsdr_set_tuner_gain_mode(dev, GAIN_MANUAL); 
                                        rtlsdr_set_tuner_gain(dev, gain_array[gain_array_ptr]);

				}
			}
		} else {
	       		if (fm.freq < FREQ_MAX - fm.step) {
				fm.freq += fm.step;
				set_center_freq = TRUE;
			}
		}

		freq_encoder.divider = 0;
                } else {
                freq_encoder.divider++;
                }
		freq_encoder.value++;
	}


	update_text = TRUE;

}

void setup_gpios()
{

    	// Frequency encoder
    	freq_encoder.pin_a = GPIO_ROTARYA;
    	freq_encoder.pin_b = GPIO_ROTARYB;
    	freq_encoder.value = 0;
    	freq_encoder.lastEncoded = 0;
    	freq_encoder.divider = 0;

    	pinMode(freq_encoder.pin_a, INPUT);
    	pinMode(freq_encoder.pin_b, INPUT);
    	pullUpDnControl(freq_encoder.pin_a, PUD_UP);
    	pullUpDnControl(freq_encoder.pin_b, PUD_UP);
    	wiringPiISR(freq_encoder.pin_a,INT_EDGE_BOTH, &ISR_frequency_encoder);
    	wiringPiISR(freq_encoder.pin_b,INT_EDGE_BOTH, &ISR_frequency_encoder);

    	// Menu / select button
    	pinMode(GPIO_SELECT, INPUT);
    	pullUpDnControl(GPIO_SELECT, PUD_UP);
    	wiringPiISR(GPIO_SELECT, INT_EDGE_FALLING, &ISR_menu_select);
 
	// HF
	pinMode(GPIO_HF, INPUT);
	pullUpDnControl(GPIO_HF, PUD_UP);
	wiringPiISR(GPIO_HF, INT_EDGE_BOTH, &ISR_hf_select);

}





void usage(void)
{
	fprintf(stderr,
		"rtl_fm, a simple narrow band FM demodulator for RTL2832 based DVB-T receivers\n\n"
		"Use:\trtl_fm -f freq [-options]\n"
		"\t[-l squelch_level (default: 0/off)]\n"
		"\t[-o oversampling (default: 1, 4 recommended)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"Experimental options:\n"
		"\t[-r output_rate (default: same as -s)]\n"
		"\t[-t squelch_delay (default: 20)]\n"
		"\t[-F enables high quality FIR (default: off/square)]\n"
		"\t[-D enables de-emphasis (default: off)]\n"
		"\t[-C enables DC blocking of output (default: off)]\n"
		"\t[-A std/fast/lut choose atan math (default: std)]\n\n"
		"Produces signed 16 bit ints, use Sox or aplay to hear them.\n"
		"\trtl_fm ... - | play -t raw -r 24k -e signed-integer -b 16 -c 1 -V1 -\n"
		"\t             | aplay -r 24k -f S16_LE -t raw -c 1\n"
		"\t  -s 22.5k - | multimon -t raw /dev/stdin\n\n");
	exit(1);
}


static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");

    	free(fft_cfg);
    	free(fft_in);
    	free(fft_out);
    	free(log_pwr_fft);

	do_exit = 1;
	rtlsdr_cancel_async(dev);
}

void rotate_90(unsigned char *buf, uint32_t len)
/* 90 rotation is 1+0j, 0+1j, -1+0j, 0-1j
   or [0, 1, -3, 2, -4, -5, 7, -6] */
{
	uint32_t i;
	unsigned char tmp;
	for (i=0; i<len; i+=8) {
		/* uint8_t negation = 255 - x */
		tmp = 255 - buf[i+3];
		buf[i+3] = buf[i+2];
		buf[i+2] = tmp;

		buf[i+4] = 255 - buf[i+4];
		buf[i+5] = 255 - buf[i+5];

		tmp = 255 - buf[i+6];
		buf[i+6] = buf[i+7];
		buf[i+7] = tmp;
	}
}

void low_pass(struct fm_state *fm)
/* simple square window FIR */
{
	int i=0, i2=0;
	while (i < (int)fm->buf_len) {
		fm->now_r += ((int)fm->buf[i]   - 127);
		fm->now_j += ((int)fm->buf[i+1] - 127);
		i += 2;
		fm->prev_index++;
		if (fm->prev_index < fm->downsample) {
			continue;
		}
		fm->signal[i2]   = fm->now_r * fm->output_scale;
		fm->signal[i2+1] = fm->now_j * fm->output_scale;
		fm->prev_index = 0;
		fm->now_r = 0;
		fm->now_j = 0;
		i2 += 2;
	}
	fm->signal_len = i2;
}

void build_fir(struct fm_state *fm)
/* for now, a simple triangle 
 * fancy FIRs are equally expensive, so use one */
/* point = sum(sample[i] * fir[i] * fir_len / fir_sum) */
{
	int i, len;
	len = fm->downsample;
	for(i = 0; i < (len/2); i++) {
		fm->fir[i] = i;
	}
	for(i = len-1; i >= (len/2); i--) {
		fm->fir[i] = len - i;
	}
	fm->fir_sum = 0;
	for(i = 0; i < len; i++) {
		fm->fir_sum += fm->fir[i];
	}
}

void low_pass_fir()
/* perform an arbitrary FIR, doubles CPU use */
// possibly bugged, or overflowing
{
	int i=0, i2=0, i3=0;
	while (i < (int)fm.buf_len) {
		i3 = fm.prev_index;
		fm.now_r += ((int)fm.buf[i]   - 127) * fm.fir[i3] * fm.downsample / fm.fir_sum;
		fm.now_j += ((int)fm.buf[i+1] - 127) * fm.fir[i3] * fm.downsample / fm.fir_sum;
		i += 2;
		fm.prev_index++;
		if (fm.prev_index < fm.downsample) {
			continue;
		}
		fm.signal[i2]   = fm.now_r * fm.output_scale;
		fm.signal[i2+1] = fm.now_j * fm.output_scale;
		fm.prev_index = 0;
		fm.now_r = 0;
		fm.now_j = 0;
		i2 += 2;
	}
	fm.signal_len = i2;
}

int low_pass_simple(int16_t *signal2, int len, int step)
// no wrap around, length must be multiple of step
{
	int i, i2, sum;
	for(i=0; i < len; i+=step) {
		sum = 0;
		for(i2=0; i2<step; i2++) {
			sum += (int)signal2[i + i2];
		}
		//signal2[i/step] = (int16_t)(sum / step);
		signal2[i/step] = (int16_t)(sum);
	}
	signal2[i/step + 1] = signal2[i/step];
	return len / step;
}

void low_pass_real()
/* simple square window FIR */
// add support for upsampling?
{
	int i=0, i2=0;
	int fast = (int)fm.sample_rate / fm.post_downsample;
	int slow = fm.output_rate;
	while (i < fm.signal2_len) {
		fm.now_lpr += fm.signal2[i];
		i++;
		fm.prev_lpr_index += slow;
		if (fm.prev_lpr_index < fast) {
			continue;
		}
		fm.signal2[i2] = (int16_t)(fm.now_lpr / (fast/slow));
		fm.prev_lpr_index -= fast;
		fm.now_lpr = 0;
		i2 += 1;
	}
	fm.signal2_len = i2;
}

/* define our own complex math ops
   because ARMv5 has no hardware float */

void multiply(int ar, int aj, int br, int bj, int *cr, int *cj)
{
	*cr = ar*br - aj*bj;
	*cj = aj*br + ar*bj;
}

int polar_discriminant(int ar, int aj, int br, int bj)
{
	int cr, cj;
	double angle;
	multiply(ar, aj, br, -bj, &cr, &cj);
	angle = atan2((double)cj, (double)cr);
	return (int)(angle / 3.14159 * (1<<14));
}

int fast_atan2(int y, int x)
/* pre scaled for int16 */
{
	int yabs, angle;
	int pi4=(1<<12), pi34=3*(1<<12);  // note pi = 1<<14
	if (x==0 && y==0) {
		return 0;
	}
	yabs = y;
	if (yabs < 0) {
		yabs = -yabs;
	}
	if (x >= 0) {
		angle = pi4  - pi4 * (x-yabs) / (x+yabs);
	} else {
		angle = pi34 - pi4 * (x+yabs) / (yabs-x);
	}
	if (y < 0) {
		return -angle;
	}
	return angle;
}

int polar_disc_fast(int ar, int aj, int br, int bj)
{
	int cr, cj;
	multiply(ar, aj, br, -bj, &cr, &cj);
	return fast_atan2(cj, cr);
}

int atan_lut_init()
{
	int i = 0;

	atan_lut = malloc(atan_lut_size * sizeof(int));

	for (i = 0; i < atan_lut_size; i++) {
		atan_lut[i] = (int) (atan((double) i / (1<<atan_lut_coef)) / 3.14159 * (1<<14));
	}

	return 0;
}

int polar_disc_lut(int ar, int aj, int br, int bj)
{
	int cr, cj, x, x_abs;

	multiply(ar, aj, br, -bj, &cr, &cj);

	/* special cases */
	if (cr == 0 || cj == 0) {
		if (cr == 0 && cj == 0)
			{return 0;}
		if (cr == 0 && cj > 0)
			{return 1 << 13;}
		if (cr == 0 && cj < 0)
			{return -(1 << 13);}
		if (cj == 0 && cr > 0)
			{return 0;}
		if (cj == 0 && cr < 0)
			{return 1 << 14;}
	}

	/* real range -32768 - 32768 use 64x range -> absolute maximum: 2097152 */
	x = (cj << atan_lut_coef) / cr;
	x_abs = abs(x);

	if (x_abs >= atan_lut_size) {
		/* we can use linear range, but it is not necessary */
		return (cj > 0) ? 1<<13 : -1<<13;
	}

	if (x > 0) {
		return (cj > 0) ? atan_lut[x] : atan_lut[x] - (1<<14);
	} else {
		return (cj > 0) ? (1<<14) - atan_lut[-x] : -atan_lut[-x];
	}

	return 0;
}

void fm_demod()
{
	int i, pcm;
	pcm = polar_discriminant(fm.signal[0], fm.signal[1],
		fm.pre_r, fm.pre_j);
	fm.signal2[0] = (int16_t)pcm;
	for (i = 2; i < (fm.signal_len); i += 2) {
		switch (fm.custom_atan) {
		case 0:
			pcm = polar_discriminant(fm.signal[i], fm.signal[i+1],
				fm.signal[i-2], fm.signal[i-1]);
			break;
		case 1:
			pcm = polar_disc_fast(fm.signal[i], fm.signal[i+1],
				fm.signal[i-2], fm.signal[i-1]);
			break;
		case 2:
			pcm = polar_disc_lut(fm.signal[i], fm.signal[i+1],
				fm.signal[i-2], fm.signal[i-1]);
			break;
		}
		fm.signal2[i/2] = (int16_t)pcm;
	}
	fm.pre_r = fm.signal[fm.signal_len - 2];
	fm.pre_j = fm.signal[fm.signal_len - 1];
	fm.signal2_len = fm.signal_len/2;
}

void am_demod()
{
	int i, pcm;
	for (i = 0; i < (fm.signal_len); i += 2) {
		// hypot uses floats but won't overflow
		//fm->signal2[i/2] = (int16_t)hypot(fm->signal[i], fm->signal[i+1]);
		pcm = fm.signal[i] * fm.signal[i];
		pcm += fm.signal[i+1] * fm.signal[i+1];
		fm.signal2[i/2] = (int16_t)sqrt(pcm); // * fm->output_scale;
	}
	fm.signal2_len = fm.signal_len/2;
	// lowpass? (3khz)  highpass?  (dc)
}

void usb_demod()
{
	int i, pcm;
	for (i = 0; i < (fm.signal_len); i += 2) {
		pcm = fm.signal[i] + fm.signal[i+1];
		fm.signal2[i/2] = (int16_t)pcm; // * fm->output_scale;
	}
	fm.signal2_len = fm.signal_len/2;
}

void lsb_demod()
{
	int i, pcm;
	for (i = 0; i < (fm.signal_len); i += 2) {
		pcm = fm.signal[i] - fm.signal[i+1];
		fm.signal2[i/2] = (int16_t)pcm; // * fm->output_scale;
	}
	fm.signal2_len = fm.signal_len/2;
}

void raw_demod()
{
	/* hacky and pointless code */
	int i;
	for (i = 0; i < (fm.signal_len); i++) {
		fm.signal2[i] = (int16_t)fm.signal[i];
	}
	fm.signal2_len = fm.signal_len;
}

void deemph_filter()
{
	static int avg;  // cheating...
	int i, d;
	// de-emph IIR
	// avg = avg * (1 - alpha) + sample * alpha;
	for (i = 0; i < fm.signal2_len; i++) {
		d = fm.signal2[i] - avg;
		if (d > 0) {
			avg += (d + fm.deemph_a/2) / fm.deemph_a;
		} else {
			avg += (d - fm.deemph_a/2) / fm.deemph_a;
		}
		fm.signal2[i] = (int16_t)avg;
	}
}

void dc_block_filter()
{
	int i, avg;
	int64_t sum = 0;
	for (i=0; i < fm.signal2_len; i++) {
		sum += fm.signal2[i];
	}
	avg = sum / fm.signal2_len;
	avg = (avg + fm.dc_avg * 9) / 10;
	for (i=0; i < fm.signal2_len; i++) {
		fm.signal2[i] -= avg;
	}
	fm.dc_avg = avg;
}

int mad(int *samples, int len, int step)
/* mean average deviation */
{
	int i=0, sum=0, ave=0;
	if (len == 0)
		{return 0;}
	for (i=0; i<len; i+=step) {
		sum += samples[i];
	}
	ave = sum / (len * step);
	sum = 0;
	for (i=0; i<len; i+=step) {
		sum += abs(samples[i] - ave);
	}
	return sum / (len / step);
}

int post_squelch()
/* returns 1 for active signal, 0 for no signal */
{
	int dev_r, dev_j, len, sq_l;
	/* only for small samples, big samples need chunk processing */
	len = fm.signal_len;
	sq_l = fm.squelch_level;
	dev_r = mad(&(fm.signal[0]), len, 2);
	dev_j = mad(&(fm.signal[1]), len, 2);
	if ((dev_r > sq_l) || (dev_j > sq_l)) {
		fm.squelch_hits = 0;
		return 1;
	}
	fm.squelch_hits++;
	return 0;
}


void full_demod(struct fm_state *fm)
{
	int i, sr = 0;
	rotate_90(fm->buf, fm->buf_len);
	if (fm->fir_enable) {
		low_pass_fir(fm, fm->buf, fm->buf_len);
	} else {
		low_pass(fm);
	}
	pthread_mutex_unlock(&data_write);
	fm->mode_demod(fm);
        if (fm->mode_demod == &raw_demod) {
		fwrite(fm->signal2, 2, fm->signal2_len, fm->file);
		return;
	}
	sr = post_squelch();
	if (!sr && fm->squelch_hits > fm->conseq_squelch) {
		if (fm->terminate_on_squelch) {
			fm->exit_flag = 1;}
			for (i=0; i < fm->signal_len; i++) {
				fm->signal2[i] = 0;}
	}
	if (fm->post_downsample > 1) {
		fm->signal2_len = low_pass_simple(fm->signal2, fm->signal2_len, fm->post_downsample);}
	if (fm->output_rate > 0) {
		low_pass_real();
	}
	if (fm->deemph) {
		deemph_filter();}
	if (fm->dc_block) {
		dc_block_filter();}
	/* ignore under runs for now */
	fwrite(fm->signal2, 2, fm->signal2_len, fm->file);
}

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct fm_state *fm2 = ctx;
	if (do_exit) {
		return;}
	if (!ctx) {
		return;}
	pthread_mutex_lock(&data_write);
	memcpy(fm2->buf, buf, len);
	fm2->buf_len = len;
	pthread_mutex_unlock(&data_ready);
	/* single threaded uses 25% less CPU? */
	/* full_demod(fm2); */
}

static void *demod_thread_fn(void *arg)
{
	struct fm_state *fm2 = arg;
	while (!do_exit) {
		pthread_mutex_lock(&data_ready);
		full_demod(fm2);
		if (fm2->exit_flag) {
			do_exit = 1;
			rtlsdr_cancel_async(dev);
		}
	}
	return 0;
}

double atofs(char* f)
/* standard suffixes */
{
	char* chop;
        double suff = 1.0;
	chop = malloc((strlen(f)+1)*sizeof(char));
	strncpy(chop, f, strlen(f)-1);
	switch (f[strlen(f)-1]) {
		case 'G':
			suff *= 1e3;
		case 'M':
			suff *= 1e3;
		case 'k':
			suff *= 1e3;
                        suff *= atof(chop);}
	free(chop);
	if (suff != 1.0) {
		return suff;}
	return atof(f);
}


void fm_init(struct fm_state *fm)
{
	fm->squelch_level = 0;
	fm->conseq_squelch = 20;
	fm->terminate_on_squelch = 0;
	fm->squelch_hits = 0;
	fm->fir_enable = 0;
	fm->prev_index = 0;
	fm->post_downsample = 1;  // once this works, default = 4
	fm->custom_atan = 0;
	fm->deemph = 0;
	fm->output_rate = -1;  // flag for disabled
	fm->pre_j = fm->pre_r = fm->now_r = fm->now_j = 0;
	fm->prev_lpr_index = 0;
	fm->deemph_a = 0;
	fm->now_lpr = 0;
	fm->dc_block = 0;
	fm->dc_avg = 0;
	fm->step = step_array[STEP_ARRAY_INITIAL];
	fm->gain = gain_array[GAIN_ARRAY_INITIAL];
	fm->file = stdout;

	if (digitalRead(GPIO_HF)) {
		fm->freq = FREQ_INITIAL_HF;
		fm->sample_rate = SAMPLE_RATE_DEFAULT;
		fm->freq_offset = UPCONVERTER_CRYSTAL;
		fm->mode_demod = &am_demod;
		fm->mode_name = "AM";
		hf_mode = TRUE;
	} else {
		fm->freq = FREQ_INITIAL_NORM;
		fm->sample_rate = SAMPLE_RATE_WFM;
		fm->freq_offset = 0;
		fm->mode_demod = &fm_demod;
		fm->mode_name = "WFM";
		hf_mode = FALSE;
	}
}

static void draw_text(cairo_t *cr)
{
    	
	cairo_text_extents_t cte;
    	double txt1_y, txt2_y;
	uint32_t sample_rate = fm.sample_rate;
	uint32_t freq_display = fm.freq - fm.freq_offset + OFFSET_CORRECTION;
	uint32_t freq_lower = ( (freq_display > (capture_rate / 2)) ? (freq_display - (capture_rate / 2)) : 0);
	uint32_t freq_upper = freq_display + (capture_rate / 2);	

	gchar *mode_str = g_strdup_printf(fm.mode_name);
	gchar *freq_str = g_strdup_printf("%.3f kHz", 1.e-3f*freq_display);
	gchar *freq_lower_str = g_strdup_printf("%.1f MHz", 1.e-6f*freq_lower);
	gchar *freq_upper_str = g_strdup_printf("%.1f MHz", 1.e-6f*freq_upper);
    	gchar *step_str = g_strdup_printf("Step: %.2f kHz", 1.e-3f*(float)(fm.step));
	gchar *gain_str = g_strdup_printf("Gain: %.1f dB", (float)gain_array[gain_array_ptr]/10);
	
	if (gain_array_ptr == GAIN_AUTO_PTR) {
		gain_str = g_strdup_printf("Gain: AUTO");
	}

    	/* clear area */
    	cairo_set_source_rgb(cr, 0.02, 0.02, 0.09);
    	cairo_set_line_width(cr, 1.0);
    	cairo_rectangle(cr, 0, 0, width, yzero);
    	cairo_stroke_preserve(cr);
    	cairo_fill(cr);

    	cairo_select_font_face(cr, "Sans",
                           CAIRO_FONT_SLANT_NORMAL,
                           CAIRO_FONT_WEIGHT_BOLD);

	cairo_set_font_size(cr, 20);

	// Frequency
    	cairo_text_extents(cr, freq_str, &cte);
    	txt1_y = text_margin + cte.height;
    	cairo_set_source_rgba(cr, 0, 0.9, 0, 0.8);
    	cairo_move_to(cr, text_margin, txt1_y);
    	cairo_show_text(cr, freq_str);

	// Mode
	cairo_text_extents(cr, mode_str, &cte);
	cairo_move_to(cr, width - 65, txt1_y);
	cairo_show_text(cr, mode_str);

	// Step
    	cairo_set_font_size(cr, 10);
    	cairo_text_extents(cr, step_str, &cte);
    	txt2_y = txt1_y + cte.height + text_margin;
    	cairo_set_source_rgba(cr, 0, 0.9, 0, 0.8);
    	cairo_move_to(cr, text_margin, txt2_y);
    	cairo_show_text(cr, step_str);

	// Gain
        cairo_set_font_size(cr, 10);
        cairo_text_extents(cr, gain_str, &cte);
        txt2_y = txt1_y + cte.height + text_margin;
        cairo_set_source_rgba(cr, 0, 0.9, 0, 0.8);
        cairo_move_to(cr, width - text_margin - cte.width, txt2_y);
        cairo_show_text(cr, gain_str);

	
	// Display upper and lower frequencies
	cairo_set_font_size(cr, 8);
	cairo_set_source_rgba(cr, 0, 0.3, 0.7, 0.8);

	// Freq lower
    	cairo_text_extents(cr, freq_lower_str, &cte);
    	cairo_move_to(cr, text_margin, yzero - cte.height);
    	cairo_show_text(cr, freq_lower_str);

	// Freq upper
    	cairo_text_extents(cr, freq_upper_str, &cte);
    	cairo_move_to(cr, width - text_margin - cte.width, yzero - cte.height);
    	cairo_show_text(cr, freq_upper_str);
	

    	g_free(freq_str);
    	g_free(step_str);
	g_free(mode_str);
	g_free(freq_lower_str);
	g_free(freq_upper_str);
}

static int db_to_pixel(float dbfs)
{
    return yzero+(int)(-dbfs*scale);
}

static void run_fft()
{   
    	int i;
    	kiss_fft_cpx pt;
    	float pwr;

	pthread_mutex_lock(&data_write);

	for (i = 0; i < (fft_size-1); i+=2)
    	{
        	fft_in[i].r = ((float)fm.buf[2*i])/255.f;
		fft_in[i+1].r = fft_in[i].r;
        	fft_in[i].i = ((float)fm.buf[2*i+1])/255.f;
		fft_in[i+1].i = fft_in[i].i;
    	}

	pthread_mutex_unlock(&data_write);

    	kiss_fft(fft_cfg, fft_in, fft_out);

    	for (i = 0; i < (fft_size-1); i+=2)
    	{
        	/* shift, normalize and convert to dBFS */
        	if (i < fft_size / 2)
        	{
            		pt.r = fft_out[fft_size/2+i].r / fft_size;
            		pt.i = fft_out[fft_size/2+i].i / fft_size;
        	}
        	else
        	{
            		pt.r = fft_out[i-fft_size/2].r / fft_size;
            		pt.i = fft_out[i-fft_size/2].i / fft_size;
        	}
        
		pwr = pt.r * pt.r + pt.i * pt.i;
        
        	log_pwr_fft[i] = 10.f * log10(pwr + 1.0e-20f);
    	}
}


static void draw_gui(cairo_t *cr)
{
   	cairo_set_source_rgb(cr, 0.02, 0.02, 0.09);
	cairo_set_line_width(cr, 1.0);

 	cairo_rectangle(cr, 0, yzero, width, height);
 	cairo_stroke_preserve(cr);
	cairo_fill(cr);

	if (!menu_req) {
		cairo_set_source_rgba(cr, 1.0, 0.2, 0.6, 1.0);

		int x, y;

		for (x = 0; x < (width-1); x+=2 )
    		{
        		y = db_to_pixel(log_pwr_fft[x]);
        		cairo_move_to(cr, x, height);
        		cairo_line_to(cr, x, y);
    		}

		cairo_stroke(cr);

		// Add center line
		cairo_set_source_rgba(cr, 0, 0.3, 0.7, 0.8);
		cairo_set_line_width(cr, 2.0);
		cairo_move_to(cr, width/2, height);
		cairo_line_to(cr, width/2, yzero);  
		cairo_stroke(cr);
	} else {
		// Draw menu
		cairo_set_source_rgba(cr, 0, 0.9, 0, 0.8); 
		cairo_rectangle(cr, (menu_item*100)+4, height -4, 60, -27);
		cairo_stroke(cr);

		cairo_text_extents_t cte;
    		double txt1_y;

		gchar *mode_str = g_strdup_printf("MODE");
		gchar *step_str = g_strdup_printf("STEP");
		gchar *gain_str = g_strdup_printf("GAIN");

    		cairo_select_font_face(cr, "Sans",
                           CAIRO_FONT_SLANT_NORMAL,
                           CAIRO_FONT_WEIGHT_NORMAL);

    		cairo_set_font_size(cr, 16);
    		cairo_text_extents(cr, mode_str, &cte);
    		txt1_y = height - cte.height;
    		cairo_set_source_rgba(cr, 0, 0.9, 0, 0.8);
    		cairo_move_to(cr, text_margin, txt1_y);
    		cairo_show_text(cr, mode_str);

		cairo_move_to(cr, text_margin + 100, txt1_y);
		cairo_show_text(cr, step_str);

                cairo_move_to(cr, text_margin + 200, txt1_y);
                cairo_show_text(cr, gain_str);

	}
}



void *gtk_window_thread() {

	gdk_threads_enter();
	gtk_main();
	gdk_threads_leave();

	return NULL;

}

gint timeout_fft_text_cb(gpointer darea)
{
	cairo_t *cr;
	cr = gdk_cairo_create(gtk_widget_get_window(GTK_WIDGET(darea)));

	if (update_text)
    	{
        	draw_text(cr);
        	update_text = FALSE;
    	}
	
	cairo_destroy(cr);

	return TRUE;


}

gint timeout_fft_chart_cb(gpointer darea)
{
	// Calculate FFT
    	run_fft();
	
	// Update chart
	cairo_t *cr;
	cr = gdk_cairo_create(gtk_widget_get_window(GTK_WIDGET(darea)));
	draw_gui(cr);
    
	cairo_destroy(cr);

	return TRUE;
}

int main(int argc, char **argv)
{
	struct sigaction sigact;
	int r, opt, wb_mode = 0;
	int i; 
	uint8_t *buffer;
	int device_count;
	int ppm_error = 0;
	char vendor[256], product[256], serial[256];

	// Setup encoder
	wiringPiSetup();
	setup_gpios();

	// GUI
	guint tid_fft_chart, tid_fft_text;
	pthread_t gtk_window_tid, set_center_freq_tid;
	GtkWidget *window;
	gdk_threads_init();
	gtk_init (&argc, &argv);
    	window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    	gtk_window_set_default_size(GTK_WINDOW(window), width, height);
    	scale = (float)height/DYNAMIC_RANGE * SCREEN_FRAC;
    	yzero = (int)(height*(1.0f-SCREEN_FRAC));
    	text_margin = yzero/10;

    	gtk_widget_show(window);
    	gdk_window_set_cursor(gtk_widget_get_window(window), gdk_cursor_new(GDK_BLANK_CURSOR));

    	/* set up FFT */
	fft_size = width;
    	fft_cfg = kiss_fft_alloc(fft_size, FALSE, NULL, NULL);
    	fft_in = malloc(width * sizeof(kiss_fft_cpx));
    	fft_out = malloc(width * sizeof(kiss_fft_cpx));
    	log_pwr_fft = malloc(width * sizeof(float));

	tid_fft_chart = g_timeout_add(FFT_REFRESH_MILLI, timeout_fft_chart_cb, window);
	tid_fft_text = g_timeout_add(100, timeout_fft_text_cb, window);
	
	pthread_create(&set_center_freq_tid, NULL, set_center_freq_thread, NULL);
	pthread_create(&gtk_window_tid, NULL, gtk_window_thread, NULL);



	fm_init(&fm);
	pthread_mutex_init(&data_ready, NULL);
	pthread_mutex_init(&data_write, NULL);

	while ((opt = getopt(argc, argv, "b:l:o:t:r:p:FA:MULRDC")) != -1) {
		switch (opt) {
		case 'l':
			fm.squelch_level = (int)atof(optarg);
			break;
		case 'r':
			fm.output_rate = (int)atofs(optarg);
			break;
		case 'o':
			fm.post_downsample = (int)atof(optarg);
			if (fm.post_downsample < 1 || fm.post_downsample > MAXIMUM_OVERSAMPLE) {
				fprintf(stderr, "Oversample must be between 1 and %i\n", MAXIMUM_OVERSAMPLE);}
			break;
		case 't':
			fm.conseq_squelch = (int)atof(optarg);
			if (fm.conseq_squelch < 0) {
				fm.conseq_squelch = -fm.conseq_squelch;
				fm.terminate_on_squelch = 1;
			}
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'F':
			fm.fir_enable = 1;
			break;
		case 'A':
			if (strcmp("std",  optarg) == 0) {
				fm.custom_atan = 0;}
			if (strcmp("fast", optarg) == 0) {
				fm.custom_atan = 1;}
			if (strcmp("lut",  optarg) == 0) {
				atan_lut_init();
				fm.custom_atan = 2;}
			break;
		case 'D':
			fm.deemph = 1;
			break;
		case 'C':
			fm.dc_block = 1;
			break;
		default:
			usage();
			break;
		}
	}
	
	buffer = malloc(lcm_post[fm.post_downsample] * DEFAULT_BUF_LENGTH * sizeof(uint8_t));

	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		exit(1);
	}

	fprintf(stderr, "Found %d device(s):\n", device_count);
	rtlsdr_get_device_usb_strings(0, vendor, product, serial);
	fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
	fprintf(stderr, "\n");

	fprintf(stderr, "Using device: %s\n", rtlsdr_get_device_name(0));

	r = rtlsdr_open(&dev, 0);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device\n");
		exit(1);
	}
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);


	if (fm.deemph) {
		fm.deemph_a = (int)round(1.0/((1.0-exp(-1.0/(fm.output_rate * 75e-6)))));
	}

	build_fir(&fm);

	

	// Update the frequency on the screen for the first time
	update_sample_rate();
	set_center_freq = TRUE;
	update_text = TRUE;

	// Set initial gain
        if (digitalRead(GPIO_HF)) {
                rtlsdr_set_tuner_gain_mode(dev, GAIN_MANUAL);
		gain_array_ptr = GAIN_AM_PTR;
                rtlsdr_set_tuner_gain(dev, gain_array[gain_array_ptr]);
        } else {
		gain_array_ptr = GAIN_AUTO_PTR;
                rtlsdr_set_tuner_gain_mode(dev, GAIN_AUTO);
        }
	
	r = rtlsdr_set_freq_correction(dev, ppm_error);

	/* Reset endpoint before we start reading from it (mandatory) */
	r = rtlsdr_reset_buffer(dev);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to reset buffers\n");}

	pthread_create(&demod_thread, NULL, demod_thread_fn, (void *)(&fm));

	// Main function of the decoder
	rtlsdr_read_async(dev, rtlsdr_callback, (void *)(&fm),
			      DEFAULT_ASYNC_BUF_NUMBER,
			      lcm_post[fm.post_downsample] * DEFAULT_BUF_LENGTH);

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");}
	else {
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);}
	rtlsdr_cancel_async(dev);
	pthread_mutex_destroy(&data_ready);
	pthread_mutex_destroy(&data_write);

	if (fm.file != stdout) {
		fclose(fm.file);}

	rtlsdr_close(dev);
	free (buffer);
	
	g_source_remove(tid_fft_chart);
	g_source_remove(tid_fft_text);
	return r >= 0 ? r : -r;
}








