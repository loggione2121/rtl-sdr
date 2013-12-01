
#define OFFSET_CORRECTION		-300000
#define SAMPLE_RATE_WFM			200000
#define SAMPLE_RATE_DEFAULT		48000

#define STEP_ARRAY_INITIAL		9
#define STEP_ARRAY_SIZE			14
#define GAIN_ARRAY_INITIAL		0
#define GAIN_ARRAY_SIZE			14

#define GAIN_AUTO			0
#define GAIN_MANUAL			1
#define GAIN_AM_PTR			13
#define GAIN_AUTO_PTR			0

#define DEFAULT_ASYNC_BUF_NUMBER	32
#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define MAXIMUM_OVERSAMPLE		16
#define MAXIMUM_BUF_LENGTH		(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)

// GUI
#define DYNAMIC_RANGE 			90.f  // -dBFS coreresponding to bottom of screen
#define SCREEN_FRAC 			0.7f  // Fraction of screen height used for FFT
#define MENU_ITEM_MAX 			2
#define MENU_MODE 			0
#define MENU_STEP 			1
#define MENU_GAIN			2

#define FREQ_DIVIDER 			4

// GPIOs
#define GPIO_HF				2
#define GPIO_ROTARYA			4
#define GPIO_ROTARYB			5
#define GPIO_SELECT			8

#define UPCONVERTER_CRYSTAL 		125000000
#define FREQ_MIN 			52000000 
#define FREQ_MAX 			220000000 
#define FREQ_INITIAL_NORM 		98000000
#define FREQ_INITIAL_HF 		133000000

#define DEBOUNCE_TIME_MILLI 		200

#define SCREEN_WIDTH			320
#define SCREEN_HEIGHT			240

#define FFT_REFRESH_MILLI		600

void fm_demod();
void am_demod();
void usb_demod();
void lsb_demod();
