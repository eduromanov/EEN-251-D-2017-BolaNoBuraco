/*
Eduardo Ra 14.01160-3
Matheus Ra 14.03521-9
Luca Avancini
Carolina Stivalli
*/

#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "smc.h"
#include "math.h"
#include "aat31xx.h"

//#####----DEFINES---####----DEFINES---####----DEFINES---####----DEFINES---####----DEFINES---####----DEFINES---####----DEFINES---####----DEFINES---####----DEFINES---####----DEFINES---##

#define CONF_UART              UART0
#define CONF_UART_BAUDRATE     115200
#define CONF_UART_CHAR_LENGTH  US_MR_CHRL_8_BIT
#define CONF_UART_PARITY       US_MR_PAR_NO
#define CONF_UART_STOP_BITS    US_MR_NBSTOP_1_BIT
#define PIN_LED_BLUE	19
#define PIN_LED_RED		20
#define PIN_LED_GREEN	20
#define PIN_BUTTON		3
#define PIN_BUTTON1		12
#define time			100
#define PORT_LED_BLUE	PIOA
#define PORT_LED_GREEN	PIOA
#define PORT_LED_RED	PIOC
#define PORT_BUT_2		PIOB
#define PORT_BUT_1		PIOC
#define ID_LED_BLUE		ID_PIOA
#define ID_LED_GREEN	ID_PIOA
#define ID_LED_RED		ID_PIOC
#define ID_BUT_2		ID_PIOB
#define ID_BUT_1		ID_PIOC
#define MASK_LED_BLUE	(1u << PIN_LED_BLUE)
#define MASK_LED_GREEN	(1u << PIN_LED_GREEN)
#define MASK_LED_RED	(1u << PIN_LED_RED)
#define MASK_BUT_2		(1u << PIN_BUTTON)
#define MASK_BUT_1		(1u << PIN_BUTTON1)
#define PIN_PUSHBUTTON_1_MASK	PIO_PB3
#define PIN_PUSHBUTTON_1_PIO	PIOB
#define PIN_PUSHBUTTON_1_ID		ID_PIOB
#define PIN_PUSHBUTTON_1_TYPE	PIO_INPUT
#define PIN_PUSHBUTTON_1_ATTR	PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE
#define PIN_PUSHBUTTON_2_MASK	PIO_PC12
#define PIN_PUSHBUTTON_2_PIO	PIOC
#define PIN_PUSHBUTTON_2_ID		ID_PIOC
#define PIN_PUSHBUTTON_2_TYPE	PIO_INPUT
#define PIN_PUSHBUTTON_2_ATTR	PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_FALL_EDGE
#define TC			TC0
#define CHANNEL		0
#define ID_TC		ID_TC0
#define TC_IRQn     TC0_IRQn
#define IRQ_PRIOR_PIO    1
#define ADC_CLOCK   6400000
#define ILI93XX_LCD_CS      1

//##---VARIAVEIS GLOBAIS---##---VARIAVEIS GLOBAIS---##---VARIAVEIS GLOBAIS---##---VARIAVEIS GLOBAIS---##---VARIAVEIS GLOBAIS---##---VARIAVEIS GLOBAIS---##---VARIAVEIS GLOBAIS---##

int counter=0;
unsigned char counterprint[14];
int Tcounter=0;
unsigned char Tcounterprint[14];
int ufreq_desired = 200;
uint32_t ul_div = 0;
uint32_t ul_tcclks = 0;
uint32_t ul_tcwrite = 0;
uint32_t ul_sysclk = 120000000;
float result =0.0;
float sensor =0.0;
unsigned char resultprint[14];
float teta=0.0;
volatile uint16_t bmw= 0;
volatile uint16_t cnt = 0;
float erro_now = 0;
float erro_last = 0;
float atuador_now = 0;
float atuador_last = 0;
int REF = 1550;
struct ili93xx_opt_t g_ili93xx_display_opt;

//##---CONFIG UART---####---CONFIG UART---####---CONFIG UART---####---CONFIG UART---####---CONFIG UART---####---CONFIG UART---####---CONFIG UART---####---CONFIG UART---####---CONFIG UART---##
static void configure_console(void)
{
	static usart_serial_options_t usart_options = {
		.baudrate = CONF_UART_BAUDRATE,                     //Atribuindo o Baudrate
		.charlength = CONF_UART_CHAR_LENGTH,                //Atribuindo o tamanho da mensagem
		.paritytype = CONF_UART_PARITY,                     //Atribuindo a paridade
		.stopbits = CONF_UART_STOP_BITS                     //Atribuindo o StopBit
	};
	
	usart_serial_init(CONF_UART, &usart_options);              //Inicializa a USART
	stdio_serial_init((Usart *)CONF_UART, &usart_options);     //Inicializa o stdio no modo Serial
}

//##---TC HANDLER---##//##---TC HANDLER---##//##---TC HANDLER---##//##---TC HANDLER---##//##---TC HANDLER---##//##---TC HANDLER---##//##---TC HANDLER---##//##---TC HANDLER---##//##---TC HANDLER---##

void TC0_Handler(void)											//configura a função da interrupção do timer
{
	
	tc_get_status(TC0,0);
	adc_start(ADC);
		
}

//##---AACD HANDLER---####---AACD HANDLER---####---AACD HANDLER---####---AACD HANDLER---####---AACD HANDLER---####---AACD HANDLER---####---AACD HANDLER---####---AACD HANDLER---##

void ADC_Handler(void)
{
	

	if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY)
	{
		// Get latest digital data value from ADC and can be used by application
		result = adc_get_latest_value(ADC);
	}
	erro_last=erro_now;
	erro_now=(REF-result);
	atuador_last=atuador_now;
	atuador_now=(0.013612*erro_now)-(0.01347539*erro_last)+(0.97619048*atuador_last);
	
bmw=(atuador_now*1.7)*100/14;
	PWM->PWM_CH_NUM[0].PWM_CDTY = bmw*4096 + 3000;
	sensor=result;
	result=bmw;
	ili93xx_set_foreground_color(COLOR_DARKBLUE);
	ili93xx_draw_filled_rectangle(0,45,340,85);
	sprintf(resultprint,"%.0f %%" ,result);
	ili93xx_set_foreground_color(COLOR_GOLD);
	ili93xx_draw_string(20, 50,  (uint8_t *)resultprint);
	sprintf(resultprint,"%.0f %%" ,sensor);
	ili93xx_draw_string(180, 50,  (uint8_t *)resultprint);
	
}

//##---CONFIG TC---####---CONFIG TC---####---CONFIG TC---####---CONFIG TC---####---CONFIG TC---####---CONFIG TC---####---CONFIG TC---####---CONFIG TC---####---CONFIG TC---##

static void configure_tc(void)														//configura timer counter
{

	ul_sysclk = sysclk_get_cpu_hz();
	pmc_enable_periph_clk(ID_TC0);
	tc_find_mck_divisor(1, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	ul_tcwrite=((ul_sysclk/ul_div)/ufreq_desired);
	tc_init(TC0,0,TC_CMR_CPCTRG| ul_tcclks);
	tc_write_rc(TC0,0,ul_tcwrite);
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
	NVIC_EnableIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn,30);
	tc_start(TC0,0);
	
}

//##---CONFIG AACD---####---CONFIG AACD---####---CONFIG AACD---####---CONFIG AACD---####---CONFIG AACD---####---CONFIG AACD---####---CONFIG AACD---####---CONFIG AACD---####---CONFIG AACD---##

void aacd_setup(void)
{
	pmc_enable_periph_clk(ID_ADC);

	adc_init(ADC, sysclk_get_main_hz(), ADC_CLOCK, 8);
	
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
	
	adc_set_resolution(ADC, ADC_MR_LOWRES_BITS_12);
	
	adc_enable_channel(ADC, ADC_CHANNEL_0);
	
	adc_enable_interrupt(ADC, ADC_IER_DRDY);

	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
	
	NVIC_EnableIRQ(ADC_IRQn);

	NVIC_SetPriority(ADC_IRQn,30);
}

//##---CONFIG LSD---####---CONFIG LSD---####---CONFIG LSD---####---CONFIG LSD---####---CONFIG LSD---####---CONFIG LSD---####---CONFIG LSD---####---CONFIG LSD---##

void lsd_setup(void)
{
	/** Enable peripheral clock */
	pmc_enable_periph_clk(ID_SMC);

	/** Configure SMC interface for Lcd */
	smc_set_setup_timing(SMC, ILI93XX_LCD_CS, SMC_SETUP_NWE_SETUP(2)
	| SMC_SETUP_NCS_WR_SETUP(2)
	| SMC_SETUP_NRD_SETUP(2)
	| SMC_SETUP_NCS_RD_SETUP(2));
	smc_set_pulse_timing(SMC, ILI93XX_LCD_CS, SMC_PULSE_NWE_PULSE(4)
	| SMC_PULSE_NCS_WR_PULSE(4)
	| SMC_PULSE_NRD_PULSE(10)
	| SMC_PULSE_NCS_RD_PULSE(10));
	smc_set_cycle_timing(SMC, ILI93XX_LCD_CS, SMC_CYCLE_NWE_CYCLE(10)
	| SMC_CYCLE_NRD_CYCLE(22));
	#if ((!defined(SAM4S)) && (!defined(SAM4E)))
	smc_set_mode(SMC, ILI93XX_LCD_CS, SMC_MODE_READ_MODE
	| SMC_MODE_WRITE_MODE
	| SMC_MODE_DBW_8_BIT);
	#else
	smc_set_mode(SMC, ILI93XX_LCD_CS, SMC_MODE_READ_MODE
	| SMC_MODE_WRITE_MODE);
	#endif
	/** Initialize display parameter */
	g_ili93xx_display_opt.ul_width = ILI93XX_LCD_WIDTH;
	g_ili93xx_display_opt.ul_height = ILI93XX_LCD_HEIGHT;
	g_ili93xx_display_opt.foreground_color = COLOR_BLACK;
	g_ili93xx_display_opt.background_color = COLOR_DARKBLUE;

	/** Switch off backlight */
	aat31xx_disable_backlight();

	/** Initialize LCD */
	ili93xx_init(&g_ili93xx_display_opt);

	/** Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	ili93xx_set_foreground_color(COLOR_DARKBLUE);
	ili93xx_draw_filled_rectangle(0, 0, ILI93XX_LCD_WIDTH,
	ILI93XX_LCD_HEIGHT);
	/** Turn on LCD */
	ili93xx_display_on();
	ili93xx_set_cursor_position(0, 0);
		
	/*Desenha bmw + cm*/
	ili93xx_set_foreground_color(COLOR_GOLD);
	ili93xx_draw_string(20, 20, (uint8_t *)"BMW");
	ili93xx_draw_string(180, 20, (uint8_t *)"CM");

	// desenha logo da maua
	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(10, 195, 90, 275);
	ili93xx_set_foreground_color(COLOR_DARKBLUE);
	ili93xx_draw_filled_circle(50, 235, 40);
	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(20, 205, 80, 265);
	ili93xx_set_foreground_color(COLOR_DARKBLUE);
	ili93xx_draw_filled_circle(50, 235, 30);
	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(105, 195, 230, 215);
	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(105, 255, 230, 275);
	ili93xx_draw_string(147, 230, (uint8_t *)"MAUA");
}


int main(void)
{
	/*Initialize system*/
	
	sysclk_init();
	board_init();
	
	 
	 // disable the PIO (peripheral controls the pin)
	 PIOA->PIO_PDR = PIO_PDR_P19;
	 // select alternate function B (PWML0) for pin PA19
	 PIOA->PIO_ABCDSR[0] |= PIO_ABCDSR_P19;
	 PIOA->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P19;
	 // Enable the PWM peripheral from the Power Manger
	 PMC->PMC_PCER0 = (1 << ID_PWM);
	 // Select the Clock to run at the MCK (4MHz)
	 PWM->PWM_CH_NUM[0].PWM_CMR = PWM_CMR_CPRE_MCK;
	 // select the period 10msec
	 PWM->PWM_CH_NUM[0].PWM_CPRD = 4096;// freq em khz
	 // select the duty cycle
	 PWM->PWM_CH_NUM[0].PWM_CDTY = 3500;
	 // enable the channel
	 PWM->PWM_ENA = PWM_ENA_CHID0;
	
	/** Initialize debug console */
	configure_console();
	/** Initialize lsd */
	lsd_setup();		
	/** Initialize AACD */
	aacd_setup();
	/** Initialize TC */
	configure_tc();
	while (1) {
	
	}
}

