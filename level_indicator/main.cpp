/*
 * Level_indicator.c
 *
 * Created: 04/04/2020 15:48:29
 * Author : dbrown
 */ 

 // for the ATTINY816

 //define F_CPU in toolchain -> C++ -> symbols -> defined -> F_CPU=8000000UL = 8MHz & -std=c++11 in C++ misc
 //F_CPU tells the compiler that our clock is an 16Mhz one so it can generate an accurate delay, must be declared above delay so delay knows what is the value of F_CPU


#include <avr/io.h>
#include <util/delay.h>        //Contains some delay functions that will generate accurate delays of ms and us
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include "ws2811_8.h"



//function prototypes
void cpu_clock_init();
void init_ADC();
void init_pins();
void read_set_level();
uint16_t get_adc(uint8_t sel_pin); //sel_pin = ADC_V or ADC_I
void convert_hsl_rgb(float hue, float saturation, float luminosity);

//define structures for rgb values and time
struct rgb{
	uint8_t g, r, b;
};


//Pin numbers;
//LED
#define led_port PORTA
#define led_mask PIN1_bm			
#define led_ctrl PIN1CTRL
//numbers of LEDs
#define ws_pin 1 //setup led port
#define no_leds 6 //number of LEDs
//mask for pressure sensor
#define pressure_sensor_port PORTA
#define pressure_sensor_mask PIN4_bm
#define pressure_sensor_ctrl PIN4CTRL
#define pressure_sensor_mux_pos 4
//define low level calibration pot
#define low_level_pot_port PORTB
#define low_level_pot_mask PIN1_bm
#define low_level_pot_ctrl PIN1CTRL
#define low_level_pot_mux_pos 10
//define high level calibration pot
#define high_level_pot_port PORTB
#define high_level_pot_mask PIN0_bm
#define high_level_pot_ctrl PIN0CTRL
#define high_level_pot_mux_pos 11

//Global variables
volatile uint16_t low_level_value = 0;
volatile uint16_t high_level_value = 0;
volatile uint16_t pressure_value = 0;
volatile uint8_t flag_update = 0;
uint8_t max_brightness = 20; //output brightness scalar


//instantiate class/structure instances
rgb leds[no_leds]; 	//generate array for number of leds in system
float temp_r = 0;
float temp_g = 0;
float temp_b = 0;


int main(void)
{
	//init functions
	cpu_clock_init();
	init_ADC();

	//setup pins
	init_pins();

    while (1) 
    {
		
		//read low level pot
		uint16_t temp_read = ((low_level_value * 9) + get_adc(low_level_pot_mux_pos)) / 10;
		
		//update if changed
		if (temp_read != low_level_value)
		{
			low_level_value = temp_read;
			flag_update = 1;
		}
		

		//read high level pot
		temp_read = ((high_level_value * 9) + get_adc(high_level_pot_mux_pos)) / 10;
		
		//update if changed
		if (temp_read != high_level_value)
		{
			high_level_value = temp_read;
			flag_update = 1;
		}


		//read pressure sensor
		temp_read = ((pressure_value * 9) + get_adc(pressure_sensor_mux_pos)) / 10;
		
		//update if changed
		if (temp_read != pressure_value)
		{
			pressure_value = temp_read;
			flag_update = 1;
		}
		
		//update if change
		if (flag_update != 0)
		{
			read_set_level();
			flag_update = 0;
		}
    }
}


//Functions - Core
//============================================================================================
//setup the CPU
void cpu_clock_init()
{
	//set the clock multiplier to 2, with the clock fuse set to 16MHz to give 8MHz
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);
}


//Setup ADC
void init_ADC()
{
	//TINY - setup ADC
	//VREF - setup voltage reference to 4.3V
	VREF.CTRLA = VREF_ADC0REFSEL_4V34_gc;
	//VREF - enable ADC ref
	VREF.CTRLB |= VREF_ADC0REFEN_bm;
	//ADC - select resolution 10bit
	ADC0.CTRLA = ADC_RESSEL_10BIT_gc;
	//ADC - number of convertions accumulated per measurement
	ADC0.CTRLB = ADC_SAMPNUM_ACC8_gc;
	//ADC - select reference
	ADC0.CTRLC = ADC_REFSEL_INTREF_gc;
	//ADC - sampling rate pre-scaler ~1.25MHz
	ADC0.CTRLC = ADC_PRESC_DIV16_gc;
	//ADC - initial input PA3 / AIN3
	ADC0.MUXPOS = ADC_MUXPOS_AIN3_gc;
	//ADC - enable start event (start measuring on enable)
	ADC0.EVCTRL |= ADC_STARTEI_bm;
	//ADC - enable ADC (ready for measurement trigger)
	ADC0.CTRLA |= ADC_ENABLE_bm;
};



//setup ports
void init_pins()
{
	//sensor
	pressure_sensor_port.DIRCLR = pressure_sensor_mask;

	//adjust pot
	low_level_pot_port.DIRCLR = low_level_pot_mask;
	high_level_pot_port.DIRCLR = high_level_pot_mask;
	
	//set LED pin
	led_port.DIRSET = led_mask;

}



//send data to LEDs
void send()
{
	WS_send(&(leds[0].g), no_leds, ws_pin);
}


//read and update led level indicator
void read_set_level()
{
	//set colour and number of LEDs to light up
	volatile uint8_t number_to_light = 0;
	volatile float hue = 0;

	//read level pots 10bit = 1024
	//the low level pot sets the value when fill is zero and the high level similar
	//scale pressure value by trim pots to give a 0 to 1 value
	float scaled_level = (float(pressure_value) - float(low_level_value)) / (float(high_level_value) - float(low_level_value));

	//check for valid data
	if (isnan(scaled_level) == 1)
	{
		scaled_level = 0;
	}

	//calculate colour (hue)
	//hue; red = 0, yellow = 60, green = 120
	if (scaled_level > 0.9)
	{
		//if pressure too high, then change to blue to indicate overflow or blocked vent
		hue = 240;
		number_to_light = no_leds;
	} 
	if (scaled_level <= 0) //pressure below zero level set to purple
	{
		hue = 355;
		number_to_light = no_leds;
	}
	if((scaled_level > 0) && (scaled_level <= 0.9)) //in set range, run from red to green
	{	
		hue = scaled_level * 120;
		number_to_light = uint8_t(scaled_level * no_leds) + 1;
	}


	//convert HSL to RGB values
	convert_hsl_rgb(hue, 1, float(max_brightness) / 255);

	//set leds
	for (uint8_t i = 0; i < no_leds; i++)
	{
		if (i < number_to_light)
		{
			leds[i].r = temp_r;
			leds[i].g = temp_g;
			leds[i].b = temp_b;
		} 
		else
		{
			leds[i].r = 0;
			leds[i].g = 0;
			leds[i].b = 0;
		}
	}
	
	//send to LEDs
	send();
}


//read ADC value
uint16_t get_adc(uint8_t sel_pin)
{
	//wait for current measurement to finish
	while((ADC0.COMMAND & ADC_STCONV_bm) == 1){}
	//select input
	ADC0.MUXPOS = sel_pin;
	//trigger reading
	ADC0.COMMAND |= ADC_STCONV_bm;
	//wait while reading taken
	while((ADC0.COMMAND & ADC_STCONV_bm) == 1){}
	//process result
	uint16_t result = (ADC0.RES/8); //divide by number of samples accumulated
	return result;
}


//convert HSL to RGB
void convert_hsl_rgb(float hue, float saturation, float lightness)
{	
	//calculate chroma
	float c = (1 - fabs(2 * lightness - 1)) * saturation;
	//
	float x = c * (1.0 - fabs(fmod(hue / 60.0, 2) - 1.0));

	//find appropriate hue and chroma point
	if ((hue >= 0) && (hue < 60))	
	{	temp_r = c;
		temp_g = x;
		temp_b = 0;
	}
	if ((hue >= 60) && (hue < 120))
	{	temp_r = x;
		temp_g = c;
		temp_b = 0;
	}
	if ((hue >= 120) && (hue < 180))
	{	temp_r = 0;
		temp_g = c;
		temp_b = x;
	}
	if ((hue >= 180) && (hue < 240))
	{	temp_r = 0;
		temp_g = x;
		temp_b = c;
	}
	if ((hue >= 240) && (hue < 300))
	{	temp_r = x;
		temp_g = 0;
		temp_b = c;
	}
	if ((hue >= 300) && (hue < 360))
	{	temp_r = c;
		temp_g = 0;
		temp_b = x;
	}

	//find rgb by adding same amount to each component to match lightness
	float m = lightness - (c / 2);

	temp_r = (temp_r + m) * 255;
	temp_g = (temp_g + m) * 255;
	temp_b = (temp_b + m) * 255;
}