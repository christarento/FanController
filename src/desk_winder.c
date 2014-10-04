/*
 * Fan Controller
 * Copyright (C) 2014  Christopher Tarento
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */ 

#define F_CPU 4800000UL
#define F_PWM 18750UL

#include <limits.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>


//------------ DEFINES
#define TRUE 1
#define FALSE 0
#define STARTUP_PWM 200
#define STARTUP_TIME (F_PWM * 2)
#define STALL_THRESHOLD_TIME (F_PWM / 2)
#define STALL_RETRY_TIME (F_PWM * 2)
#define MIN_PWM_DUTY 40

//------------ FUSES
//- high byte -> 0xff

//- low byte -> 0x79
//CKDIV8[0] 1 : Clock divider -> NO
//SUT[1:0] 10 : Start-up times -> Slowly rising time
//CKSEL[1:0] 01 Internal RC Oscillator -> 4.8 MHz

//------------ I/O
//PB0 (OC0A) PWM control @21-28KHz output
//PB1 (INT0) tach input two pulses by revolution
//PB4 (ADC2) potz input
#define CTRL_PIN 0
#define TACH_PIN 1
#define POTZ_PIN (1<<MUX1)


//! count time since last sensor signal
volatile uint32_t g_stall_time = 0;
//! used to measure time
volatile uint32_t g_time_counter = 0;

/*! \brief Flags for motor state
 * uses C bit fields
 */
volatile struct
{
	//! true when motor stalled
	uint8_t stall : 1;
	//! true when starting motor
	uint8_t starting : 1;
	//! padding to 8 bits
	uint8_t : 6;
} g_fan = {FALSE, FALSE};


/*! \brief Pin Change Interrupt vector
 */
ISR(PCINT0_vect)
{
	// reset time since last sensor change
	g_stall_time = 0;
}

/*! \brief Timer 0 Overflow Interrupt vector
 */
ISR(TIM0_OVF_vect)
{
	// increment time counter as long as not overflowing
	if (g_time_counter < UINT32_MAX)
	{
		++g_time_counter;
	}

	// increment as long as not at stall limit, otherwise raise stall flag
	if (g_stall_time < STALL_THRESHOLD_TIME)
	{
		++g_stall_time;
	}
	else
	{
		g_fan.stall = TRUE;
	}
	
	// check fan startup end
	if (g_fan.starting == TRUE && g_time_counter > STARTUP_TIME)
	{
		// fan has finished its startup phase
		g_fan.starting = FALSE;
	}
}


/*! \brief init clock, adc, pwm ...
 */
inline void init(void)
{
	//== PORT configuration
	//CTRL_PIN direction as output
	DDRB = (1<<CTRL_PIN);

	//== Pin change Interrupt
	PCMSK = (1<<TACH_PIN);//pin change mask register, activate TACH_PIN
	GIMSK = (1<<PCIE);//pin change interrupt enable
	
	//== TIMER / PWM configuration
	// fast PWM, prescaler 1, Fpwm = F_CPU / (prescaler*256) -> 18750Hz
	TCCR0A = (1<<WGM01) | (1<<WGM00);//set OC0A at TOP, Fast PWM with TOP=0xff
	TCCR0B = (1<<CS00);//no prescaler (1)
	TCNT0 = 0;//Timer/Counter register to 0
	TIMSK0 = (1<<TOIE0);//Timer/Counter0 Overflow Interrupt Enable
		
	//== Analog to Digital Converter configuration
	// prescaler of ADC clock to 128 and enable ADC
	ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN);
	// VCC as ref, left adjusted reading, POTZ_PIN as input (secured by mask)
	ADMUX = (0<<REFS0) | (1<<ADLAR) | (0b0011 & POTZ_PIN);
		
	//== Global interrupt
	sei();
}

/*! \brief Reads ADC input
 * \return read value 0-255
 */
uint8_t potzRead(void)
{
    // clear ADIF (write logical 1 to store 0) and start single conversion
    ADCSRA |= (1 << ADIF) | (1 << ADSC);
    // wait for conversion done -> ADIF flag active
    while (!(ADCSRA & (1 << ADIF)));
    // read out ADCH register
    return ADCH;
}

inline void setPower(uint8_t a_duty_cycle)
{
	if (a_duty_cycle == 0)
	{
		// PWM generator cannot give pure 0% duty cycle -> 1 clock cycle narrow spike 
		// we disconnect PWM generator and set output to 0 by hand
		TCCR0A &= ~(1<<COM0A1);
		PORTB &= ~(1<<CTRL_PIN);
	}
	else
	{
		// set PWM generator duty cycle
		OCR0A = a_duty_cycle;
		TCCR0A |= (1<<COM0A1);//clear OC0A on Compare Match
	}
}

/*! \brief Startup Pulse
 * Increase PWM value (if needed) for a short period of time to ensure that fan is starting
 * \param a_duty Preferred duty cycle. It can be increased to STARTUP_PWM for a short period
 */ 
void startFan(uint8_t a_duty)
{
	if (a_duty < STARTUP_PWM)
	{
		a_duty = STARTUP_PWM;
	}
	
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		g_fan.stall = FALSE;
		g_fan.starting = TRUE;
		g_stall_time = 0;
		g_time_counter = 0;
	}
	setPower(a_duty);
}

/*! \brief main
 */
int main(void)
{
	//! temporary duty value
	uint8_t duty;
	//! temporary time counter
	uint32_t time_counter;
	
	init();
	startFan(potzRead());
	
    while (1)
    {
        duty = potzRead();
		if (duty < MIN_PWM_DUTY)
		{
			duty = MIN_PWM_DUTY;
		}
		
		// do not update power when starting
		if (g_fan.starting == FALSE)
		{
			setPower(duty);
		}
		
		// stalled rotor : power off, wait and restart
		if (g_fan.stall == TRUE)
		{
			// power off
			setPower(0);
			
			// reset time counter
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				g_time_counter = 0;
			}
			
			// wait until delay finished
			do
			{
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					time_counter = g_time_counter;
				}
			}
			while (time_counter <= STALL_RETRY_TIME);
	
			// start up the fan again
			startFan(duty);
		}
    }
}
