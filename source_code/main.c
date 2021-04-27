/****************************************************************************
 * main.c>
 * Tomatotificatorul
 *
 * Copyright 2021 Iulian-Razvan Matesica <iulian.matesica@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *   http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include <inttypes.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ((64/1200000 * 256 * 73) + (64/1200000 * 135))) = 1s
 * 
 * This is close enough to 1s. The error for 24h is only 2.82s.
 * I can live with that.
 */

// /8
//#define TIMER_OVERFLOW_TICK  587
//#define TIMER_REMAINDER_TICK 255
// o ia inainte cu ~1s / ora 


#define TIMER_OVERFLOW_TICK  (uint16_t)4705
#define TIMER_REMAINDER_TICK 28

#define HOURLY_ERROR_SEC     1
#define FULL_DAY_TICKS       (((uint32_t)3600 + HOURLY_ERROR_SEC) * 24)

/* This not not precise, it's just an estimation. 
 * Don't use it for precise scheduling. 
 */

#define WATER_EVENT(hour, min, sec) \
    ((uint32_t)3600 * (hour) + \
     (uint32_t)60 * (min) + \
     (sec))

#define MAX(a, b)     ((a) > (b) ? (a) : (b))
#define ARRAY_LEN(a) (sizeof(a) / sizeof(a[0]))

/* Pin mapping::
 *   - PB0 - Pump
 *   - PB1 - Water button
 *   - PB2 - Status LED
 *   - PB3 (ADC3) - Duration adjustment
 *   - PB4 (ADC2) - Solar panel voltage
 */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void pump_init(void);
static void water_button_init(void);
static void status_led_init(void);
static void timer_init(void);
static void adc_init(void);
static uint16_t adc_read(uint8_t channel);
static void pump_water(uint8_t sec);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Counts TIMER0 overflows. */

static volatile uint16_t g_overflows;

/* Multiples of 16s since boot-up. */

static volatile uint32_t g_ticks;

/* Set to true from ISR when it's time 
 * to pump some water.
 */

static volatile bool g_water_plant;

/* Add as many "water plant" events as you wish. 
 * Note that the timing is not perfect, it's an estimation, since 
 * the "systick" has a period of 16s.
 */

static const uint32_t g_daily_events[] =
{
    /* WATER_EVENT(hour, minute, second) */

    WATER_EVENT(0, 0, 5),  /* Second event: 7 hours after boot-up. */
    WATER_EVENT(0, 1, 0),  /* Second event: 7 hours after boot-up. */
    WATER_EVENT(0, 5, 0),  /* Second event: 7 hours after boot-up. */
    WATER_EVENT(0, 10, 0),  /* Second event: 7 hours after boot-up. */
    WATER_EVENT(0, 30, 0),  /* Second event: 7 hours after boot-up. */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pump_init
 *
 * Description:
 *   Initializes the pump. 
 *
 * Input Parameters:
 *   None. 
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pump_init(void)
{
    /* Output pin:
     *   0: pump off
     *   1: pump on
     */

    DDRB |= (1 << PB0);
    PORTB &= ~(1 << PB0);
}

/****************************************************************************
 * Name: water_button_init
 *
 * Description:
 *   Configures the water button: interrupt on falling edge, internal 
 *   pull-up resistor enabled.
 *
 * Input Parameters:
 *   None. 
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void water_button_init(void)
{
    /* Configure INT0 for this button. 
     *   - input pin
     *   - internal pull-up resistor
     */

    DDRB &= ~(1 << PB1);
    PORTB |= (1 << PB1);

    /* Falling edge interrupt. */

    MCUCR |= (2 << ISC00);
    GIMSK |= (1 << INT0);
}

/****************************************************************************
 * Name: INT0_vect
 *
 * Description:
 *   ISR for INT0 - Water button. It is configured to trigger on the 
 *   falling edge. It sets the 'g_water_plant' flag to schedule an 
 *   immediate event.
 *
 * Input Parameters:
 *   None. 
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

ISR(INT0_vect)
{
    g_water_plant = true;
}

/****************************************************************************
 * Name: status_led_init
 *
 * Description:
 *   Initializes the status LED. 
 *
 * Input Parameters:
 *   None. 
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void status_led_init(void)
{
    /* Output pin; default off.
     * 
     * Configure and light the status LED 
     * for few seconds.
     */

    DDRB  |= (1 << PB2);
    PORTB |= (1 << PB2);
    //_delay_ms(2000);
    PORTB &= ~(1 << PB2);
}

/****************************************************************************
 * Name: timer_init
 *
 * Description:
 *   Initializes Timer0 to generate both overflow and match interrupts 
 *   which are used to schedule the water events, 2 per day.
 *
 * Input Parameters:
 *   None. 
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void timer_init(void)
{
    /* 16s period:
     *   - 61 overflows
     *   - 9 remainder ticks
     *   - prescaler 1024
     *   - F_CPU 1MHz
     */

    TCNT0 = 0;
    OCR0A = TIMER_REMAINDER_TICK;
    TIMSK0 |= (1 << TOIE0); // | (1 << OCIE0A); 
    //TCCR0A |= (1 << WGM01) | (1 << COM0A0);
    TCCR0B |= (1 << CS00);
}

/****************************************************************************
 * Name: TIM0_OVF_vect
 *
 * Description:
 *   Timer0 is free running and this is the ISR for overflow (when it 
 *   reaches 0xFF).
 *
 * Input Parameters:
 *   None. 
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

ISR(TIM0_OVF_vect)
{
    g_overflows++;

    if (g_overflows == TIMER_OVERFLOW_TICK)
    {
        TIMSK0 |= (1 << OCIE0A);
        g_overflows = 0;
    }
}

/****************************************************************************
 * Name: TIM0_COMPA_vect
 *
 * Description:
 *   ISR for Timer0 Output Compare A match interrupt. 
 *   It is used to count the time since boot-up. 
 *
 * Input Parameters:
 *   None. 
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

ISR(TIM0_COMPA_vect)
{
    TIMSK0 &= ~(1 << OCIE0A);
    g_ticks++;

    /* Should we water the plants? */
    
    for (int i = 0; i < ARRAY_LEN(g_daily_events); i++)
    {
        if (g_ticks == g_daily_events[i])
        {
            g_water_plant = true;
            break;
        }
    }

    if (g_ticks >= FULL_DAY_TICKS)
    {
        /* 1 day has passed. */

        g_ticks = 0;
    }
}

/****************************************************************************
 * Name: adc_init
 *
 * Description:
 *   Initializes the ADC and disables the digital block for the 
 *   analog pins. 
 *
 * Input Parameters:
 *   None. 
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void adc_init(void)
{
    DDRB &= ~((1 << PB3) | (1 << PB4));
    
    /* ADC prescaler 128. */

    ADCSRA |= (7 << ADPS0);

    /* Disable digital block for pins PB3 (ADC3) and PB4 (ADC2). */

    DIDR0 |= (1 << ADC2D) | (1 << ADC3D);
}

/****************************************************************************
 * Name: adc_read
 *
 * Description:
 *   Sample the request ADC channel. 
 *
 * Input Parameters:
 *   channel - The ADC channel to read.
 *
 * Returned Value:
 *   The ADC value in range [0, 1023].
 *
 ****************************************************************************/

static uint16_t adc_read(uint8_t channel)
{
    const uint8_t channel_mask = (1 << MUX1) | (1 << MUX0);
    uint16_t val = 0;

    /* Clear the MUX0 and MUX 1 bits. */
    
    ADMUX &= ~channel_mask;
    ADMUX |= channel & channel_mask;

    /* Turn on the ADC. */

    ADCSRA |= (1 << ADEN);

    /* Start the conversion. */

    ADCSRA |= (1 << ADSC);

    /* Wait for the conversion to end. */
    
    while ((ADCSRA & (1 << ADSC)));
    val = ADC; 

    /* Turn off the ADC. */
    
    ADCSRA &= ~(1 << ADEN);

    return val;
}

/****************************************************************************
 * Name: pump_water
 *
 * Description:
 *   Turn the pump on for 'sec' seconds. 
 *
 * Input Parameters:
 *   sec - Pump ON duration (in seconds).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pump_water(uint8_t sec)
{
    /* Pump 'sec' seconds of water. */

    PORTB |= (1 << PB0);
    
    for (int i = 0; i < sec; i++)
    {
        _delay_ms(1000);
    }

    PORTB &= ~(1 << PB0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
    /* Initialize all subsystems. */

    pump_init();
    
    water_button_init();
    
    adc_init();
    
    timer_init();

    status_led_init();
        
    /* Enable MCU sleep (idle). */

    sleep_enable();

    /* Enable global interrupts. */

    sei();

    while (true)
    {
        bool water = false;
        
        cli();
        water = g_water_plant;
        sei();

        if (water)
        {
            /* Read the duration adjustment potentiometer. */

            uint32_t duration = adc_read(3);

            /* y = ax + b
             *   x = 0 (0V)    => y = 5 sec  => b = 5
             *   x = 1023 (5V) => y = 60 sec
             *
             *  => a = 55 / 1023
             *  => y = 55 / 1023 * x + 5;
             */

            duration = 55 * duration / 1023 + 5;

            pump_water((uint16_t)duration);
            
            /* Reset the flag set from interrupt. */

            g_water_plant = false;
        }

        /* Save some power. */

        sleep_cpu();
    }

    return 0;
}
