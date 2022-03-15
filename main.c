/*
 * File:   main.c
 * Author: Fabian Sohrmann
 * Email:  mfsohr@utu.fi
 * 
 * This program assists in playing the Dino game. When the LDR "sees" a cactus,
 * the servo is activated and presses down the space bar. The threshold value
 * can be calibrated with the trimpot, and the "hundreds" of the value are 
 * displayed.
 *
 * Created on 16. marraskuuta 2021, 18:14
 */


#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#define SERVO_PWM_PERIOD       (0x1046)
#define SERVO_PWM_DUTY_MAX     (0x01A0) //416
#define SERVO_PWM_DUTY_MIN     (0x00D0) //208
#define RTC_PERIOD             (0x0066) // ~100ms

/*
* This function is copied and modified from technical brief TB3213.
*/
void rtc_init()
{
    uint8_t temp;
    
    /* Initialize 32.768kHz Oscillator: */
    /* Disable oscillator: */
    temp = CLKCTRL.XOSC32KCTRLA;
    temp &= ~CLKCTRL_ENABLE_bm;
    /* Writing to protected register */
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);
    
    while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)
    {
        ; /* Wait until XOSC32KS becomes 0 */
    }
    
    /* SEL = 0 (Use External Crystal): */
    temp = CLKCTRL.XOSC32KCTRLA;
    temp &= ~CLKCTRL_SEL_bm;
    /* Writing to protected register */
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);
    
    /* Enable oscillator: */
    temp = CLKCTRL.XOSC32KCTRLA;
    temp |= CLKCTRL_ENABLE_bm;
    /* Writing to protected register */
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);
    
    /* Initialize RTC: */
    while (RTC.STATUS > 0)
    {
        ; /* Wait for all register to be synchronized */
    }
    
    /* 32.768kHz External Crystal Oscillator (XOSC32K) */
    RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
    
    /* Set period (overflow) to 100 ms */
    RTC.PER = RTC_PERIOD;
    
    /* Enable Overflow Interrupt */
    RTC.INTCTRL = RTC_OVF_bm; // 
}

int trimpot_read()
{   
    // Vdd as reference, prescaler for CLK_ADC to 16
    ADC0.CTRLC |= ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc;
    ADC0.MUXPOS = ADC_MUXPOS_AIN14_gc; // AIN14 (PF4) as trimpot input
    ADC0.CTRLA |= ADC_ENABLE_bm; // Enable ADC (default resolution 10)
    // Conversions start here
    ADC0.COMMAND = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
    {
        ;
    }
    return ADC0.RES;
}

int ldr_read()
{
    // Selecting internal voltage reference, prescaler to 16
    ADC0.CTRLC |= ADC_PRESC_DIV16_gc | ADC_REFSEL_INTREF_gc;
    VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc; // Selecting voltage level 2.5 V
    ADC0.MUXPOS = ADC_MUXPOS_AIN8_gc; // AIN08 (PE0) as LDR input
    ADC0.CTRLA |= ADC_ENABLE_bm;    
    // Conversions start here
    ADC0.COMMAND = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
    {
        ;
    }
    return ADC0.RES;
}

/*
 * This function moves the servo arm to the MAX position and starts the
 * RTC counter to ~100ms. Together with ISR below they move the arm back 
 * and forth.
 */
void servo_move()
{
    TCA0.SINGLE.CMP2BUF = SERVO_PWM_DUTY_MAX;
    // Start the counter. When ~100ms have passed raise overflow interrupt
    RTC.CTRLA = RTC_RTCEN_bm | RTC_PRESCALER_DIV32_gc; 
}

/*
 * This function is copied and modified from technical brief TB3213. This code
 * runs when RTC triggers overflow interrupt (100ms have passed). The counter
 * is disabled and the arm moved back to its MIN position.
*/
ISR(RTC_CNT_vect)
{
    //Clear flag by writing '1': 
    RTC.INTFLAGS = RTC_OVF_bm;
    RTC.CTRLA = ~RTC_RTCEN_bm; // Disable counter, ~100ms have passed
    TCA0.SINGLE.CMP2BUF = SERVO_PWM_DUTY_MIN; // Move servo arm to start pos.
}

int main(void) 
{
    sei(); // Enable interrupts
    rtc_init(); // Initialize rtc
    
    uint8_t number[] =
    {
        0b00111111,     // 0
        0b00000110,
        0b01011011,
        0b01001111,
        0b01100110,
        0b01101101,     // 5
        0b01111101,
        0b00000111,
        0b01111111,
        0b01101111,     // 9
        0b01110111      // A           
    };

    // Display, setting all pins in PORTC to output
    PORTC.DIRSET = 0xFF;
    
    // Transistor connects display ground to GND
    PORTF.DIRSET = PIN5_bm; 
    PORTF.OUTSET = PIN5_bm;
    
    // Configuring PE0 and PF4 for ADC0 input
    // Setting to input, disabling input buffer (pull-up disabled by default)
    PORTF.DIRCLR = PIN4_bm;
    PORTF.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTE.DIRCLR = PIN0_bm;
    PORTE.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;

    // Creating PWM-signal for servo-control
    PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTB_gc; // Routing TCA0 to PORTB
    PORTB.DIRSET = PIN2_bm; // PB2 to output
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc; // TCA0 prescaler to 16
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_WGMODE_SINGLESLOPE_gc; // Single slope PWM
    TCA0.SINGLE.PERBUF = SERVO_PWM_PERIOD; // Period length
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP2EN_bm; // Enable Compare Channel 2
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; // Enable TCA0 
    
    TCA0.SINGLE.CMP2BUF = SERVO_PWM_DUTY_MIN; // Servo to MIN
    
    int old_value = trimpot_read(); // Used for display refresh
    PORTC.OUTSET = number[old_value/100]; // Display with first measured value
    while (1) 
    { 
        int value = trimpot_read();
        // Refresh display only when the hundreds of the treshold change
        if ((old_value/100)!=(value/100)) 
        {
            PORTC.OUTCLR = 0xFF;
            PORTC.OUTSET = number[value/100];
            old_value=value;
        }
        
        int ldr = ldr_read();
        int trimpot = trimpot_read();
        if(ldr > trimpot) // Compare ldr to threshold
        {
            servo_move();
        }
    }
}