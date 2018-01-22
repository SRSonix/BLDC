/*
 * File:   main.c
 * Author: Michael Meissner
 *
 * Created on 29. Oktober 2017, 11:38
 */

#define uint unsigned int 
#define ushort unsigned short 
#define byte unsigned char

#define PWM_MAX 128
#define PWM_MIN 32
#define PWM_SLOPE  0.375

#define SLAVE_ADRESS 0b0101010

// PIC16F1828 Configuration Bit Settings

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON      // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#include <xc.h>

void device_init();
byte getADCVal();
void i2cTransmit(byte adress,byte data);

uint pwmVal = 0;

void main(void) {
    device_init();

    TMR0IF = 0;
    while(1)
    {
        while(!TMR0IF);
        TMR0IF = 0;
        
        TRISC3 = !TRISC3;
        pwmVal = (uint)(getADCVal() * PWM_SLOPE) + PWM_MIN;
        
        i2cTransmit(SLAVE_ADRESS,pwmVal);
    }
}

void device_init()
{
     // oscilator control
    OSCCON= 1 << _OSCCON_SPLLEN_POSITION |      //PULL ENABLE
            0b1110 << _OSCCON_IRCF_POSITION |     //INTERNAL OSC FREQ
            0b00 << _OSCCON_SCS_POSITION;         //SYSTEM CLOCK SELECT
    
    //alternative pin fnk contr
    APFCON0=0;
    APFCON1= 0 << _APFCON1_CCP2SEL_POSITION;
    
    //Port a config
    // tris 1 in 0 out
    TRISA=  1 << _TRISA_TRISA0_POSITION |       //tris a0
            1 << _TRISA_TRISA1_POSITION |       //tris a1
            1 << _TRISA_TRISA2_POSITION |       //tris a2
            1 << _TRISA_TRISA4_POSITION |       //tris a4
            1 << _TRISA_TRISA5_POSITION;        //tris a5
    
    //ansel 0 digi io 1 an in
    ANSELA =0 << _ANSELA_ANSA0_POSITION |       //ansel a0
            0 << _ANSELA_ANSA1_POSITION |       //ansel a1
            0 << _ANSELA_ANSA2_POSITION |       //ansel a2
            0 << _ANSELA_ANSA4_POSITION;        //ansel a4
    
    //Port b config
    // tris 1 in 0 out
    TRISB=  1 << _TRISB_TRISB4_POSITION |       //tris b4
            1 << _TRISB_TRISB5_POSITION |       //tris b5
            1 << _TRISB_TRISB6_POSITION |       //tris b6
            1 << _TRISB_TRISB7_POSITION;        //tris b7
    
    //ansel 0 digi io 1 an in
    ANSELB =0 << _ANSELB_ANSB4_POSITION |       //ansel B0
            0 << _ANSELB_ANSB5_POSITION;       //ansel B1
    
    //Port c config
    // tris 1 in 0 out
    TRISC=  1 << _TRISC_TRISC0_POSITION |       //tris c0
            1 << _TRISC_TRISC1_POSITION |       //tris c1
            1 << _TRISC_TRISC2_POSITION |       //tris c2
            0 << _TRISC_TRISC3_POSITION |       //tris c3
            1 << _TRISC_TRISC4_POSITION |       //tris c4
            1 << _TRISC_TRISC5_POSITION |       //tris c5
            1 << _TRISC_TRISC6_POSITION |       //tris c6
            1 << _TRISC_TRISC7_POSITION;        //tris c7
    
    //ansel 0 digi io 1 an in
    ANSELC =0 << _ANSELC_ANSC0_POSITION |       //ansel a0
            0 << _ANSELC_ANSC1_POSITION |       //ansel a1
            0 << _ANSELC_ANSC2_POSITION |       //ansel a2
            0 << _ANSELC_ANSC3_POSITION |       //ansel a3
            0 << _ANSELC_ANSC6_POSITION |       //ansel a6
            0 << _ANSELC_ANSC7_POSITION;        //ansel a7
    
    LATA = 0;
    LATB = 0;
    LATC = 0;
    
    //I2c master mode 
    SSP1ADD = 0x4F;  // baud gen value: 100kHz
    
    SSP1STAT =  1 << _SSP1STAT_SMP_POSITION | //standard speed mode
                0 << _SSP1STAT_CKE_POSITION; //somethign with SMbus specific... ?
                
    SSP1CON2 =  1 << _SSP1CON2_ACKDT_POSITION; //enable ack on receive
    
    SSP1CON3 =  1 << _SSP1CON3_SDAHT_POSITION | // 300 ns hold of sda after scl falling edge
                0 << _SSP1CON3_AHEN_POSITION | //disable adress hold
                0 << _SSP1CON3_DHEN_POSITION; //disable data hold

    //this is last since it containts enable SSP
    SSP1CON=   0b1000 << _SSP1CON1_SSPM_POSITION | //master mode clock = FOSC/(4*(1 + SSP1ADD) )
       1 << _SSP1CON1_SSPEN_POSITION; // enable

    SSP1IF = 0;
    
    //tmr 0 for controller and start commutation
    OPTION_REG = 0 << _OPTION_REG_TMR0CS_POSITION | // tmr 0 source is fosc/4
            1 << _OPTION_REG_TMR0SE_POSITION | // tmr 0 on falling edge
            0 << _OPTION_REG_PSA_POSITION | // tmr 0 has prescale
            0b111 << _OPTION_REG_PS_POSITION; // tmr 0 prescale is 256
        
    //adc
    ADCON1 = 0 << _ADCON1_ADFM_POSITION | // left justified
             0b110 << _ADCON1_ADCS_POSITION | //clocl fosc/64 = 2ns at fosc 32mhz
             0 << _ADCON1_ADNREF_POSITION | // Vss as - ref
             0 << _ADCON1_ADPREF_POSITION; // Vdd as + ref
    
    ADCON0= 2 << _ADCON0_CHS_POSITION | // chanel 2 select
            1 << _ADCON0_ADON_POSITION; //adc on

}

byte getADCVal()
{
    GO_nDONE = 1;
    while(GO_nDONE);
    return ADRESH;
}

void i2cTransmit(byte adress,byte data)
{
    SEN = 1; //set start con
    SSP1IF = 0;
    while(!SSP1IF); //wait for flag to confirm start
    
    SSP1BUF = (unsigned int) (adress << 1);
    SSP1IF = 0;
    while(!SSP1IF); //wait for flag to confirm adress write
    
    if(ACKSTAT == 0) //if ack was receaved
    {
        SSPBUF = data;
        SSP1IF = 0;
        while(!SSP1IF); //wait for flag to confirm data write
    }
    
    PEN = 1;
    SSP1IF = 0;
    while(!SSP1IF); //wait for flag to confirm stop cond
    
}