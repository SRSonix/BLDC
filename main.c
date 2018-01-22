/*
 * File:   main.c
 * Author: Michael Meissner
 *
 * Created on 29. Oktober 2017, 11:38
 */

#define uint unsigned int 
#define ushort unsigned short 

#define PHASE_FLOAT 0
#define PHASE_PULL 1

#define PHASE_HIGH 1
#define PHASE_LOW 0

#define PHASE_A_FLOAT LATA5
#define PHASE_A_HL TRISA4 
#define PHASE_B_FLOAT LATC5
#define PHASE_B_HL TRISC4
#define PHASE_C_FLOAT LATC6
#define PHASE_C_HL TRISC7

#define PHASE_A_HL_LAT LATA4
#define PHASE_B_HL_LAT LATC4
#define PHASE_C_HL_LAT LATC7

#define PHASE_OUT LATC0

#define PWM_MAX 320

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
void setPWMperc(float percent);
void setPWM(ushort pwm_cycle);
float getADCPercentage();

void setOutPhase(uint phase);
void setInPhase(uint phase);
void setSection(uint section);

char send= '0';

uint section = 0;
ushort sectionLenth = 0; //in multiples of tmr2 tocs = fosc/4/64
ushort sectionLenthTP = 0;

ushort secLengthDes = 20; 
ushort pwmVal = 0;

void main(void) {
    device_init();
    setPWMperc(0.45);
    
    
    for(uint c=0;c<265;c++)
    {
          while(!TMR0IF);
          TMR0IF=0;

          if(!(c% 10))
          {
              section++;
              setOutPhase(section%6);
          }
    }
    
    
    for(uint c=0;c<265;c++)
    {
          while(!TMR0IF);
          TMR0IF=0;

          if(!(c% 5))
          {
              section++;
              setOutPhase(section%6);
          }
    }
    
    for(uint c=0;c<265;c++)
    {
          while(!TMR0IF);
          TMR0IF=0;

          if(!(c% 4))
          {
              section++;
              setOutPhase(section%6);
          }
    }
    
    for(uint c=0;c<1024;c++)
    {
          while(!TMR0IF);
          TMR0IF=0;

          if(!(c% 3))
          {
              section++;
              setOutPhase(section%6);
          }
    }
    
    section++;
    C1ON = 1;
    
    setOutPhase(10); //phase not in [0; 5] will set all floating
    setInPhase(section%6);
    
    C1IF = 0;
    C1IE = 1;
    
    for(uint c=0;c<1020;c++)
    {
          while(!TMR0IF);
          TMR0IF=0;
    }
    SSP1IE = 1;
    
    while(1)
    {
        
    }
}

void interrupt tc_int(void)
{
    if(C1IF && C1IE)
    {
        C1IF = 0;
        C1IE = 0;
        
        sectionLenth = (((unsigned int)TMR4) >> 1) + (sectionLenth >> 1);
        
        TMR4 = 0;
        
        TMR6 = 255 - (sectionLenth >> 1);
        TMR6ON = 1;
    }
    
    if(TMR6IF && TMR6IE)
    {
        TMR6IF=0;
        TMR6ON = 0;
        
        section++;
        setSection(section%6);      
    }
    
    
    if(SSP1IF && SSP1IE)
    {          
         SSP1IF=0;

         if(WCOL || SSPOV)
         {
             WCOL=0;
             SSPOV=0;
         }

         ushort buf=SSP1BUF;

         if( R_nW &&  //read recived
                 ( !(D_nA) || !(ACKSTAT)) ) //last was adress or ack was read
         {
             SSP1BUF= sectionLenth ;
         }
         else
         {
             setPWM((ushort)(
                     buf*2));
         }
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
            0 << _TRISA_TRISA2_POSITION |       //tris a2
            1 << _TRISA_TRISA4_POSITION |       //tris a4
            0 << _TRISA_TRISA5_POSITION;        //tris a5
    
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
    TRISC=  0 << _TRISC_TRISC0_POSITION |       //tris c0
            1 << _TRISC_TRISC1_POSITION |       //tris c1
            1 << _TRISC_TRISC2_POSITION |       //tris c2
            0 << _TRISC_TRISC3_POSITION |       //tris c3
            1 << _TRISC_TRISC4_POSITION |       //tris c4
            0 << _TRISC_TRISC5_POSITION |       //tris c5
            0 << _TRISC_TRISC6_POSITION |       //tris c6
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
    
    // pwm ccp2 on rc3
    PR2 = 0x4f;  // pwm freq = 100khz at fosc = 32mhz 8 bit res
    CCP2CON=0b1100 << _CCP2CON_CCP2M_POSITION | //PWM mode 
            0b00 << _CCP2CON_P2M_POSITION ; // singe output
            
    setPWM(0);
    //configure tmr2 for ccp2
    CCPTMRS0 &= ~_CCPTMRS0_C2TSEL_MASK; // mask c2tsel
    CCPTMRS0 |= 0x00 << _CCPTMRS0_C2TSEL_POSITION; //set c2tsel
    T2CON=  0b00 << _T2CON_T2CKPS_POSITION |    //tmr2 prescale 1
            1 << _T2CON_TMR2ON_POSITION;        //tmr2 enable
    
    //interrupt
    INTCON = 1 << _INTCON_GIE_POSITION | //global interrupt enable
             1 << _INTCON_PEIE_POSITION;  //Periferal interrupt enable
    
      //I2c slave mode 
    SSP1ADD = 0b0101010 << 1;  // 7 bit adress
    
    SSP1STAT =  1 << _SSP1STAT_SMP_POSITION | //standard speed mode
                0 << _SSP1STAT_CKE_POSITION; //somethign with SMbus specific... ?
                
    SSP1CON2 =  0 << _SSP1CON2_GCEN_POSITION | //disable interrupt on general call 
                0 << _SSP1CON2_ACKDT_POSITION; //enable ack on receive
    
    SSP1CON3 =  1 << _SSP1CON3_BOEN_POSITION |  //buffer override enable
                1 << _SSP1CON3_SDAHT_POSITION | // 300 ns hold of sda after scl falling edge
                0 << _SSP1CON3_AHEN_POSITION | //disable adress hold
                0 << _SSP1CON3_DHEN_POSITION; //disable data hold
    
    SSP1CON=   0b0110 << _SSP1CON1_SSPM_POSITION | //7 bit i2c mode
       1 << _SSP1CON1_CKP_POSITION | //enable clock
       1 << _SSP1CON1_SSPEN_POSITION; // enable

    SSP1IF = 0;
    
    //tmr 0 for controller and start commutation
    OPTION_REG = 0 << _OPTION_REG_TMR0CS_POSITION | // tmr 0 source is fosc/4
            1 << _OPTION_REG_TMR0SE_POSITION | // tmr 0 on falling edge
            0 << _OPTION_REG_PSA_POSITION | // tmr 0 has prescale
            0b11 << _OPTION_REG_PS_POSITION; // tmr 0 prescale is 16
     
    //tmr4 for measuring section length
    T4CON=  0b00 << _T2CON_T2CKPS_POSITION |    //tmr6 prescale 64
            1 << _T2CON_TMR2ON_POSITION |        //tmr6 enable
            0 << _T2CON_T2OUTPS_POSITION;  //tmr postscale 1
    
    //tmr6 delay section swith trigger by half a section
    T6CON=  0b00 << _T6CON_T6CKPS_POSITION |    //tmr6 prescale 64
            0 << _T6CON_TMR6ON_POSITION |        //tmr6 enable
            0 << _T6CON_T6OUTPS_POSITION;  //tmr postscale 1
    
    PIE3 = 1 << _PIE3_TMR6IE_POSITION;  //tmr 6 ienable
    
    
    //adc
   /* ADCON0= 4 << _ADCON0_CHS_POSITION | // chanel 4 select
            1 << _ADCON0_ADON_POSITION; //adc on

    ADCON1 = 0 << _ADCON1_ADFM_POSITION | // left justified
             0b110 << _ADCON1_ADCS_POSITION; //clocl fosc/64 = 2ns at fosc 32mhz
            */
    
    //comperator
    CM1CON0 = 0 << _CM1CON0_C1ON_POSITION | //enable c1
              1 << _CM1CON0_C1OE_POSITION   |  //enable c1out on RA2
              1 << _CM1CON0_C1POL_POSITION | // invert C1 
              1 << _CM1CON0_C1SP_POSITION | // hith speed mode
              0 << _CM1CON0_C1HYS_POSITION | //hysteresis enabled
              0 << _CM1CON0_C1SYNC_POSITION; // c1 is synced with timr1
    
    CM1CON1 = 1 << _CM1CON1_C1INTP_POSITION | //enable seting of C1IF on positive flank
              1 << _CM1CON1_C1INTN_POSITION | //enable seting of C1IF on negative flank
              0 << _CM1CON1_C1PCH_POSITION | // C1IN+ as pos channel
              0 << _CM1CON1_C1NCH_POSITION; // C12IN1- as neg channel (phase 0:A 1:B 2:C)
}

void setPWMperc(float percent)
{
    unsigned int pwm_cycle=(unsigned int) (320*percent);
    setPWM(pwm_cycle);
}

void setPWM(ushort pwm_cycle)
{
    if(pwm_cycle > PWM_MAX) pwm_cycle = PWM_MAX;
    
    DC2B0=pwm_cycle;
    DC2B1=pwm_cycle >> 1;
    CCPR2L = pwm_cycle >> 2;
    pwmVal = pwm_cycle;
}

float getADCPercentage()
{
    GO_nDONE = 1;
    while(GO_nDONE);
    return ADRESH  / 255.0;
}


void setSection(uint section)
{
    C1IE = 0;
    
    setOutPhase(section);
    setInPhase(section);
    
//    for(uint i=0;i<0;i++) NOP();  // kickback avoid
    
    C1IF = 0;
    C1IE = 1;
}

void setOutPhase(uint phase)
{
    switch(phase)
    {
    case 0:
        PHASE_OUT=1;
        
        PHASE_C_FLOAT = PHASE_FLOAT;
        
        PHASE_B_FLOAT = PHASE_PULL;
        PHASE_B_HL = PHASE_LOW;

        PHASE_A_FLOAT = PHASE_PULL;
        PHASE_A_HL = PHASE_HIGH;

        break;
    case 1:        
        PHASE_B_FLOAT = PHASE_FLOAT;
        
        PHASE_C_FLOAT = PHASE_PULL;
        PHASE_C_HL = PHASE_LOW;

        PHASE_A_FLOAT = PHASE_PULL;
        PHASE_A_HL = PHASE_HIGH;

        break;
    case 2:
        PHASE_A_FLOAT = PHASE_FLOAT;
        
        PHASE_C_FLOAT = PHASE_PULL;
        PHASE_C_HL = PHASE_LOW;
        
        PHASE_B_FLOAT = PHASE_PULL;
        PHASE_B_HL = PHASE_HIGH;
        
        break;
    case 3:
        PHASE_OUT=0;
        
        PHASE_C_FLOAT = PHASE_FLOAT;
        
        PHASE_A_FLOAT = PHASE_PULL;
        PHASE_A_HL = PHASE_LOW;
        
        PHASE_B_FLOAT = PHASE_PULL;
        PHASE_B_HL = PHASE_HIGH;

        break;
    case 4:
        PHASE_B_FLOAT = PHASE_FLOAT;
        
        PHASE_A_FLOAT = PHASE_PULL;
        PHASE_A_HL = PHASE_LOW;
        
        PHASE_C_FLOAT = PHASE_PULL;
        PHASE_C_HL = PHASE_HIGH;
        
        break;
    case 5:
        PHASE_A_FLOAT = PHASE_FLOAT;
        
        PHASE_B_FLOAT = PHASE_PULL;
        PHASE_B_HL = PHASE_LOW;
        
        PHASE_C_FLOAT = PHASE_PULL;
        PHASE_C_HL = PHASE_HIGH;

        break;
    }
}

void setInPhase(uint phase)
{
    C1NCH0 = 0;
    C1NCH1 = 0;
    
    switch(phase)
    {
    case 0: //phase c: C1NCH == 2  neg
    case 3:
        C1NCH1 = 1;
        break;
    case 1://phase b: C1NCH == 1 pos
    case 4:
        C1NCH0 = 1;    
        break;
    case 2: //phase a: C1NCH == 0 neg
    case 5:
        break;
    }
}