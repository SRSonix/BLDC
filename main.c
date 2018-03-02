/*
 * File:   main.c
 * Author: Michael Meissner
 * mail: michi.meissner@tum.de
 *
 * Created on 29. Oktober 2017, 11:38
 */

//type definitions
#define uint unsigned int 
#define ushort unsigned short 
#define bool unsigned char
#define true 1
#define false 0

typedef enum{
    normal = 0,
    fieldweaken
}mode;

//defines of the PHASE Pin values
#define PHASE_FLOAT 0
#define PHASE_PULL 1

#define PHASE_HIGH 1
#define PHASE_LOW 0

//defines of the phase pins
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

//define of the PWM duty circle 100% device should be initialized accordingly
#define PWM_MAX 320

//adress of the device
#define ADRESS 0b0101010

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

//xc device header
#include <xc.h>

//inits the device: inits tmr 0,4,6,PWM module(using tmr2), pin configuration, osccilator, interrupts, I2C
void device_init();
//sets pem duty circle to a certain value [0 PWM_MAX]
void setPWM(ushort pwm_cycle);

//sets phase outputs accortding to section [0,5]. sets all phases floating else
void setOutSection(uint section);
//sets IOC interrupt to right pin and flank direction accorting to section [0,5] disables IOC else
void setInSection(uint section);
// deactivates IOC, calls setOutSection(),setInSction(), reactivates IOC
void setSection(uint section);

//section the motor is in
uint section = 0;
// length of a section with first order TP. in multibles of tmr 4 tics = 4*64/fosc
uint sectionLenth = 0;
// 4* section length for higher order TP currently unused. in multibles of tmr 4 tics = 4*64/fosc
int sectionLength4 = 0;

//current value of pwm
ushort pwmVal = 0;
//current mode
mode curr_mode = fieldweaken;

void main(void) {
    //init device and set starting PWM level
    device_init();
    setPWM(48);
    
    //300 steps of forced commutation
    for(uint c=0;c<300;c++)
    {
          while(!TMR0IF);
          TMR0IF=0;

          if(!(c% 10))
          {
              section++;
              setOutSection(section%6);
          }
    }
    
    //set up for free commutation start
    setOutSection(10); //set all phases floating
    setInSection(section%6);//select right input level
    
    IOCIE = 1;//enable IOC interrupt for free commutation
        
    //wait for some time (512*8us) and swich to normal commutation mode
    for(uint c=0;c<512;c++)
    {
          while(!TMR0IF);
          TMR0IF=0;
    }
    curr_mode = normal;
    
    //wait for some time (512*8us) before start of comunicatioon
    for(uint c=0;c<512;c++)
    {
          while(!TMR0IF);
          TMR0IF=0;
    }
    
    while(1)
    {
        //I2C communication: if I2C iflag is set
        if(SSP1IF)
        {    
            //reset I2C flag
            SSP1IF=0;

            //clear error flags
            if(WCOL || SSPOV)
            {
                WCOL=0;
                SSPOV=0;
            }

            if( R_nW )  //read recived
            {
               if( !(D_nA) || !(ACKSTAT)) //last was adress or ack was read
               {
                   //write transmit sectionLength and stop clock abitration
                   SSP1BUF= sectionLenth ;
                   CKP = 1;
               }
            }
            else //write request reciefed
            { 
                ushort buf = SSP1BUF; 
                if( D_nA )//last adress was data
                {
                   //read data and set PWM accordingly  
                   setPWM(buf);
                }
            }
        }
    }
}

//interrupt service routine
void interrupt tc_int(void)
{
    //Interrupt on change was detected on floating phase
    if(IOCAF && IOCIE)
    {
        IOCAF = 0;
        //turn of IOC until next phase IOC will be turned on in setInputSection called by setSection 
        IOCIE = 0;
             
        //toggle PIN for debug
        PHASE_OUT = !PHASE_OUT;
        NOP();
        PHASE_OUT = !PHASE_OUT;

//this enables higher order lowpass for sectionLength. makes kommutation slower
#if 0   
        //higher order LP of sectionLength measured by TMR4 tocs
        int t4 = TMR4 << 2;
        sectionLength4 += (t4 - sectionLength4) >> 2;
        sectionLenth = (ushort)(sectionLength4>>2);
#else     
        //first order LP of sectionLength measured in TMR4 tocs
        sectionLenth = ( ((uint)TMR4 >> 1) + (sectionLenth >> 1));
#endif

        //restart TMR4 to time next section length
        TMR4 = 0;

        if(curr_mode == normal) //mode is normal mode with commutation timing
        {
            //calculate delay in TMR6 tocs (each aprox. 8us): sectionLength/2 + swichtime
            //measured time to swich section (runing of setSection() method) is approx 70 us. 9 timer  tics are approx 72 us
            int delay = (sectionLenth >> 1) - 9; 
            if(delay > 0) //if delay positive
            {
                //set TMR6 value and turn on timer
                TMR6 = (ushort)(255 - ( delay )) ; 
                TMR6ON = 1; 
            }
            else //if delay negative
            {
                //swich section right away
                section++;
                setSection(section%6);   
            }
        }
        else // mode is fielweaken
        {
            //swich section as soon as IOC is detected. no TMR6 action
            section++;
            setSection(section%6);  
        }

        //toggle pin for debug
        PHASE_OUT = !PHASE_OUT;
        NOP();
        PHASE_OUT = !PHASE_OUT;
    }
    
    //if TMR6 interrupt
    if(TMR6IF && TMR6IE)
    {
        TMR6IF=0;
        //turn off tmr6 (will be turned on in IOC)
        TMR6ON = 0;
        
        //advance section
        section++;
        setSection(section%6);      
    }
}

//init of all functions
void device_init()
{
     // oscilator control
    OSCCON= 1 << _OSCCON_SPLLEN_POSITION |      //PULL ENABLE
            0b1110 << _OSCCON_IRCF_POSITION |     //INTERNAL OSC FREQ
            0b00 << _OSCCON_SCS_POSITION;         //SYSTEM CLOCK SELECT
    
    //alternative pin fnk contr
    APFCON0=0;
    APFCON1=0;
    
    //Port a config
    // tris 1 in 0 out
    TRISA=  1 << _TRISA_TRISA0_POSITION |       //tris a0
            1 << _TRISA_TRISA1_POSITION |       //tris a1
            1 << _TRISA_TRISA2_POSITION |       //tris a2
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
    
    //set all high logic levels to high levels fpr comperator inputs
    INLVLA = 0b111;
    
    //set all output lats to 0
    LATA = 0;
    LATB = 0;
    LATC = 0;
    
    // pwm ccp2 on rc3
    PR2 = 0x4f;  // pwm freq = 100khz at fosc = 32mhz 8 bit res
    CCP2CON=0b1100 << _CCP2CON_CCP2M_POSITION | //PWM mode 
            0b00 << _CCP2CON_P2M_POSITION ; // singe output
            
    setPWM(0);
    //configure tmr2 for ccp2 pwm
    CCPTMRS0 &= ~_CCPTMRS0_C2TSEL_MASK; // mask c2tsel
    CCPTMRS0 |= 0x00 << _CCPTMRS0_C2TSEL_POSITION; //set c2tsel
    T2CON=  0b00 << _T2CON_T2CKPS_POSITION |    //tmr2 prescale 1
            1 << _T2CON_TMR2ON_POSITION;        //tmr2 enable
    
    //interrupt
    INTCON = 1 << _INTCON_GIE_POSITION | //global interrupt enable
             1 << _INTCON_PEIE_POSITION;  //Periferal interrupt enable
    
      //I2c slave mode 
    SSP1ADD = ADRESS << 1;  // 7 bit adress
    
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
    T4CON=  0b11 << _T2CON_T2CKPS_POSITION |    //tmr6 prescale 64
            1 << _T2CON_TMR2ON_POSITION |        //tmr6 enable
            0 << _T2CON_T2OUTPS_POSITION;  //tmr postscale 3
    
    //tmr6 delay section swith trigger by half a section
    T6CON=  0b11 << _T6CON_T6CKPS_POSITION |    //tmr6 prescale 64
            0 << _T6CON_TMR6ON_POSITION |        //tmr6 enable
            0 << _T6CON_T6OUTPS_POSITION;  //tmr postscale 1
    
    PIE3 = 1 << _PIE3_TMR6IE_POSITION;  //tmr 6 ienable
    
    
    //adc
   /* ADCON0= 4 << _ADCON0_CHS_POSITION | // chanel 4 select
            1 << _ADCON0_ADON_POSITION; //adc on
    ADCON1 = 0 << _ADCON1_ADFM_POSITION | // left justified
             0b110 << _ADCON1_ADCS_POSITION; //clocl fosc/64 = 2ns at fosc 32mhz
            */
    
    //IOC on A0 A1 A2 for phases ABC
    IOCAP = 0; // disactive for 0,1,2
    IOCAN = 0; // disactive for 0,1,2
}

//sets PWM dutcycle to the value
void setPWM(ushort pwm_cycle)
{
    //check if val in bounds
    if(pwm_cycle > PWM_MAX) pwm_cycle = PWM_MAX;
    
    //set val to right regisers
    DC2B0=pwm_cycle;
    DC2B1=pwm_cycle >> 1;
    CCPR2L = pwm_cycle >> 2;
    //set global variable to save current PWM dutycycle value
    pwmVal = pwm_cycle;
}

//sets section to value
void setSection(uint section)
{
    //deactivate IOC
    IOCIE = 0;
    
    //set Phases outputs to drive phases according to section plan
    setOutSection(section);
    //sets IOC to listen to right phase according to section plan
    setInSection(section);
    
    //turn on IOC
    IOCAF = 0;
    IOCIE = 1;
}

//sets phase outputs accrding to section plan
void setOutSection(uint phase)
{
    //swich depending on phase and set one phase floating, one low and one high, toggle phaseout
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
        PHASE_OUT=0;
        PHASE_B_FLOAT = PHASE_FLOAT;
        
        PHASE_C_FLOAT = PHASE_PULL;
        PHASE_C_HL = PHASE_LOW;

        PHASE_A_FLOAT = PHASE_PULL;
        PHASE_A_HL = PHASE_HIGH;

        break;
    case 2:
        PHASE_OUT=1;
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
        PHASE_OUT=1;
        PHASE_B_FLOAT = PHASE_FLOAT;
        
        PHASE_A_FLOAT = PHASE_PULL;
        PHASE_A_HL = PHASE_LOW;
        
        PHASE_C_FLOAT = PHASE_PULL;
        PHASE_C_HL = PHASE_HIGH;
        
        break;
    case 5:
        PHASE_OUT=0;
        PHASE_A_FLOAT = PHASE_FLOAT;
        
        PHASE_B_FLOAT = PHASE_PULL;
        PHASE_B_HL = PHASE_LOW;
        
        PHASE_C_FLOAT = PHASE_PULL;
        PHASE_C_HL = PHASE_HIGH;
 
        break;
        
    // if inVal is not in [0,6] set all phases to float
    default:
        PHASE_A_FLOAT = PHASE_FLOAT;
        PHASE_B_FLOAT = PHASE_FLOAT;
        PHASE_C_FLOAT = PHASE_FLOAT;
    }
}

//set IOC to correct flance and phase according to section plan
void setInSection(uint phase)
{
    //disable all IOC (on PORTA)
    IOCAP = 0; // disactive positive flanc for 0,1,2
    IOCAN = 0; // disactive negative flanc for 0,1,2
    
    //according to phase activate right IOC 
    switch(phase)
    {
    case 0: //phase c:   neg
        IOCAN2 = 1;
        break;
    case 1: //phase b:  pos
        IOCAP1 = 1;
        break;
    case 2: //phase a:  neg
        IOCAN0 = 1;
        break;
    case 3: //phase c:   pos
        IOCAP2 = 1;
        break;
    case 4: //phase b:  neg
        IOCAN1 = 1;
        break;
    case 5: //phase a:  pos
        IOCAP0 = 1;
        break;
    }
}
