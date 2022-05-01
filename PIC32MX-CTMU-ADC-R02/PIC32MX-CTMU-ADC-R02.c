//!----------------------------------------------------------------------------
//! electronics-lis
//! CH-2000 Neuchatel
//! info@electronics-lis.com
//! https://electronics-lis.com
//! L. Lisowski April 2022
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
//! PIC32MK ADC tests  REVISION 03
//!----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
//! check where is <p32mk1024mcf064.h> file on your PC
#include <C:\Program Files\Microchip\xc32\v2.50\pic32mx\include\proc\PIC32MX\p32mx250f128b.h>
#include <xc.h>            
#include <sys/attribs.h> 
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
//! This part need to be adapted for your application.
// PIC32MX250F128B Configuration Bit Settings
// 'C' source line config statements
// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)
// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_19         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_4         // USB PLL Input Divider (2x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)
// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = ON            // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_4           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)
// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)
//! That are my Configuration Bits version, you need adapt them for your applicatio
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
/*********************************/
//! Type declarations
/*********************************/
typedef signed char  int8;
typedef unsigned char uint8;
typedef short int16;
typedef unsigned short uint16;
typedef signed int int32;
typedef unsigned int uint32;
/************************************//**
** Macro Definitions
****************************************/
//! Don't forget !!! the result are uint32_l !!!!!!!!
#define TSTBIT(D,i) (D>>i & 1)
#define SETBIT(D,i) (D |= ((uint32)1<<i))
#define CLRBIT(D,i) (D &= ~((uint32)1 << i))
//#define SETPARA(D,p,i) (D |= (p<<i)
#define ADC_MAX_LOOPS           100
#define SYSCLK                  36864000L
#define PBCLK                   (SYSCLK/4) //! voir FPBDIV = DIV_4
#define UARTspeed               115200
#define TRUE                    1
#define FALSE                   0
/*********************************/
//! Local Variables
/*********************************/
char i, k;
uint16  cnt = 16, count, loopCount = 5;
/*********************************/
//! Function prototype
/*********************************/
void initU1ART(void);
void initANALOG(void);
uint8 initADC(void);
/*********************************/
//! main 
void main()
{  
    // Function initialization
    //__builtin_enable_interrupts();  // global interrupt enable
    __XC_UART = 1;                  // printf on the U1ART    
    //INTCONbits.MVEC = 1; // Set the interrupt controller for multi-vector mode  
    PMCONbits.ON = 0;               //! no PMP  
    // Function initialization
    initU1ART();
    printf ("\n\r This is my first CTMU and PIC32MX250F128B test \n\r"); 
    initANALOG();       //! Analog part initialization
    if(initADC() == TRUE){
        printf ("\n\r initADC accomplished \n\r");
    } 
    else{
        printf ("\n\r initADC not accomplished \n\r"); 
    }       
    while (1) 
	{
        //U1TXREG = 'a'; 
        while( !U1STAbits.URXDA);   //! wait until data available in RX buffer
        i = U1RXREG;                //1 Hit any key
        while( U1STAbits.UTXBF);    //! wait while TX buffer full
        U1TXREG = i;                //! Echo
        while( !U1STAbits.TRMT);    //! wait for last transmission to finish*/
        if (i == 'c')               //! U1ART test
        {
            printf("\n\r Start\n\r");
            SETBIT(TRISA,0); SETBIT(ANSELA,0);     //! A0 pin analogue input
            SETBIT(CTMUCON,9);          //! IDISSEN = 1; Analog current source output is grounded
                printf(" idissen \n\r"); //! Delay
            CLRBIT(CTMUCON,9);          //! IDISSEN = 0; Analog current source output is not grounded
            SETBIT(AD1CON1,1);          //! SAMP = 1; The ADC sample and hold amplifier is sampling
            CLRBIT(CTMUCON,25);         //! EDG2STAT = 0; Ground charge pump
            SETBIT(CTMUCON,24);         //! EDG1STAT = 1; start charging
            SETBIT(AD1CON1,1);          //! SAMP = 1; The ADC sample and hold amplifier is sampling
                for (count = 0; count < loopCount; count++);  //! Tacq delay
            CLRBIT(AD1CON1,1);          //! SAMP = 0;  stop acquisition start conversion
            //! Discharging before conversion
            //CLRBIT(TRISA,0); CLRBIT(ANSELA,0);     //! A0 digital output
            //CLRBIT(LATA,0);             //! discharge external capacitor
            while (!AD1CON1bits.DONE){}; // Wait for ADC conversion
            //! Discharging after conversion
            CLRBIT(TRISA,0); CLRBIT(ANSELA,0);     //! A0 digital output
            CLRBIT(LATA,0);             //! discharge external capacitor
                printf(" Result = %lu \n\r Stop",ADC1BUF0 );           
        }
      }
	}
/*********************************/
void initU1ART(void)
{
    // UART init
    // Pins initialization. !!!!! have to be adapted to hardware        
    //TRISGbits.TRISG8= 1;       //RG8 digital input
    //U1RXRbits.U1RXR = 10;        //SET RX to RG8
    //RPG6Rbits.RPG6R = 1;        //SET RG6 to TX 
    // Based on OneAxe configuration
    //TRISGbits.TRISG8= 1;       //RG8 digital input
    //U1RXRbits.U1RXR = 10;        //SET RX to RG8
    //RPG6Rbits.RPG6R = 1;        //SET RG6 to TX
    // Based on PIC32MX250F128B configuration
    //! U1RX
    TRISBbits.TRISB4= 1;       //RB4 digital input ==> short circuit B4 and A4
    TRISAbits.TRISA4= 1;       //RB4 digital input
    U1RXRbits.U1RXR = 2;        //SET RX to RB4
    //! U1TX
    TRISBbits.TRISB5= 1;        //short circuit B5 as input and B7
    TRISBbits.TRISB7= 0;        //RB7 digital output ==> short circuit B5 as input and B7
    RPB7Rbits.RPB7R= 1;        //SET RB7 to TX
    // disable UART1 and autobaud, TX and RX enabled only,8N1,idle=HIGH
    U1MODE = 0x0000; 
    SETBIT(U1STA,12);           //! Enable RX 
    SETBIT(U1STA,10);           //! Enable TX             
    U1BRG = (PBCLK/(16*UARTspeed)) - 1;
    //U1BRG = 4;               //! Baud Speed => 115200
    //!Interrupt example   
    //IPC9bits.U1RXIP = 3; IPC9bits.U1RXIS = 2; //!set IPL3 and sub-priority2
    //U1STAbits.URXISEL = 0;      //!where receive one character
    //IPC10bits.U1TXIP = 3; IPC10bits.U1TXIS = 2;//!set IPL3 and sub-priority2
    //U1STAbits.UTXISEL = 2;      //!where transmit is empty
    // For the future applications 
    IFS1bits.U1TXIF = 0;        //!< Clear the Transmit Interrupt Flag
    IEC1bits.U1TXIE = 0;        //!< Disble Transmit Interrupts
    IFS1bits.U1RXIF = 0;        //!< Clear the Recieve Interrupt Flag
    IEC1bits.U1RXIE = 0;        //!< Disble Recieve Interrupts
    U1MODEbits.ON = 1;          //!< U1ART ON
}
void initANALOG(void)
{
    //! all digital
    ANSELA = 0; ANSELB = 0; 
    //! No pull-up resistor => necessairy to define ditital Output 
    CNPUA = 0; CNPUB= 0; 
    //! No pull-down resistor => necessairy to define ditital Output
    CNPDA = 0; CNPDB= 0;
     //! No Change Notification (interruption generated by I/O )
    CNCONA = 0;  CNCONB = 0;
    /* Initialize pins as analog inputs */ 
    //SETBIT(TRISA,0);  SETBIT(ANSELA,0);     //! AN0pic to pin A0
    //SETBIT(TRISA,1);  SETBIT(ANSELA,1);     //! AN1pic to pin A1
    //! Initial discharge external capacitor
    CLRBIT(TRISA,0); CLRBIT(ANSELA,0);     //! A0 digital output
    CLRBIT(LATA,0);             //! discharge external capacitor   
    /* Configure ADCCMPCONx No analog comparators are used.*/
        CM1CON = CM2CON = CM3CON = 0;
    return;
}
/*********************************/
uint8 initADC(void)
{ 
    //! ***********************
    //! CTMU initialisation 
    //! ***********************
    CTMUCON = 0;
    CTMUCONbits.ON = 0;         //! Turn off CTMU
    CTMUCONbits.TGEN = 0;       //! Disables edge delay generation    
    CTMUCONbits.IDISSEN = 0;    //! End drain of circuit
    //CTMUCONbits.EDG2POL = 0;
    CTMUCONbits.EDG2SEL = 0x3;  //! Edge 1 Source Select bits, don't care 
    //CTMUCONbits.EDG1POL = 0;
    CTMUCONbits.EDG1SEL = 0x3;  //! Edge 2 Source Select bits, don't care  
    CTMUCONbits.IRNG = 3;       //! Current Range Select bits
    CTMUCONbits.ITRIM = 0;      //! Current Source Trim bits    
    //! ***********************
    //! ADC initialisation 
    //! ***********************
    AD1CON1bits.ADON = 0;       //! ensure the ADC is off before setting the configuration
    AD1CON1 = 0;
    AD1CON1bits.SIDL = 1;       //! Continue module operation when the device enters Idle mode
    AD1CON1bits.FORM = 4;       //! Integer 32-bit (DOUT = 0000 0000 0000 0000 0000 00dd dddd dddd)
    AD1CON1bits.SSRC = 0;       //! Clearing SAMP bit ends sampling and starts conversion
    AD1CON2 = 0;
    AD1CON2bits.VCFG  = 0;      //! Voltage Reference Configuration bits: AVdd and AVss
    AD1CON3 = 0;
    AD1CON3bits.SAMC = 20;      //! The acquisition time, Auto-Sample Time bits 31 Tad
    AD1CON3bits.ADCS = 4;       //! ADC Conversion Clock Select bits( 4xTpb =Tad
    AD1CHS = 0;
    AD1CHSbits.CH0SA = 0x0;     //! Positive Input A0 Select bits for Sample A
    AD1CHSbits.CH0SB = 0x0;     //! Positive Input A0 Select bits for Sample B
    AD1CSSL = 0x0000;           //! ADC INPUT SCAN SELECT REGISTER
    AD1CON1bits.ADON = 1;       //! ADC ON
    CTMUCONbits.ON = 1;         //! Turn on CTMU
    return (TRUE);
}
 