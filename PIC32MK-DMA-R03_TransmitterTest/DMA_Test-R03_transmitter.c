//!----------------------------------------------------------------------------
//! electronics-lis
//! CH-2000 Neuchatel
//! info@electronics-lis.com
//! https://electronics-lis.com
//! L. Lisowski 20 January 2018
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
// PIC32MK DMA tests
//!----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
//! check where is <p32mk1024mcf064.h> file on your PC
#include <C:\Program Files\Microchip\xc32\v2.50\pic32mx\include\proc\PIC32MK-MC\p32mk1024mcf064.h>
#include <xc.h>            
#include <sys/attribs.h> 
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
//! This part need to be adapted for your application. 
//! These Configuration Bits need to be adapted for your HW and SW application.
//! This is FRC version without Quartz/Oscillator
// DEVCFG3
#pragma config PWMLOCK = OFF, FUSBIDIO2 = OFF, FVBUSIO2 = OFF, PGL1WAY = OFF    
#pragma config PMDL1WAY = OFF, IOL1WAY = OFF, FUSBIDIO1 = OFF, FVBUSIO1 = OFF                    
// DEVCFG2
#pragma config FPLLIDIV = DIV_1, FPLLRNG = RANGE_5_10_MHZ, FPLLICLK = PLL_FRC         
#pragma config FPLLMULT = MUL_60, FPLLODIV = DIV_4, VBATBOREN = ON, DSBOREN = ON      
#pragma config DSWDTPS = DSPS32, DSWDTOSC = LPRC, DSWDTEN = OFF, FDSEN = OFF          
#pragma config BORSEL = HIGH, UPLLEN = OFF               
// DEVCFG1
#pragma config FNOSC = FRC, DMTINTV = WIN_127_128, FSOSCEN = OFF, IESO = ON
#pragma config POSCMOD = OFF, OSCIOFNC = ON, FCKSM = CSECME, WDTPS = PS16  
#pragma config WDTSPGM =STOP, WINDIS = NORMAL, FWDTEN = OFF, FWDTWINSZ = WINSZ_25
#pragma config DMTCNT = DMT8, FDMTEN = OFF
// DEVCFG0
#pragma config DEBUG = OFF, JTAGEN = OFF, ICESEL = ICS_PGx2, TRCEN = ON
#pragma config BOOTISA = MIPS32,FSLEEP = OFF, DBGPER = PG_ALL, SMCLR = MCLR_NORM     
#pragma config SOSCGAIN = GAIN_2X, SOSCBOOST = ON, POSCGAIN = GAIN_LEVEL_3
#pragma config POSCBOOST = ON, EJTAGBEN = NORMAL 
// DEVCP
#pragma config CP = OFF                 // Code Protect (Protection Disabled)
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
/************************************
// Macro Definitions
/****************************************/
//! Don't forget !!! the result are uint32_l !!!!!!!!
#define TSTBIT(D,i) (D>>i & 1)
#define SETBIT(D,i) (D |= ((uint32)1<<i))
#define CLRBIT(D,i) (D &= ~((uint32)1 << i))
#define SETPARA(D,p,i) (D |= (p<<i))
#define SYSCLK                  120000000L
#define PBCLK                   (SYSCLK/2)
#define UARTspeed               115200
#define TRUE                    1
#define FALSE                   0
/*********************************/
//! Global variables and data
/*********************************/
char i;
#define SYSCLK 120000000L
#define PBCLK (SYSCLK/2)
#define UARTspeed   115200
//! Extremely important. Only this form woeking well
#define VirtToPhys(p) (int)p<0?((int)p&0x1fffffffL):(unsigned int)((unsigned char*)p+0x40000000L) 
//! Buffer initialization
#define RX_BUF_SIZE  1
#define TX_BUF_SIZE 30
char rxBuf[RX_BUF_SIZE];
char txBuf[TX_BUF_SIZE] = "transfer1234 =>\r\n"; // test string
/*********************************/
//! Function prototype
/*********************************/
void initGen(void);
void initU1ART(void);
void initDMA1TX(void);
/*********************************/
void main()
{ 
    //! Clock initialization
    /* switch to PLL input (SPLL) */
    SYSKEY = 0x0;                        /* Ensure OSCCON is locked */
    SYSKEY = 0xAA996655;                 /* write Key1 unlock sequence */
    SYSKEY = 0x556699AA;                 /* write Key2 unlock sequence */
    OSCCONCLR = 0x07000600;
    OSCCONSET = 0x0100;
    OSCCONSET = 1;                       /* Start clock switching */
    SYSKEY = 0x0;                        /* OSCCON is re-locked */
    // Function initialization
    __builtin_enable_interrupts();  // global interrupt enable
    __XC_UART = 1;                  // printf on the U1ART    
    INTCONbits.MVEC = 1; // Set the interrupt controller for multi-vector mode
    PMCONbits.ON = 0;                   //! no PMP
    PMD5bits.U1MD = 0; //! PERIPHERAL MODULE DISABLE
    // Function initialization
    initGen();       //! PWM part initialization 
    initU1ART();  
    initDMA1TX();
    printf ("\n\r This is my first DMA transmit test \n\r");
    /*while (1) // U1ART test
	{
        char i;
        while( !U1STAbits.URXDA);   //! wait until data available in RX buffer
        i = U1RXREG;
        while( U1STAbits.UTXBF);    //! wait while TX buffer full
        U1TXREG = i;                //! Echo
        while( !U1STAbits.TRMT);    //! wait for last transmission to finish
	}*/
    while(1)
    { 
        if (U1STAbits.URXDA == 1)
        {
            if (U1RXREG == 's')
            {    
                DCH1CONbits.CHEN = 1;       // Enable channel 
                DCH1ECONbits.CFORCE = 1;    // set CFORCE to 1 to start Transmitter
                while(1)                    //verify if precedent transmit is finished 
                {                           //not very nice but is working 
                    if (DCH1CONbits.CHEN == 0) break; //CHEN=0 when transmit finishedaarr                
                }     
            }
        }
    }
}
/*********************************/
void initU1ART(void)
{
    // UART init
    // Pins initialization. !!!!! have to be adapted to hardware 
    TRISGbits.TRISG8= 1;       //RG8 digital input
    U1RXRbits.U1RXR = 10;        //SET RX to RG8
    RPG6Rbits.RPG6R = 1;        //SET RG6 to TX   
    // disable UART1 and autobaud, TX and RX enabled only,8N1,idle=HIGH
    U1MODE = 0x0000;         
    U1STAbits.URXEN = 1; U1STAbits.UTXEN = 1;    //!Enable RX and TX
    U1BRG = (PBCLK/(16*UARTspeed)) - 1;       
    //U1BRG = 0x40;                        // Baud rate: 115200 (+0.16%)
    //!Interrupt example   
    IPC9bits.U1RXIP = 3; IPC9bits.U1RXIS = 2; //!set IPL3 and sub-priority2
    U1STAbits.URXISEL = 0;      //!where receive one character

    IPC10bits.U1TXIP = 3; IPC10bits.U1TXIS = 2;//!set IPL3 and sub-priority2
    U1STAbits.UTXISEL = 2;      //!where transmit is empty
    // For the future applications 
    IFS1bits.U1TXIF = 0;        //!< Clear the Transmit Interrupt Flag
    IEC1bits.U1TXIE = 0;        //!< Disble Transmit Interrupts
    IFS1bits.U1RXIF = 0;        //!< Clear the Recieve Interrupt Flag
    IEC1bits.U1RXIE = 0;        //!< Disble Recieve Interrupts
    U1MODEbits.ON = 1;          //!< U1ART ON
}
/*********************************/
/*********************************/
void initGen(void)
{
    //! all digital
    ANSELA = 0; ANSELB = 0; ANSELC = 0;ANSELE = 0; ANSELG = 0; 
    //! No pull-up resistor => necessairy to define ditital Output 
    CNPUA = 0; CNPUB= 0; CNPUC= 0; CNPUD= 0; CNPUE= 0; CNPUF= 0; CNPUG= 0; 
    CNPUF= 0;
    //! No pull-down resistor => necessairy to define ditital Output
    CNPDA = 0; CNPDB= 0; CNPDC= 0; CNPDD= 0; CNPDE= 0; CNPDF= 0; 
    CNPDG= 0; CNPDF= 0;
     //! No Change Notification (interruption generated by I/O )
    CNCONA = 0;  CNCONB = 0; CNCONC = 0; CNCOND = 0; CNCONE = 0; CNCONF = 0;
    CNCONG = 0;    
    /* Configure ADCCMPCONx No analog comparators are used.*/
        CM1CON = CM2CON = CM3CON = CM4CON = CM5CON = 0;
    /* DAC disabled */
        DAC1CON = DAC2CON = DAC3CON = 0;
    //! Disable and enable peripheral modules
    PMD2 = 1; //! Ampli op and comparator without clock
    return;
}
/*********************************/
void initDMA1TX(void) //! This is the DMA initialization for the transmitter 
{
    DMACONbits.ON = 1;              // DMA enable
    IEC2bits.DMA1IE = 0;            // Interrupt disable
    IFS2bits.DMA1IF = 0;            // Flag Interrupt clear
    DCH1CONbits.CHPRI = 3;          // Interruption priority
    DCH1ECONbits.PATEN = 1;         // Pattern ON
    DCH1CONbits.CHPATLEN = 0;       // Two bytes of pattern
    DCH1DATbits.CHPDAT = '\n';       // Pattern definition to finish transfer
    //DCH1DATbits.CHPDAT = 0;
    DCH1ECONbits.CHSIRQ = 40;       // IRQ vector for "UART1 Transfer Done"
                                    // ===> U1TXREG empty and new Cell of 8 bytes
                                    // Could be transfered 
                                    // See table 8-3 p 123 in PIC32MK documentation
    DCH1ECONbits.SIRQEN = 1;        // DMA's interruption enable       
    DCH1SSA = VirtToPhys((void*)&txBuf[0]); // transmit buffer address
    DCH1DSA = VirtToPhys((void*)&U1TXREG);  // receiver buffer address
    DCH1SSIZ = TX_BUF_SIZE;     // transmit buffer size                    
    DCH1DSIZ = 1;               // receiver buffer size
    DCH1CSIZ = 8;               // PIC32MK has 8 bytes Transmitter FIFO                                              
    DCH1INT = 0;                // any interruption
    DCH1INTCLR=0x00ff00ff; // clear existing events, disable all interrupts
    DCH1INTSET=0x00090000; // enable Block Complete and error interrupts
    // Interrupt setup
    IPC18 = 0;
    IPC18bits.DMA1IP = 5;
    IPC18bits.DMA1IS = 2;
    IFS2bits.DMA1IF = 0;
    IEC2bits.DMA1IE = 1;        // Interrupt enable
    DCH1CONbits.CHEN = 1;       // Enable channel
}
/*************************************/
void __ISR(_DMA1_VECTOR, IPL5AUTO) DmaHandler1(void) //_DMA0_VECTOR = 72
{
    DCH1INTCLR=0x000000ff;      // clear DMA's channel interrupt flags
    IFS2bits.DMA1IF = 0;        // clear interruption flag         
}




