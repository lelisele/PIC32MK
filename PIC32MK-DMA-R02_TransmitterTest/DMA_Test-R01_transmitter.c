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
#include <C:\Program Files (x86)\Microchip\xc32\v1.44\pic32mx\include\proc\p32mk1024mcf064.h>
#include <xc.h>            
#include <sys/attribs.h> 
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
//! This part need to be adapted for your application. 
#pragma config PWMLOCK = OFF, FUSBIDIO2 = OFF, FVBUSIO2 = OFF, PGL1WAY = ON    
#pragma config PMDL1WAY = ON, IOL1WAY = ON, FUSBIDIO1 = OFF, FVBUSIO1 = OFF                    
// DEVCFG2
#pragma config FPLLIDIV = DIV_1, FPLLRNG = RANGE_5_10_MHZ, FPLLICLK = PLL_POSC         
#pragma config FPLLMULT = MUL_48, FPLLODIV = DIV_4, VBATBOREN = ON, DSBOREN = ON      
#pragma config DSWDTPS = DSPS32, DSWDTOSC = LPRC, DSWDTEN = OFF, FDSEN = OFF          
#pragma config BORSEL = HIGH, UPLLEN = OFF               
// DEVCFG1
#pragma config FNOSC = SPLL, DMTINTV = WIN_127_128, FSOSCEN = OFF, IESO = ON
#pragma config POSCMOD = HS, OSCIOFNC = ON, FCKSM = CSECME, WDTPS = PS1048576   
#pragma config WDTSPGM =STOP,WINDIS = NORMAL, FWDTEN = OFF, FWDTWINSZ = WINSZ_25
#pragma config DMTCNT = DMT31, FDMTEN = ON
// DEVCFG0
#pragma config DEBUG = OFF, JTAGEN = OFF, ICESEL = ICS_PGx1, TRCEN = ON
#pragma config BOOTISA = MIPS32,FSLEEP = OFF, DBGPER = PG_ALL, SMCLR = MCLR_NORM     
#pragma config SOSCGAIN = GAIN_2X, SOSCBOOST = ON, POSCGAIN = GAIN_LEVEL_3
#pragma config POSCBOOST = ON, EJTAGBEN = NORMAL 
// DEVCP
#pragma config CP = OFF                 // Code Protect (Protection Disabled)
//! That are my Configuration Bits version, you need adapt them for your application 
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
#define _SYSCLK 120000000L
#define _PBCLK (_SYSCLK/2)
#define _UARTspeed   115200
/*********************************/
//! Global variables and data
/*********************************/
/*********************************/
//! Extremely important. Only this form woeking well
#define VirtToPhys(p) (int)p<0?((int)p&0x1fffffffL):(unsigned int)((unsigned char*)p+0x40000000L) 
//! Buffer initialization
#define RX_BUF_SIZE  1
#define TX_BUF_SIZE 30
char rxBuf[RX_BUF_SIZE];
char txBuf[TX_BUF_SIZE] = "transfer1234xfgsfg\r\n......."; // test string
int rx_number;  
/*********************************/
//! Function prototype
/*********************************/
void initDMA_global(void);
void initDMA0RX(void);
void initDMA1TX(void);
void initU1ART(void);
/*********************************/
void main()
{ 
    unsigned long  k, i;
    __builtin_enable_interrupts();  // global interrupt enable
    __XC_UART = 1;                  // printf on the U1ART    
    INTCONbits.MVEC = 1; // Set the interrupt controller for multi-vector mode  
    // Function initialization
    initU1ART();
    initDMA_global();
    initDMA1TX();
    printf ("\n\r This is my first DMA transmit test \n\r");
    rx_number = 0;   
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
        if (U1STAbits.URXDA == 1)       // Hit any key only for test and verify 
                                        // displayed message (finished on \n !!! 
                                        // Receiver register not empty
        {
            if (U1RXREG == 's')
            {    
                DCH1CONbits.CHEN = 1;       // Enable channel 
                DCH1ECONbits.CFORCE = 1;// set CFORCE to 1 to start Transmitter
                while(1)            //verify if precedent transmit is finished 
                {                   //not very nice but is working 
                    if (DCH1CONbits.CHEN == 0) break; //CHEN=0 when transmit finished
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
    TRISCbits.TRISC6 = 1;       // C6 digital input
    U1RXRbits.U1RXR = 5;        //SET RX to RC6 
    RPC7Rbits.RPC7R = 1;        //SET RC7 to TX    
    // disable UART1 and autobaud, TX and RX enabled only,8N1,idle=HIGH
    U1MODE = 0x0000;         
    U1STAbits.URXEN = 1;        // Enable RX 
    U1STAbits.UTXEN = 1;        // Enable TX
    U1BRG = (_PBCLK/(16*_UARTspeed)) - 1;
    //U1BRG = 32;               //! Baud Speed => 115200
    // Interrupt         
    IPC9bits.U1RXIP = 3;        // set IPL 3
    IPC9bits.U1RXIS = 2;        // sub-priority 2
    U1STAbits.URXISEL = 3;      // where receive is full
    
    IPC10bits.U1TXIP = 3;       // set IPL 3
    IPC10bits.U1TXIS = 2;       // sub-priority 2
    U1STAbits.UTXISEL = 2;      // where transmit is empty
            
    IFS1bits.U1TXIF = 0;        //!< Clear the Transmit Interrupt Flag
    IEC1bits.U1TXIE = 0;        //!< Enable Transmit Interrupts
    IFS1bits.U1RXIF = 0;        //!< Clear the Recieve Interrupt Flag
    IEC1bits.U1RXIE = 0;        //!< Enable Recieve Interrupts
    U1MODEbits.ON = 1;          //!< U1ART ON
}
/*********************************/
void initDMA_global(void) /* Turns on the DMA controller in the PIC */
{
    DMACONbits.ON = 1;
    // Is it necessary ??????
    DCH1CONbits.CHEN = 0;     // Enable channel
    DCH2CONbits.CHEN = 0;     // Enable channel 
    DCH3CONbits.CHEN = 0;     // Enable channel
    DCH4CONbits.CHEN = 0;     // Enable channel
    DCH5CONbits.CHEN = 0;     // Enable channel
    DCH6CONbits.CHEN = 0;     // Enable channel
    DCH7CONbits.CHEN = 0;     // Enable channel
}
/*********************************/
void initDMA1TX(void) /* This is the DMA initialization for the transmitter */
{
    IEC2bits.DMA1IE = 0;
    IFS2bits.DMA1IF = 0; 
    DCH1CONbits.CHPRI = 3;
    DCH1ECONbits.PATEN = 1;         // Pattern ON
    DCH1CONbits.CHPATLEN = 0;       // Two bytes of pattern
    DCH1DATbits.CHPDAT = '\n';      // Pattern definition to finish transfer
    //DCH1DATbits.CHPDAT = 0;
    DCH1ECONbits.CHSIRQ = 40;       // IRQ vector for "UART1 Transfer Done"
                                    // ===> U1TXREG empty and new Cell of 8 bytes
                                    // Could be transfered 
                                // See table 8-3 p 123 in PIC32MK documentation
    DCH1ECONbits.SIRQEN = 1;    // DMA's interruption enable       
    DCH1SSA = VirtToPhys((void*)&txBuf[0]);
    DCH1DSA = VirtToPhys((void*)&U1TXREG);
    DCH1SSIZ = TX_BUF_SIZE;                        
    DCH1DSIZ = 1;
    DCH1CSIZ = 8;               // PIC32MK has 8 bytes Transmitter FIFO                                              
    DCH1INT = 0;                // any interruption
    DCH1INTCLR=0x00ff00ff; // clear existing events, disable all interrupts
    DCH1INTSET=0x00090000; // enable Block Complete and error interrupts
    // Interrupt setup
    IPC18 = 0;
    IPC18bits.DMA1IP = 5;
    IPC18bits.DMA1IS = 2;
    IFS2bits.DMA1IF = 0;
    IEC2bits.DMA1IE = 1;        // Interrupt disable
    DCH1CONbits.CHEN = 1;       // Enable channel
}
/*************************************/
void __ISR(_DMA1_VECTOR, IPL5AUTO) DmaHandler1(void) //_DMA0_VECTOR = 72
{
    DCH1INTCLR=0x000000ff;      // clear DMA's channel interrupt flags
    IFS2bits.DMA1IF = 0;        // clear interruption flag         
}