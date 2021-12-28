//!----------------------------------------------------------------------------
//! electronics-lis
//! CH-2000 Neuchatel
//! info@electronics-lis.com
//! https://electronics-lis.com
//! L. Lisowski December 2021
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
//! PIC32MK ADC tests  REVISION 03
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
//#define SETPARA(D,p,i) (D |= (p<<i))
#define REG_SHIFT               4
#define ADC_MAX_LOOPS           100
#define SYSCLK                  120000000L
#define PBCLK                   (SYSCLK/2)
#define UARTspeed               115200
#define TRUE                    1
#define FALSE                   0
/*********************************/
//! Local Variables
/*********************************/
char i, k;
uint32  reg ,pos, New;
volatile uint32 *Val;
uint8   MaxLoops,NbrLoops;
uint8   StrNbrLoops[10];
static uint32  result[10];
/*********************************/
//! Function prototype
/*********************************/
void initU1ART(void);
void initANALOG(void);
uint8 initADC(void);
uint8 VerifStatZERO( volatile uint32 * reg, uint32 BitPos);
uint8 VerifStatONE( volatile uint32 *reg, uint32 mask);
void SetPara(volatile uint32 * reg, uint32 para, uint32 ParaPos);
/*********************************/
//! main 
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
    SYSKEY = 0x0;                        /* OSCCON is relocked */
    // Function initialization
    __builtin_enable_interrupts();  // global interrupt enable
    __XC_UART = 1;                  // printf on the U1ART    
    INTCONbits.MVEC = 1; // Set the interrupt controller for multi-vector mode  
    PMCONbits.ON = 0;               //! no PMP  
    // Function initialization
    PMD5bits.U1MD = 0; //! Enable clock
    initU1ART();
    printf ("\n\r This is my first ADC test \n\r"); 
    initANALOG();       //! Analog part initialization
    if(initADC() == TRUE){
        printf ("\n\r initADC accomplished \n\r");
    } 
    else{
        printf ("\n\r initADC not accomplished \n\r"); 
    }          
    //! Clock initialization
    SETBIT(PB2DIV,15);      //! PBCLK2 clock  => ON
    SETBIT(PB5DIV,15);       //! PBCLK5 clock  => ON (ADC module)
    SetPara(&PB5DIV,0,0);    //! PBDIV=0 clock  => ON DIV = 1        
    while (1) 
	{
        while( !U1STAbits.URXDA);   //! wait until data available in RX buffer
        i = U1RXREG;                //1 Hit any key
        while( U1STAbits.UTXBF);    //! wait while TX buffer full
        U1TXREG = i;                //! Echo
        while( !U1STAbits.TRMT);    //! wait for last transmission to finish
        if (i == 'c')       //! U1ART test
        {
            /* Trigger a conversion */
            SetPara(&DAC2CON,500,16);    //! DAC value to test one ADC
            SETBIT(ADCCON3,6);    //! GSWTRG =>Global Software Trigger bit
            /* Without filtering */
            /*for (k=0; k<=5; k++){
                VerifStatONE( &ADCDSTAT1, k);   //!ARDYx data is ready
                Val = &ADCDATA0 + k*4;
                result[k] = *Val;                 //! Read result 
            }                               
            printf("\n\r \tAD0=%lu \tAD1=%lu \tAD2=%lu \tAD3=%lu \tAD4=%lu \tAD5=%lu ",
                    result[0],result[1], result[2],result[3],result[4],result[5] );*/      
            /* With filtering */
            for (k=0; k<=3; k++){               //! Only four modules
                Val = &ADCFLTR1 + k*4;          //! Address calculation
                VerifStatONE(Val, 24);          //!ARDYx data is ready             
                result[k] = (*Val) & 0xFFFF;    //! Read result 
            }               
            printf("\n\r Filter=> \tAD0=%lu \tAD1=%lu \tAD2=%lu \tAD3=%lu   ",
                    result[0],result[1], result[2],result[3]);
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
    SETBIT(U1STA,12);           //! Enable RX 
    SETBIT(U1STA,10);           //! Enable TX             
    U1BRG = (PBCLK/(16*UARTspeed)) - 1;
    //U1BRG = 30;               //! Baud Speed => 115200
    U1MODE = 0x0000; 
    SETBIT(U1STA,12);           //! Enable RX 
    SETBIT(U1STA,10);           //! Enable TX             
    U1BRG = (PBCLK/(16*UARTspeed)) - 1;
    //U1BRG = 32;               //! Baud Speed => 115200
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
void initANALOG(void)
{
    //! all digital
    ANSELA = 0; ANSELB = 0; ANSELC = 0; ANSELE = 0; ANSELG = 0; 
    //! No pull-up resistor => necessairy to define ditital Output 
    CNPUA = 0; CNPUB= 0; CNPUC= 0; CNPUD= 0; CNPUE= 0; CNPUF= 0; CNPUG= 0; 
    CNPUF= 0;
    //! No pull-down resistor => necessairy to define ditital Output
    CNPDA = 0; CNPDB= 0; CNPDC= 0; CNPDD= 0; CNPDE= 0; CNPDF= 0; 
    CNPDG= 0; CNPDF= 0;
     //! No Change Notification (interruption generated by I/O )
    CNCONA = 0;  CNCONB = 0; CNCONC = 0; CNCOND = 0; CNCONE = 0; CNCONF = 0;
    CNCONG = 0;   
    /* Initialize pins as analog inputs */ 
    SETBIT(TRISA,0);  SETBIT(ANSELA,0);     //! AN0pic to pin A0
    SETBIT(TRISA,1);  SETBIT(ANSELA,1);     //! AN1pic to pin A1 
    SETBIT(TRISB,0);  SETBIT(ANSELB,0);     //! AN2pic to pin A2
    SETBIT(TRISB,1);  SETBIT(ANSELB,1);     //! AN3pic to pin A3
    SETBIT(TRISB,2);  SETBIT(ANSELB,2);     //! AN4pic to pin A4
    SETBIT(TRISB,3);  SETBIT(ANSELB,3);     //! AN5pic to pin A5
    //! Test IMCU09
    SETBIT(TRISC,1);  SETBIT(ANSELC,1);     //! AN7pic to pin A7
    CLRBIT(TRISC,10);  SETBIT(ANSELC,10);    //! DAC2 output   
    /* Configure ADCCMPCONx No analog comparators are used.*/
        CM1CON = CM2CON = CM3CON = CM4CON = CM5CON = 0;
    /* DAC disabled */
        DAC1CON = DAC2CON = DAC3CON = 0;
    //! Disable and enable peripheral modules
    PMD2 = 1; //! Ampli op and comparator without clock
    return;
}
uint8 initADC(void)
{       
        //! Disable all analog inputs
        for (k=0; k<=7; k++) CLRBIT(ADCANCON,k); 
        //! Configuration Register 
        ADC0CFG = DEVADC0; ADC1CFG = DEVADC1; ADC2CFG = DEVADC2; ADC3CFG = DEVADC3;
        ADC4CFG = DEVADC4; ADC5CFG = DEVADC5; //ADC6CFG = DEVADC6; 
        ADC7CFG = DEVADC7;        
        /* Turn the ADC off and reset all bits*/
        ADCCON1 = 0;
        /* Configure ADCCON1 */
        ADCCON1 = 0; // No ADCCON1 features are enabled including: Stop-in-Idle, turbo,
                    // CVD mode, Fractional mode and scan trigger source.
        /* Configure ADCCON2 */
        ADCCON2 = 0; // Since, we are using only the Class 1 inputs, no setting is
                    // required for ADCDIV    
        /* Digital Disable Bit*/
        for (k=0; k<=7; k++) CLRBIT(ADCCON3,k+16); //! ADCx=0 =>  Digital Disable
         /* Global clock setting */
        ADCCON3 = 0;
        SetPara(&ADCCON3,0,30); //! ADCSEL=0 => Select input clock source = PBCLK5
        SetPara(&ADCCON3,2,24); //! CONCLKDIV=2 => Control clock frequency is half of input clock
        SetPara(&ADCCON3,0,13); //! VREFSEL=0 => elect AVDD and AVSS as reference source
        SetPara(&ADCCON1,0,23); //! FRACT=0 => Integer format output
        /* Select ADC0-5 sample time, conversion clock and resolution */ 
        for (k=0; k<=5; k++){
            Val = (&ADC0TIME + k*REG_SHIFT);
            SetPara(Val,3,24);  //! SELRES=3 => resolution is 12 bits
            SetPara(Val,2,16);  //! ADCDIV=2 => ADC channel clock frequency
            SetPara(Val,2,0);   //! SAMC=5 => ADC0 sampling time = 5 * TAD0
            pos = 16 + k; 
            CLRBIT(ADCTRGMODE,pos); //! SHxALT => Select analog input for ADC modules
            pos = 0 + k; 
            //SETBIT(ADCTRGMODE,pos); //! Presynchronized Triggers ???????
            CLRBIT(ADCTRGSNS,pos); //! LVLx => Positive Edge trigger
            pos = 0 + 2*k; 
            CLRBIT(ADCTRGMODE,pos); //! SIGNx => Unsigned data format
            pos = 1 + 2*k; //! DIFF0  to  DIFF5 bit position
            CLRBIT(ADCIMCON1,pos); //! DIFFx => Single ended mode            
        }      
        /* Select ADC6 sample time, conversion clock and resolution */
        SetPara(&ADCCON2,2,16); SetPara(&ADCCON2,5,0); SetPara(&ADCCON2,3,24);
           
        for (k=0; k<=3; k++){       //! Trigger edge from software
            pos = 0 + 8*k; //! TRGSRC0  to  TRGSRC3 bit position => ADCTRG1
            SetPara(&ADCTRG1,1,pos); //! TRGSRCx => Positive Edge trigger
        }
        for (k=4; k<=5; k++){       //! Trigger edge from software
            pos = 0 + 8*k; //! TRGSRC4  to  TRGSRC5 bit position => ADCTRG1
            SetPara(&ADCTRG2,1,pos); //! Positive Edge trigger
        }
        /* Configure ADCGIRQENx = No interrupts are used */
        ADCGIRQEN1 = 0; ADCGIRQEN2 = 0;  
        /* Configure ADCCSSx No scanning is used */
        ADCCSS1 = 0; ADCCSS2 = 0; 
        //! DAC2 connected to one of ADC for test DAC and ADC
        SETBIT(DAC2CON,15);      //! ON=1 => CDAC Enable bit
        SETBIT(DAC2CON,8);       //! DACOE=1 => CDAC Output Buffer Enable bit        
        SetPara(&DAC2CON,3,0);    //! REFSEL=3=> Reference Source Select = AVdd      
        /* Configure ADCCMPCONx No digital comparators are used.*/
        ADCCMPCON1 = 0; 
        ADCCMPCON2 = 0; // register to '0' ensures that the comparator is disabled.
        ADCCMPCON3 = 0; // Other registers are ?don't care?.
        ADCCMPCON4 = 0;         
        /* Early interrupt = No early interrupt */
        ADCEIEN1 = 0; ADCEIEN2 = 0;
        /* Initial reset*/
        ADCANCON = 0; 
        SetPara(&ADCANCON,5,24); //! WKUPCLKCNT=5 => Wakeup exponent = 32 * TADx 
        /* For filtering and oversampling you need comment or not requested part*/
        ADCFLTR1 = 0; ADCFLTR2 = 0; ADCFLTR3 = 0; ADCFLTR4 = 0; //! Initial  
        //! Filtering sequence
        for(k=0; k<=3; k++ ){
            Val = &ADCFLTR1 + k*REG_SHIFT;               
            SetPara(Val,1,29);  //! DFMODE=1 => Filter ?x? works in Averaging mode
            SetPara(Val,3,26);  //! OVRSAM=1 =>  Oversampling Filter Ratio=16           
            SetPara(Val,k,16);  //! CHLIDx=k Digital Filter Analog Input Selection
                                //! ADC0 to Filter 1, ADC1 to 2 etc.
            SetPara(Val,1,31);  //! AFEN=1 => Filter enable
        }
        /* Individual conversion example*/
        /*SETBIT(ADCCON3,8);      //! RQCNVRT=1 => Individual ADC Input Conversion
        SETPARA(ADCCON3,0,0);       //! ADINSEL=0 => ADC0 as input 
        //ADCCON3bits.ADINSEL = 4;  //! AN0*/
         SETBIT(ADCCON1,15);     //! ON=1 => ADC module enabled
        /* Wait for voltage reference to be stable */
        if(VerifStatONE(&ADCCON2,31)==TRUE){} else return(FALSE);//! BGVRRDY=> Reference Voltage Status
        if(VerifStatZERO(&ADCCON2,30)==TRUE){} else return(FALSE);//! REFFLT=> Reference OK
        /* Enable clock to analog circuit */
        for(k=0; k<=5; k++) {
            SETBIT(ADCANCON,k);     //!ANENx=1 =>  Enable the clock to analog bias          
            pos = k + 8; 
            if(VerifStatONE(&ADCANCON,pos)){} else return(FALSE); //! WKRDYx => Wake-up Status     
            pos = k + 16;        
            SETBIT(ADCCON3,pos);            //! ADCx =>Digital Enable bit 
        }
        SETBIT(ADCANCON,7);     //! ANEN7=1 Shared ADC7 (ADC7) Analog Enable
        //VerifStatONE( &ADCANCON, 15);      //! WKRDY7 => Wake-up Status
        SETBIT(ADCCON3,23);     //! ADC7=>Digital Enable bit
        return (TRUE);
 }
/*********************************/
uint8 VerifStatZERO(volatile uint32 * reg, uint32 BitPos)
{     
    MaxLoops = ADC_MAX_LOOPS;
    NbrLoops = 0;               //! To test if loops is well implemented
    while (MaxLoops--){
        NbrLoops++;
        if(TSTBIT(*reg,BitPos) == 0){
            //printf("\n\r ==Zero NbrLoops=%u \tRegAddr=x%x",NbrLoops,reg);
            return (TRUE);
        }
    }
    printf("\n\r Error==0 BitPos=x%x  RegAddr=x%x", BitPos,reg);
    return (FALSE);
}
uint8 VerifStatONE(volatile uint32 * reg, uint32 BitPos)
{     
    MaxLoops = ADC_MAX_LOOPS;
    NbrLoops = 0;               //! To test if loops is well implemented
    while (MaxLoops--){
        NbrLoops++;
        if(TSTBIT(*reg,BitPos) == 1){
            //printf("\n\r ==ONE \tNbrLoops=%u RegAddr=x%x",NbrLoops,reg);
            return (TRUE);
        }
    }
    printf("\n\r Error==1 \tBitPos=%u  \tRegAddr=x%x", BitPos,reg);
    return (FALSE);
}
void SetPara(volatile uint32 * reg, uint32 para, uint32 ParaPos)
{     
    *reg |= (para<<ParaPos);
    return;
}  