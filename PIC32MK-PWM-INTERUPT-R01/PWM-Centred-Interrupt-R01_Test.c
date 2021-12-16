//!----------------------------------------------------------------------------
//! electronics-lis
//! CH-2000 Neuchatel
//! info@electronics-lis.com
//! https://electronics-lis.com
//! L. Lisowski 20 January 2018
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
// PIC32MK ADC tests
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
// DEVCFG3
#pragma config PWMLOCK = OFF, FUSBIDIO2 = OFF, FVBUSIO2 = OFF, PGL1WAY = OFF    
#pragma config PMDL1WAY = OFF, IOL1WAY = OFF, FUSBIDIO1 = OFF, FVBUSIO1 = OFF                    
// DEVCFG2
#pragma config FPLLIDIV = DIV_1, FPLLRNG = RANGE_5_10_MHZ, FPLLICLK = PLL_POSC         
#pragma config FPLLMULT = MUL_48, FPLLODIV = DIV_4, VBATBOREN = ON, DSBOREN = ON      
#pragma config DSWDTPS = DSPS32, DSWDTOSC = LPRC, DSWDTEN = OFF, FDSEN = OFF          
#pragma config BORSEL = HIGH, UPLLEN = OFF               
// DEVCFG1
#pragma config FNOSC = SPLL, DMTINTV = WIN_127_128, FSOSCEN = OFF, IESO = ON
#pragma config POSCMOD = HS, OSCIOFNC = ON, FCKSM = CSECME, WDTPS = PS1048576   
#pragma config WDTSPGM =STOP,WINDIS = NORMAL, FWDTEN = OFF, FWDTWINSZ = WINSZ_25
#pragma config DMTCNT = DMT31, FDMTEN = OFF
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
uint16 count;
uint8   flag0_state;
/*********************************/
//! Function prototype
/*********************************/
void initU1ART(void);
void initPWMgen(void);
uint8 initPWMcentred(void);
uint8 VerifStatZERO( volatile uint32 * reg, uint32 BitPos);
uint8 VerifStatONE( volatile uint32 *reg, uint32 mask);
void SetPara(volatile uint32 * reg, uint32 para, uint32 ParaPos);
/*********************************/
//! main 
void main()
{  
    __builtin_enable_interrupts();  // global interrupt enable
    __XC_UART = 1;                  // printf on the U1ART    
    INTCONbits.MVEC = 1; // Set the interrupt controller for multi-vector mode  
    PMCONbits.ON = 0;               //! no PMP
    // Function initialization
    PMD5bits.U1MD = 0; //! PERIPHERAL MODULE DISABLE
    PMD4bits.PWM11MD = 0; //! PERIPHERAL MODULE DISABLE
    initU1ART();
    printf ("\n\r This is my first PWM and Interruption test \n\r"); 
    initPWMgen();       //! PWM part initialization
    if(initPWMcentred() == TRUE){
        printf ("\n\r initPWMcentred accomplished \n\r");
    } 
    else{
        printf ("\n\r initPWMcentred not accomplished \n\r"); 
    }               
    while (1) 
	{
        while( !U1STAbits.URXDA);   //! wait until data available in RX buffer
        i = U1RXREG;                //1 Hit any key
        while( U1STAbits.UTXBF);    //! wait while TX buffer full
        U1TXREG = i;                //! Echo
        while( !U1STAbits.TRMT);    //! wait for last transmission to finish
        if (i >= '1' && i<='9')     //! U1ART test
        {                    
            PDC1 = (i-'0')*PHASE1/10; //! i * 10 /100 % calculation
            printf("\n\r Duty=%u \tPhase1=%u  \tNewDuty (1-9)*10 % = ", PDC1, PHASE1);
        }
        else {
            printf("\n\r Argument has to be number");
        }   
	}
}
/*********************************/
void initU1ART(void)
{
    // UART init
    // Pins initialization. !!!!! have to be adapted to hardware        
    SETBIT(TRISC,6);            //!  C6 digital input   
    SetPara(&U1RXR,5,0);        //!  SET RX to RC6
    SETBIT(RPC7R,0);            //!  SET RC7 to TX
    // disable UART1 and autobaud, TX and RX enabled only,8N1,idle=HIGH
    U1MODE = 0x0000; 
    SETBIT(U1STA,12);           //! Enable RX 
    SETBIT(U1STA,10);           //! Enable TX             
    U1BRG = (PBCLK/(16*UARTspeed)) - 1;
    //U1BRG = 32;               //! Baud Speed => 115200
    // Interrupt         
    SetPara(&IPC9,3,26);   
    SetPara(&IPC9,3,24);          
    SetPara(&U1STA,0,6);
    //! For future applications    
    SetPara(&IPC10,3,2);   
    SetPara(&IPC10,2,0);  
    SetPara(&U1STA,0,14);   
    CLRBIT(IFS1,8);    
    CLRBIT(IEC1,8);   
    CLRBIT(IFS1,7);  
    CLRBIT(IEC1,7);
    SETBIT(U1MODE,15);
}
void initPWMgen(void)
{
    //! all digital
    ANSELA = 0; ANSELB = 0; ANSELC = 0; ANSELD = 0; ANSELE = 0; ANSELF = 0;
    ANSELG = 0; ANSELF = 0; 
    //! No pull-up resistor => necessairy to define ditital Output 
    CNPUA = 0; CNPUB= 0; CNPUC= 0; CNPUD= 0; CNPUE= 0; CNPUF= 0; CNPUG= 0; 
    CNPUF= 0;
    //! No pull-down resistor => necessairy to define ditital Output
    CNPDA = 0; CNPDB= 0; CNPDC= 0; CNPDD= 0; CNPDE= 0; CNPDF= 0; 
    CNPDG= 0; CNPDF= 0;
     //! No Change Notification (interruption generated by I/O )
    CNCONA = 0;  CNCONB = 0; CNCONC = 0; CNCOND = 0; CNCONE = 0; CNCONF = 0;
    CNCONG = 0;   
    /* Initialize pins as PWM outputs */ 
    //CLRBIT(TRISB,14);  //! PIn B14 as PWMlH  
    //CLRBIT(TRISB,15);  //! PIn B15 as PWMlL
    /* Configure ADCCMPCONx No analog comparators are used.*/
        CM1CON = CM2CON = CM3CON = CM4CON = CM5CON = 0;
    /* DAC disabled */
        DAC1CON = DAC2CON = DAC3CON = 0;
    //! Disable and enable peripheral modules
    PMD2 = 1; //! Ampli op and comparator without clock
    return;
}
uint8 initPWMcentred(void)
{       
    SETBIT(PB6DIV,15);          //! Output clock is enabled
    SetPara(&PB6DIV,1,0);       //! Peripheral Bus ?6? Clock Divisor = /2
    //! PTCON initialization (Primary timer)
    PTCON = 0;                  //! Initial disable
    SetPara(&PTCON,0,2);        //! PCLKDIV = 1 = Sysclock /2
    //! CHOP initialization (Chop timer)
    CHOP =0;
    //! PWMCON1 initialization
    PWMCON1 = 0;                //! Initial settings
    CLRBIT(PWMCON1,3);          //! MTBS = 0 Primary master is the clock source for the MCPWM module
    SetPara(&PWMCON1,1,10);     //! ECAM=1= Symmetric Center-Aligned mode
    SetPara(&PWMCON1,3,6);      //! Dead Time Compensation mode enabled
    SETBIT(PWMCON1,9);          //! ITB=1 PHASE1 registers provide time base 
    //! IOCON1 initialization
    IOCON1 = 0;                 //! Initial settings
    SETBIT(IOCON1,15);          //! PENH=1 PWM module controls PWM1H pin
    SETBIT(IOCON1,14);          //! PENL=1 PWM module controls PWM1L pin
    CLRBIT(IOCON1,13);          //! PWMxH pin is active-high
    CLRBIT(IOCON1,12);          //! PWMxL pin is active-high
    SetPara(&IOCON1,0,10);      //! PMOD=0>PWM I/O pin pair is in Complementary mode
    PHASE1 = 2000;              //! phase1 PHASEx registers provide time base 
    PDC1 = 1000;                //! Duty cycles
    DTR1 = 20;                  //! Dead Time Values
    ALTDTR1 = 20;               //!
    //! Trigger implementation
    SetPara(&TRIG1,0,0);        //! 0 delay
    TRGCON1 = 0;                //! Initial value DTM=0;
    SetPara(&TRGCON1,0,12);     //! TRGDIV1=0,; Trigger1 Divider = 4
    SetPara(&TRGCON1,2,10);     //! TRGSEL=2; Trigger Cycle Selection for Center-Aligned
    //! PWM1 trigger event Interrupt implementation
    //! _PWM1_VECTOR
    SetPara(&IPC43,1,10);       //! Priority
    SetPara(&IPC43,5,8);        //! sub-Priority
    SETBIT(IEC5,13);            //! Interrupt Enable
    CLRBIT(IFS5,13);            //! Clear flag          
    CLRBIT(TRGCON1,7);          //! DTM=0 Secondary trigger event is not combined 
    CLRBIT(TRGCON1,6);          //! STRGIS=0 Interrupt selected 
    SETBIT(PWMCON1,21);         //! TRIGIEN=1 Primary Trigger Interrupt Enable
    count = 0;
    flag0_state = 0;
    //! Digital Flag F0 and F1 initialization
    SETBIT(PB4DIV,15);          //! Output clock is enabled
    SetPara(&PB4DIV,1,0);       //! Peripheral Bus ?6? Clock Divisor = /2
    PB4DIVbits.ON = 1;          //! PBCLK4 clock  => ON => Digital IO
    IOCON11 = 0;
    PMD4SET = 0xFFF00000;       //! Disable channels PWM5 to PWM12 !!!!!!!!!
    //IOCON11bits.PENH = 0;     // Pin I/O
    //IOCON11bits.PENL = 0;     // Pin I/O
    CLRBIT(ANSELF,0); CLRBIT(TRISF,0); //! PWM pins as Digital Output
    CLRBIT(ANSELF,1); CLRBIT(TRISF,1);
    SETBIT(LATF,0);   CLRBIT(LATF,1);   //! Initial    
    SETBIT(PTCON,15);           //! PWM module enable
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
//! ----------------------------------------------------------------------------
//! Brief of the exported inline code definition
//! Result 	
//! Notes: 
//! First line of the function description \n
//! Second line of the function description
//! \param[in]  Tutu : description of the variable
//! \return     Description of the return value
//! ----------------------------------------------------------------------------
void __ISR(_PWM1_VECTOR, IPL1SOFT) Pwm1Handler(void) //_PWM1_VECTOR = 173
{    
    /*count++;
    if (count == 2000)          //! Divider to test ISR
    {
        LATFINV = 0x3; 
        //putchar(':');        
    }*/
    LATFINV = 0x3; 
    CLRBIT(PWMCON1,29);         //! Trigger Interrupt Status
    CLRBIT(IFS5,13);            //! clear flag
}
 
uint32 excep_code;
uint32 excep_addr;

void _general_exception_handler (unsigned cause, unsigned status)
{
    excep_code = (cause & 0x0000007C) >> 2;
    excep_addr = __builtin_mfc0(_CP0_EPC, _CP0_EPC_SELECT);
    if ((cause & 0x80000000) != 0)
        excep_addr += 4;
    putchar('!');
    while (1);
}