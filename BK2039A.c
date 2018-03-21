/*****************************************************
Project : BK2039, BK2039A
Version : 1.05
Date    : 2017-12-14
Author  : yjjang
Company : BaKo Co., Ltd.
Comments:
BX6244R10 AVR ATmega128 Firmware Program

Chip type           : ATmega128
Program type        : Application
Clock frequency     : 16.000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024
*****************************************************/

/*////////////////////////////////////////////////////
Specification

BK2039, 10 Ch.
BK2039X, 5 / 5 Ch.
------------------------------------------------------
Output Voltage (Max.)
Haptic, On / Off Mode   = 10 Vrms / 0.5 Ap-p
Continue Mode           = 3  Vrms

Ampere Range            = 10 ~ 300 mArms
------------------------------------------------------

BK2039A, 20 Ch.
------------------------------------------------------
Output Voltage (Max.)
Haptic, On / Off Mode   = 12 Vrms / 1.5 Ap-p
Continue Mode           = 3  Vrms

Ampere Range            = 10 ~ 300 mArms
------------------------------------------------------
*/

#include <mega128.h>
#include <stdio.h>
#include <stdlib.h>
#include <delay.h>
#include <math.h>
#include <string.h>

#include "BX2039.h"

///////////////////////////////////////////
//char  DEVICE_ID[8]  = "BK2039A";
//char  DEVICE_VER[8] = "Ver1.00";

// Firmware Version Check /////////////////
#define PROGRAM_VERSION     (1.05)
#define PROGRAM_YEAR        (2018)
#define PROGRAM_MONTH       (3)
#define PROGRAM_DAY         (5)

/////////////////////////////////////////////////////////////////////////////
// UART
/*
#define UART_PACKET_LENGTH  17
#define UART_PACKET_DATA_LENGTH  8
typedef struct _UART_PACKET_
{
    char Str;
    char head;
    char index[3];
    char parameter[2];
    char data[UART_PACKET_DATA_LENGTH];
    char Ter;

} UART_PACKET, *PUART_PACKET;
*/

//int UART_ReceiveProcess(char Str);
//void UART_MessageProcess(PUART_PACKET pMsg);
//int UART_ReceiveData(void);

///////////////////////////////////////////////////////////////////
// Generator
unsigned long   DAC_SamplingFreq;
float           Angle_Step;
unsigned long   Angle_Max;

typedef struct _GEN_REAL_SIGNAL_
{
    int   SignalType;
    float Frequency;
    float Amplitude;
    int   AmpType;
} GEN_REAL_SIGNAL, *PGEN_REAL_SIGNAL;

GEN_REAL_SIGNAL GenReal[GEN_CH_NUM];

// Index Table Methode
typedef struct _SIGNAL_WAVE_
{
    // Generator Function
    int   Func;
    int   SignalType;
    // FixedSine Parameter
    float Sine_Freq;

    // Sweep Parameter
    int   Sweep_Mode;
    float Sweep_F1;
    float Sweep_F2;
    int   Sweep_nFreq;
    float Sweep_Ampl;

    //Generator Amplitude
    float Amplitude;

    // CV, CC Amp Type
    int   AmpType;

    float Time;
} SIGNAL_WAVE, *PSIGNAL_WAVE;

typedef struct _GENERATOR_SIGNAL_MULTI_
{
    SIGNAL_WAVE 	Wave[SIG_WAVE_NUM];
    float 			TimeResolution;
    unsigned long 	Index;
    unsigned long 	Length;
    unsigned long 	Count; 
} GENERATOR_SIGNAL_MULTI, *PGENERATOR_SIGNAL_MULTI;

typedef struct _CHANNEL_GENERATOR_
{
    GENERATOR_SIGNAL_MULTI SignalM[GEN_SIGNAL_NUM];
} CHANNEL_GENERATOR, *PCHANNEL_GENERATOR;

#if defined BK2039 || defined BK2039A || defined BK2039_5CH
CHANNEL_GENERATOR Gen;

PGENERATOR_SIGNAL_MULTI	GetGenSignalM(int ch, int sig)
{			
    return((PGENERATOR_SIGNAL_MULTI)&Gen.SignalM[sig]);
}
  
PSIGNAL_WAVE GetGenSignalWave(int ch, int sig, int wav)
{
    return((PSIGNAL_WAVE)&Gen.SignalM[sig].Wave[wav]);
}
#elif BK2039X
CHANNEL_GENERATOR Gen[GEN_CH_NUM];

PGENERATOR_SIGNAL_MULTI	GetGenSignalM(int ch, int sig)
{			
    return((PGENERATOR_SIGNAL_MULTI)&Gen[ch].SignalM[sig]);
}
  
PSIGNAL_WAVE GetGenSignalWave(int ch, int sig, int wav)
{
    return((PSIGNAL_WAVE)&Gen[ch].SignalM[sig].Wave[wav]);
}
#endif

char VSTR1[22];
char VSTR2[22];

void BuzzerOn(int time_ms);
void BuzzerOnOff(int on_ms, int off_ms);
void BuzzerNumber(int on_ms, int off_ms, int num);
void DisplayProcess(void);

///////////////////////////////////////////////////////////////////
#define ADC_INPUT_SIG 0
#define ADC_INPUT_CUR 2
#define ADC_VREF_AREF 0x00
#define ADC_VREF_AVCC 0x40

#define REC_LENGTH      0xFF
#define REC_LENGTH_CAL  450
#define REC_LENGTH_MAX  REC_LENGTH_CAL

unsigned int TimeData[REC_LENGTH_MAX];
unsigned int RecordLength = REC_LENGTH;
unsigned int RecordCount = 0;
unsigned int OnRecord;
unsigned int OnRecordEnd;
unsigned int ADC_MUX;

#define CUR_TEST_CHANNEL    20
/*
#if     defined BK2039A || BK2039X
#define CUR_TEST_CHANNEL    20
#elif   BK2039
#define CUR_TEST_CHANNEL    10
#elif   BK2039_5CH
#define CUR_TEST_CHANNEL    5
#endif
*/

// AMUX Channel
// AMUX1
#define AMUXCH1      		0
#define AMUXCH2      		1
#define AMUXCH3      		2
#define AMUXCH4      		3
#define AMUXCH5      		4

// AMUX2
#define AMUXCH6      		0
#define AMUXCH7      		1
#define AMUXCH8      		2
#define AMUXCH9      		3
#define AMUXCH10     		4

// AMUX3
#define AMUXCH11     		0
#define AMUXCH12     		1
#define AMUXCH13     		2
#define AMUXCH14     		3
#define AMUXCH15     		4

// AMUX4
#define AMUXCH16     		0
#define AMUXCH17     		1
#define AMUXCH18     		2
#define AMUXCH19     		3
#define AMUXCH20     		4

// ADC Channel
// ADC0
#define ADCCH1      		0
#define ADCCH2      		0
#define ADCCH3      		0
#define ADCCH4      		0
#define ADCCH5      		0

//ADC2
#define ADCCH6      		2
#define ADCCH7      		2
#define ADCCH8      		2
#define ADCCH9      		2
#define ADCCH10     		2

// ADC4
#define ADCCH11     		4
#define ADCCH12     		4
#define ADCCH13     		4
#define ADCCH14     		4
#define ADCCH15     		4

// ADC6
#define ADCCH16     		6
#define ADCCH17     		6
#define ADCCH18     		6
#define ADCCH19     		6
#define ADCCH20     		6

char AMUXCH[CUR_TEST_CHANNEL] = {	AMUXCH1,AMUXCH2,AMUXCH3,AMUXCH4,AMUXCH5,
                                    AMUXCH6,AMUXCH7,AMUXCH8,AMUXCH9,AMUXCH10,
                                    AMUXCH11,AMUXCH12,AMUXCH13,AMUXCH14,AMUXCH15,
                                    AMUXCH16,AMUXCH17,AMUXCH18,AMUXCH19,AMUXCH20
                                };

char ADCCH[CUR_TEST_CHANNEL] = {	ADCCH1,ADCCH2,ADCCH3,ADCCH4,ADCCH5,
                                    ADCCH6,ADCCH7,ADCCH8,ADCCH9,ADCCH10,
                                    ADCCH11,ADCCH12,ADCCH13,ADCCH14,ADCCH15,
                                    ADCCH16,ADCCH17,ADCCH18,ADCCH19,ADCCH20
                               };

#define DAQ_BOOTING     (0x01)
#define DAQ_BOOTOK      (0x02)
#define DAQ_GENOK       (0x04)
#define DAQ_LANOK       (0x08)

unsigned char DAQStatus = 0x0000;

void SetDAQStatus(unsigned char value)
{
    DAQStatus |= value;
}

void ClrDAQStatus(unsigned char value)
{
    DAQStatus &= ~value;
}

unsigned char ChkDAQStatus(unsigned char value)
{
    return((unsigned char)(DAQStatus&value));
}

// Debug Flag
#define DBG_NONE        0x0000
#define DBG_TEST_CH1    0x0001
#define DBG_TEST_CH2    0x0002

unsigned int DebugStatus = 0x0000;

///////////////////////////////////////////////////////////////////////////////
// Data Down Load Mode
unsigned char UART_RX_MODE;

#define UART_RX_COMMAND     0x00
#define UART_RX_IDATA       0x01

unsigned int RxByteofData;
unsigned int RxNumberofData;

unsigned int RxDataCount;

int OpMode;
int OpModeSub;
int DisplayPage;

int OnTestProcess;
int OnDisplayMain = 1;
int ProgramDisplayMode = 2;

int OnMotorPower = 0;
int CurTestCh = 0;

// VFD: 0x17 => ENT
// LCD: 0x17 => ENT => 미지원
#define VFD                 1
#define LCD                 2
#define DISPLAY             VFD

#define LCD_CUR_BLK_ON      0x0F    // Cursor ON  Blink ON
#define LCD_CUR_ON          0x0E    // Cursor ON  Blink OFF
#define LCD_CUR_OFF         0x0C    // CurSor OFF

#define CUR_MODE_ITEM       LCD_CUR_OFF
#define CUR_MODE_CURSOR     LCD_CUR_BLK_ON
#define CUR_MODE_VALUE      LCD_CUR_ON

//void DisplayTestContinue(void);
//////////////////////////////////////////////////////////////////////////////
// System
//char SID[10] = "BK2120A";
//eeprom char SYS_ID[10];

eeprom float SystemInitialRegister;

eeprom struct _SYSTEM_CAL_
{
    float Gen_Sens_Volt[GEN_CH_NUM];
    float Cur_Offset[CUR_TEST_CHANNEL];
    float Cur_Sens[CUR_TEST_CHANNEL];
} Cal;

#define SEQ_ON_OFF_1        (0)
#define SEQ_ON_OFF_2        (1)
#define SEQ_ON_OFF_3        (2)
#define SEQ_ON_OFF_4        (3)
#define SEQ_ON_OFF_NUMBER   (4)

#define MAX_TEST_NUMBER			    (9999999)
#define MAX_TEST_NUMBER_FAST	    (0xFFFF)
#define MAX_SIGNAL_REPEAT_NUMBER    (0xFFFF)

eeprom struct _TEST_CONTINUE_
{
    // Generator Setup
    int     Gen_Func;
	
    int     Gen_Sweep_Mode;
	
    float   Gen_Sweep_F1;
    float   Gen_Sweep_F2;
	
    float   Gen_Amplitude;
	
    float   Gen_Sweep_Time;
} 
#if defined BK2039 || defined BK2039A || defined BK2039_5CH
    Cont;
#elif   BK2039X
    Cont[GEN_CH_NUM];
#endif

eeprom struct _TEST_SEQUENCE_
{
    char    Active;

    // yjjang, 2012-08-27, 0.1 msec
    float   OnTime;
    float   OffTime;

    unsigned long RepeatNumber;

    // Generator Setup
    int     Gen_Func;

    float   Gen_Sine_Freq;
    int     Gen_Sweep_Mode;
    float   Gen_Sweep_F1;
    float   Gen_Sweep_F2;

    float   Gen_Amplitude;
    float   Gen_Time;
} Seq[SEQ_ON_OFF_NUMBER];

eeprom struct _TEST_SETUP_
{
    // Test Mode
    int     TestMode;
    long    TestNumber;
    // Test Count Registor
    int     TestSeq;
    int     SeqTestCount;
    long    TestCount;

    int     TestBeepActive;

    float   CurLimitLower;
    float   CurLimitUpper;

    int     LEDType;
} Setup;

typedef struct _TEST_RESULT_
{
    float   Cur;
    char    TestCur;
} TEST_RESULT;

TEST_RESULT Res[CUR_TEST_CHANNEL];

char        TestResAll;

#define CUR_OPN             0
#define CUR_SHT             1
#define CUR_NG              2
#define CUR_OK              3

#define LED_ALL         	20

#define LED_OFF             CUR_OPN
#define LED_RED             CUR_SHT
#define LED_RED_GREEN       CUR_NG
#define LED_GREEN           CUR_OK

#define CUR_OPEN_LIMIT      10      // < 20mA
#define CUR_SHORT_LIMIT     250     // > 200mA
#define CUR_SHORT			999     // 999

#if defined BK2039 || defined BK2039X || defined BK2039_5CH
#define GEN_SENS_CAL        (10.0) 	// Vrms
#define GEN_VOLT_MAX        (10.0) 	
#elif   BK2039A
#define GEN_SENS_CAL        (12.0) 	
#define GEN_VOLT_MAX        (12.0) 	
#endif

#define GEN_VOLT_CONT_MAX   (3.0)   

#define MAX_FRQ             (20000)  // Hz
#define MIN_FRQ             (10)   	// Hz

#define MIN_GEN_TIME        (0.001)	// Second

int CurrentCalMode = 0;

float CurOffsetBefore[CUR_TEST_CHANNEL];
float CurSensBefore[CUR_TEST_CHANNEL];

///////////////////////////////////////////
// Haptic Aging Setup
typedef struct _HAPTIC_WAVE
{
    float Freq;
    float Level;
    float Time;
} HAPTIC_WAVE, *PHAPTIC_WAVE;

typedef struct _HAPTIC_SIGNAL
{
    char            Active;
    unsigned long   RepeatNumber;
    HAPTIC_WAVE     Wave[HAPTIC_WAVE_NUMBER];
} HAPTIC_SIGNAL, *PHAPTIC_SIGNAL;

typedef struct _HAPTIC_SETUP
{
    unsigned long   TestNumber;    
    HAPTIC_SIGNAL   Signal[HAPTIC_SIG_NUMBER];
} HAPTIC_SETUP, *PHAPTIC_SETUP;

eeprom HAPTIC_SETUP HapticSetup; // <= Using eeprom because Global Memory is not enough.

eeprom struct _EEP_HAPTIC_SETUP
{
    unsigned long 	TestNumber; 
	
    char 			Active[HAPTIC_SIG_NUMBER];
    unsigned long 	RepeatNumber[HAPTIC_SIG_NUMBER];
    float 			Freq[HAPTIC_SIG_NUMBER][HAPTIC_WAVE_NUMBER];
    float 			Level[HAPTIC_SIG_NUMBER][HAPTIC_WAVE_NUMBER];
    float 			Time[HAPTIC_SIG_NUMBER][HAPTIC_WAVE_NUMBER];
} eepHapticSetup;

PHAPTIC_WAVE GetHapticSigWave(int sig, int wav)
{
    return((PHAPTIC_WAVE)&HapticSetup.Signal[sig].Wave[wav]);
}

PHAPTIC_SIGNAL GetHapticSignal(int sig)
{
    return((PHAPTIC_SIGNAL)&HapticSetup.Signal[sig]);
}

void EEP_LoadHapticSetup(void)
{
    int             nSig;
    int             nWave;
    PHAPTIC_SIGNAL	pSig;
    PHAPTIC_WAVE    pWave;
    
    HapticSetup.TestNumber = eepHapticSetup.TestNumber;
    
    for(nSig = 0; nSig < HAPTIC_SIG_NUMBER; nSig++)
    {
        pSig                = GetHapticSignal(nSig);
        pSig->Active        = eepHapticSetup.Active[nSig];
        pSig->RepeatNumber  = eepHapticSetup.RepeatNumber[nSig];
        for(nWave = 0; nWave < HAPTIC_WAVE_NUMBER; nWave++)
        {
            pWave           = GetHapticSigWave(nSig, nWave);
            pWave->Freq     = eepHapticSetup.Freq[nSig][nWave];
            pWave->Level    = eepHapticSetup.Level[nSig][nWave];
            pWave->Time     = eepHapticSetup.Time[nSig][nWave];
        }
    } 
}

void EEP_SaveHapticSetup(void)
{
    int nSig;
    int nWave;
    PHAPTIC_SIGNAL	pSig;
    PHAPTIC_WAVE pWave;
        
    eepHapticSetup.TestNumber = HapticSetup.TestNumber;
    
    for(nSig = 0; nSig < HAPTIC_SIG_NUMBER; nSig++)
    {
        pSig                                = GetHapticSignal(nSig);
        eepHapticSetup.Active[nSig]         = pSig->Active;
        eepHapticSetup.RepeatNumber[nSig]   = pSig->RepeatNumber;
        for(nWave = 0; nWave < HAPTIC_WAVE_NUMBER; nWave++)
        {
            pWave                               = GetHapticSigWave(nSig, nWave);
            eepHapticSetup.Freq[nSig][nWave]    = pWave->Freq;
            eepHapticSetup.Level[nSig][nWave]   = pWave->Level;
            eepHapticSetup.Time[nSig][nWave]    = pWave->Time;
        }
    } 
}

/////////////////////////////////////////////////////////////////////
eeprom struct _EEP_HAPTIC_COUNTER
{
    unsigned long   Test;
    unsigned int    SigCurrent;    
    unsigned long   SigRepeatStart;
    unsigned long   SigRepeat;
} eepHapticCount;

typedef struct _HAPTIC_COUNTER
{
    unsigned long   Test;
    unsigned int    SigCurrent;    
    unsigned long   SigRepeatStart;
    unsigned long   SigRepeat;    
} HAPTIC_COUNTER;

HAPTIC_COUNTER HapticCount;

///////////////////////////////////////////////////////////////////////
void DisplayTestHaptic(void);

////////////////////////////////////////////////////////////////////////////////
// UART
////////////////////////////////////////////////////////////////////////////////
#define RXB8    1
#define TXB8    0
#define UPE     2
#define OVR     3
#define FE      4
#define UDRE    5
#define RXC     7

#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<OVR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)

// USART0 Receiver buffer
#define RX_BUFFER_SIZE0 64
char rx_buffer0[RX_BUFFER_SIZE0];
unsigned int rx_wr_index0,rx_rd_index0,rx_counter0;
// This flag is set on USART0 Receiver buffer overflow
bit rx_buffer_overflow0;

//#if RX_BUFFER_SIZE0<256
//unsigned char rx_wr_index0,rx_rd_index0,rx_counter0;
//#else
//unsigned int rx_wr_index0,rx_rd_index0,rx_counter0;
//#endif

// USART Receiver interrupt service routine
interrupt [USART0_RXC] void usart0_rx_isr(void)
{
    char status, data;

    status = UCSR0A;
    data = UDR0;

    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
    {
        if (UART_RX_MODE == UART_RX_COMMAND)
        {
            rx_buffer0[rx_wr_index0]=data;
            if (++rx_wr_index0 == RX_BUFFER_SIZE0) rx_wr_index0=0;
            if (++rx_counter0 == RX_BUFFER_SIZE0)
            {
                rx_counter0=0;
                rx_buffer_overflow0=1;
            }
        }
        else if (UART_RX_MODE == UART_RX_IDATA)
        {
            rx_buffer0[rx_wr_index0]=data;
            if (++rx_wr_index0 == RX_BUFFER_SIZE0) rx_wr_index0=0;
            if (++rx_counter0 == RX_BUFFER_SIZE0)
            {
                rx_counter0=0;
                rx_buffer_overflow0=1;
            }
        }
        // Echo Debug
        //putchar(data);
    }
}

int getch(void)
{
    char data;

    if(rx_counter0 == 0) return(-1);
    data=rx_buffer0[rx_rd_index0];
    if (++rx_rd_index0 == RX_BUFFER_SIZE0) rx_rd_index0=0;
#asm("cli")
    --rx_counter0;
#asm("sei")
    return data;
}

#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART0 Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used+
char getchar(void)
{
    char data;
    while (rx_counter0==0);
    data=rx_buffer0[rx_rd_index0];
    if (++rx_rd_index0 == RX_BUFFER_SIZE0) rx_rd_index0=0;
#asm("cli")
    --rx_counter0;
#asm("sei")
    return data;
}
#pragma used-
#endif

// USART0 Transmitter buffer
#define TX_BUFFER_SIZE0 8
char tx_buffer0[TX_BUFFER_SIZE0];

#if TX_BUFFER_SIZE0 < 256
unsigned char tx_wr_index0,tx_rd_index0,tx_counter0;
#else
unsigned int tx_wr_index0,tx_rd_index0,tx_counter0;
#endif

// USART0 Transmitter interrupt service routine
interrupt [USART0_TXC] void usart0_tx_isr(void)
{
    if (tx_counter0)
    {
        --tx_counter0;
        UDR0=tx_buffer0[tx_rd_index0];
        if (++tx_rd_index0 == TX_BUFFER_SIZE0) tx_rd_index0=0;
    };
}

#ifndef _DEBUG_TERMINAL_IO_
// Write a character to the USART0 Transmitter buffer
#define _ALTERNATE_PUTCHAR_
#pragma used+
void putchar(char c)
{
    while (tx_counter0 == TX_BUFFER_SIZE0);
#asm("cli")

    if (tx_counter0 || ((UCSR0A & DATA_REGISTER_EMPTY)==0))
    {
        tx_buffer0[tx_wr_index0]=c;
        if (++tx_wr_index0 == TX_BUFFER_SIZE0) tx_wr_index0=0;
        ++tx_counter0;
    }
    else
    {
        UDR0=c;
    }

#asm("sei")
}
#pragma used-
#endif

/////////////////////////////////////////////////////////////////////////////
//UART Function
void UART_ClearRxBuffer(void)
{
#asm("cli")
    rx_wr_index0 = 0;
    rx_rd_index0 = 0;
    rx_counter0 = 0;
#asm("sei")
}

///////////////////////////////////////////////////////////////////////////////
float round(float value, int point)
{
    int   i, dir;
    float r_val;
    float d_val;

    if(value < 0)
    {
        value = value * -1;
        dir = -1;
    }
    else dir = 1;

    if(point > 0)
    {
        for(i = 0; i < (point - 1); i++)
        {
            value = value * 10;
        }

        r_val = (int)value;
        d_val = value - r_val;

        if(d_val >= 0.5) r_val = r_val + 1;

        for(i = 0; i < (point - 1); i++)
        {
            r_val = r_val / 10;
        }
    }
    else if(point < 0)
    {
        point = point * -1;
        for(i=0; i < point; i++)
        {
            value = value / 10;
        }

        r_val = (int)value;
        d_val = value - r_val;

        if(d_val >= 0.5) r_val = r_val + 1;

        for(i = 0; i < point; i++)
        {
            r_val = r_val * 10;
        }
    }
    else
    {
        r_val = value;
    }

    r_val = r_val * dir;

    return((float)r_val);
}

float Calc_Number(float value, int field, unsigned char aud)
{
    float tmp;

    tmp = value;

    if(aud == KEY_UP)
        switch(field)
        {
        case  7:
            tmp += 1000000.0;
            break;
        case  6:
            tmp += 100000.0;
            break;
        case  5:
            tmp += 10000.0;
            break;
        case  4:
            tmp += 1000.0;
            break;
        case  3:
            tmp += 100.0;
            break;
        case  2:
            tmp += 10.0;
            break;
        case  1:
            tmp += 1.0;
            break;
        case  0:
            tmp += 0.0;
            break;
        case -1:
            tmp += 0.1;
            break;
        case -2:
            tmp += 0.01;
            break;
        case -3:
            tmp += 0.001;
            break;
        case -4:
            tmp += 0.0001;
            break;

        }

    if(aud == KEY_DOWN)
        switch(field)
        {
        case  7: 
            if(tmp >= 1000000.0) tmp -= 1000000.0;
            break;
        case  6:
            if(tmp >= 100000.0) tmp -= 100000.0;
            break;
        case  5:
            if(tmp >= 10000.0) tmp -= 10000.0;
            break;
        case  4:
            if(tmp >= 1000.0) tmp -= 1000.0;
            break;
        case  3:
            if(tmp >= 100.0) tmp -= 100.0;
            break;
        case  2:
            if(tmp >= 10.0) tmp -= 10.0;
            break;
        case  1:
            if(tmp >= 1.0) tmp -= 1.0;
            break;
        case  0:
            if(tmp >= 0.0) tmp -= 0.0;
            break;
        case -1:
            if(tmp >= 0.1) tmp -= 0.1;
            break;
        case -2:
            if(tmp >= 0.01) tmp -= 0.01;
            break;
        case -3:
            if(tmp >= 0.001) tmp -= 0.001;
            break;
        case -4:
            if(tmp >= 0.0001) tmp -= 0.0001;
            break;
        }

    //tmp = round(tmp,5);

    return(tmp);
}

float Calc_Number_NEG(float value, int field, unsigned char aud)
{
    float tmp;

    tmp = value;

    if(aud == KEY_UP)
        switch(field)
        {
        case  7:
            tmp += 1000000.0;
            break;
        case  6:
            tmp += 100000.0;
            break;
        case  5:
            tmp += 10000.0;
            break;
        case  4:
            tmp += 1000.0;
            break;
        case  3:
            tmp += 100.0;
            break;
        case  2:
            tmp += 10.0;
            break;
        case  1:
            tmp += 1.0;
            break;
        case  0:
            tmp += 0.0;
            break;
        case -1:
            tmp += 0.1;
            break;
        case -2:
            tmp += 0.01;
            break;
        case -3:
            tmp += 0.001;
            break;
        case -4:
            tmp += 0.0001;
            break;

        }

    if(aud == KEY_DOWN)
        switch(field)
        {
        case  7: 
            tmp -= 1000000.0;
            break;
        case  6:
            tmp -= 100000.0;
            break;
        case  5:
            tmp -= 10000.0;
            break;
        case  4:
            tmp -= 1000.0;
            break;
        case  3:
            tmp -= 100.0;
            break;
        case  2:
            tmp -= 10.0;
            break;
        case  1:
            tmp -= 1.0;
            break;
        case  0:
            tmp -= 0.0;
            break;
        case -1:
            tmp -= 0.1;
            break;
        case -2:
            tmp -= 0.01;
            break;
        case -3:
            tmp -= 0.001;
            break;
        case -4:
            tmp -= 0.0001;
            break;
        }

    //tmp = round(tmp,5);

    return(tmp);
}

void BuzzerOn(int time_ms)
{
#ifdef BUZZER_ON
    REG_BUZZER = 0x01;
    delay_ms(time_ms);
    REG_BUZZER = 0x00;
#else BUZZER_OFF
    time_ms = 0;
#endif
}

void BuzzerOnOff(int on_ms, int off_ms)
{
#ifdef BUZZER_ON
    REG_BUZZER = 0x01;
    delay_ms(on_ms);
    REG_BUZZER = 0x00;
    delay_ms(off_ms);
#else BUZZER_OFF
    on_ms  = 0;
    off_ms = 0;
#endif                           
}

void BuzzerNumber(int on_ms, int off_ms, int num)
{
#ifdef BUZZER_ON
    while(num-- > 0)
    {
        REG_BUZZER = 0x01;
        delay_ms(on_ms);
        REG_BUZZER = 0x00;
        delay_ms(off_ms);
    }
#else BUZZER_OFF
    on_ms = 0;
    off_ms = 0;
    num = 0;
#endif
}

void BeepNG(void)
{
#ifdef BUZZER_ON
    if(Setup.TestBeepActive == 1)
    {
        BuzzerOn(10);
        delay_ms(30);
        BuzzerOn(10);
    }           
#else BUZZER_OFF
    delay_ms(1);
#endif
}

unsigned char ReadRegKeyButton(void)
{
    unsigned char key;
    unsigned char keyV;

    key = (~REG_KEY_BUTTON & 0x3C) | (~REG_KEY_PUSH & 0x01);

    if		((key&0x04) == 0x04)	keyV = KEY_SET  | KEY_ACTIVE;
    else if	((key&0x08) == 0x08)	keyV = KEY_RUN  | KEY_ACTIVE;
    else if	((key&0x10) == 0x10)	keyV = KEY_CAL  | KEY_ACTIVE;
    else if	((key&0x20) == 0x20)	keyV = KEY_ENT  | KEY_ACTIVE;
    else if	((key&0x01) == 0x01)	keyV = KEY_PUSH | KEY_ACTIVE;
    else    						keyV = 0x00;

    return(keyV);
}

#define KEY_DELAY	3

int Get_KEY(void)
{
    static unsigned char OnKey_loc=0;
    unsigned char key;
    unsigned char keyV;
    unsigned char keyE;
    unsigned char tmpV;
    unsigned char tmpE;

    key = ReadRegKeyButton();
    keyV = key & 0x0F;
    keyE = key & KEY_ACTIVE;
    //KEY_BUTTON = 0x00;

    if(keyE != KEY_ACTIVE) OnKey_loc = 0;

    if((keyE == KEY_ACTIVE) && (OnKey_loc == 0))
    {
        delay_ms(KEY_DELAY);
        key = ReadRegKeyButton();
        tmpV = key & 0x0F;
        tmpE = key & KEY_ACTIVE;
        if((tmpE == KEY_ACTIVE) && (tmpV == keyV))
        {
            delay_ms(KEY_DELAY);
            key = ReadRegKeyButton();
            tmpV = key & 0x0F;
            tmpE = key & KEY_ACTIVE;
            if((tmpE == KEY_ACTIVE) && (tmpV == keyV))
            {
                delay_ms(KEY_DELAY);
                key = ReadRegKeyButton();
                tmpV = key & 0x0F;
                tmpE = key & KEY_ACTIVE;
                if((tmpE == KEY_ACTIVE) && (tmpV == keyV))
                {
                    delay_ms(KEY_DELAY);
                    key = ReadRegKeyButton();
                    tmpV = key & 0x0F;
                    tmpE = key & KEY_ACTIVE;
                    if((tmpE == KEY_ACTIVE) && (tmpV == keyV))
                    {
                        BuzzerOn(5);
                        OnKey_loc = 1;
                        return(keyV);
                    }
                }
            }
        }
    }
    return(KEY_NON);
}

/////////////////////////////////////////
//  A_EVENT  - B_PHASE - DIRECTION
//   RISING  -    0    -   CW
//   RISING  -    1    -   CCW
//   FALLING -    0    -   CCW
//   FALLING -    1    -   CW


int  KEY_PHASE_COUNT_REG;
#define KEY_PHASE_RISING_BIT    (0x01)
#define KEY_PHASE_FALLING_BIT   (0x02)

int CheckPhaseCount(void)
{
    int value;
    value = KEY_PHASE_COUNT_REG;
    return(value);
}

void ClearPhaseCount(void)
{
    KEY_PHASE_COUNT_REG = 0;
}

//#define ENCODER_DELAY   75  //ms
//#define ENCODER_DELAY   25  //ms
#define ENCODER_DELAY   20  //ms
int Get_PhaseCount(void)
{
    int value;

    value = KEY_PHASE_COUNT_REG;

    if(value != 0)
    {
        REG_BUZZER = 0x01;
        delay_us(30);
        REG_BUZZER = 0x00;
        delay_ms(ENCODER_DELAY);
        KEY_PHASE_COUNT_REG = 0;
    }
    return(value);
}

// Phase A Rising interrupt
interrupt [EXT_INT6] void ext_int6_isr(void)
{
    unsigned char key;

    key = (~REG_KEY_BUTTON & 0x3C) | (~REG_KEY_PUSH & 0x01);

    if(key == 0x00)
    {
        if(REG_KEY_PHB == 0x00)
        {
            KEY_PHASE_COUNT_REG = 1;
        }
        else
        {
            KEY_PHASE_COUNT_REG = -1;
        }
    }
}

// Phase B Falling interrupt
interrupt [EXT_INT7] void ext_int7_isr(void)
{
    unsigned char key;

    key = (~REG_KEY_BUTTON & 0x3C) | (~REG_KEY_PUSH & 0x01);

    if(key == 0x00)
    {
        if(REG_KEY_PHA == 0x00)
        {
            KEY_PHASE_COUNT_REG = 1;
        }
        else
        {
            KEY_PHASE_COUNT_REG = -1;
        }
    }
}

void SetAMUX(char value)
{
    char mux;
    char mux1, mux2, mux3;

    mux = AMUXCH[value];

    if((mux & 0x01) == 0x01) mux1 = 0x01;
    else mux1 = 0;

    if((mux & 0x02) == 0x02) mux2 = 0x02;
    else mux2 = 0;

    if((mux & 0x04) == 0x04) mux3 = 0x04;
    else mux3 = 0;

    REG_CUR_MUX = mux1 | mux2 | mux3;
    
    delay_ms(1);
}

void RecordStart(int value)
{
    SetAMUX(value);
    ADC_MUX      = ADCCH[value] | ADC_VREF_AVCC;
    RecordLength = REC_LENGTH;
    RecordCount	 = 0;
    OnRecordEnd  = 0;
    OnRecord 	 = 1;
    ADMUX 		 = ADC_MUX;
    ADCSRA 		 |= 0x40;
}

void RecordStart_Cal(int value)
{
    SetAMUX(value);
    ADC_MUX      = ADCCH[value] | ADC_VREF_AVCC;
    RecordLength = REC_LENGTH_CAL;
    RecordCount	 = 0;
    OnRecordEnd  = 0;
    OnRecord 	 = 1;
    ADMUX 		 = ADC_MUX;
    ADCSRA 		 |= 0x40;
}

void RecordStop(void)
{
    OnRecord 	= 0;
    OnRecordEnd = 0;
    OnRecord	= 0;
}

// ADC interrupt service routine
//char adc_mon1=0;
//char adc_mon2=0;
interrupt [ADC_INT] void adc_isr(void)
{
    if(OnRecord == 1)
    {
        TimeData[RecordCount] = ADCW;

        if(++RecordCount >= RecordLength)
        {
            OnRecord = 0;
            OnRecordEnd = 1;

            // debug;
            //adc_mon1 = ~adc_mon1;
            //REG_LED1_G1 = adc_mon1;
        }
        else
        {
            // Select next ADC input & Start the AD conversion
            ADMUX = ADC_MUX;
            ADCSRA |= 0x40;
        }
    }
}

//////////////////////////////////////////////////
// Timer Setup
float   tick = 0.1;
//float   tickCount;

#define TIMER_EVENT_MOTOR_ON    0
#define TIMER_EVENT_MOTOR_OFF   1
#define TIMER_EVENT_FAST_ON     2
#define TIMER_EVENT_FAST_OFF    3
#define TIMER_EVENT_NUMBER      4

#define TIMER_SINGLE            0
#define TIMER_CONTINUE          1

typedef struct _TIMER_
{
    int Active;
    int Event;
    int OnEvent;
    int EventOption;
    int count;
    int interval;
} TIMER;

TIMER Timer[TIMER_EVENT_NUMBER];

void CreateTimer(void)
{
    int id;

    id = TIMER_EVENT_MOTOR_ON;
    Timer[id].count = 0;
    Timer[id].EventOption = TIMER_CONTINUE;
    Timer[id].Event = id;

    id = TIMER_EVENT_MOTOR_OFF;
    Timer[id].count = 0;
    Timer[id].EventOption = TIMER_SINGLE;
    Timer[id].Event = id;

    id = TIMER_EVENT_FAST_ON;
    Timer[id].count = 0;
    Timer[id].EventOption = TIMER_CONTINUE;
    Timer[id].Event = id;

    id = TIMER_EVENT_FAST_OFF;
    Timer[id].count = 0;
    Timer[id].EventOption = TIMER_SINGLE;
    Timer[id].Event = id;
}

void SetTimerActive(int id, int value)
{
    Timer[id].count = 0;
    Timer[id].OnEvent = 0;
    Timer[id].Active = value;
}

void SetTimerInterval(int id, float value)
{
    if(value < tick) value = tick;

    Timer[id].count = 0;
    Timer[id].OnEvent = 0;
    Timer[id].interval = (int)(value / tick);
}

char run_tick=0;
// Timer 1 overflow interrupt service routine
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{
    int i;

    //REG_LED1_G1 = timer_tick;

    // 1Sec
    //TCNT1H = 0x0B;
    //TCNT1L = 0xDC;

    // 0.1Sec
    TCNT1H = 0xE7;
    TCNT1L = 0x96;

    //tickCount += tick;

    for(i=0; i<TIMER_EVENT_NUMBER; i++)
    {
        if(Timer[i].Active == 1)
        {
            Timer[i].count++;
            if(Timer[i].count >= Timer[i].interval)
            {
                Timer[i].count = 0;
                Timer[i].OnEvent = 1;
                if(Timer[i].EventOption == TIMER_SINGLE)
                {
                    Timer[i].Active = 0;
                }
            }
        }
    }
    run_tick = 1 - run_tick;
}

//interrupt [TIM1_COMPA] void timer1_compa_isr(void)

////////////////////////////////////////////////////////////////////////////
// AVR Initialize
void MCU_InitSetup(void)
{
    // Declare your local variables here

    // Input/Output Ports initialization
    // Port A initialization
    PORTA=0x00;
    DDRA=0x00;

    // Port B initialization
    PORTB=0x00;
    DDRB=0xFE;

    // Port C initialization
    PORTC=0x00;
    DDRC=0x00;

    // Port D initialization
    PORTD=0x00;
    DDRD=0xFF;

    // Port E initialization
    PORTE=0x00;
    DDRE=0x00;

    // Port F initialization
    PORTF=0x00;
    DDRF=0x00;

    // Port G initialization
    PORTG=0x00;
    DDRG=0x1F;

    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: Timer 0 Stopped
    // Mode: Normal top=FFh
    // OC0 output: Disconnected
    ASSR=0x00;
    TCCR0=0x00;
    TCNT0=0x00;
    OCR0=0x00;

    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: 62.500 kHz
    // Mode: Normal top=FFFFh
    // OC1A output: Discon.
    // OC1B output: Discon.
    // OC1C output: Discon.
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    TCCR1A=0x00;
    TCCR1B=0x04;
    TCNT1H=0x0B;
    TCNT1L=0xD1;
    ICR1H=0x00;
    ICR1L=0x00;
    OCR1AH=0x00;
    OCR1AL=0x00;
    OCR1BH=0x00;
    OCR1BL=0x00;
    OCR1CH=0x00;
    OCR1CL=0x00;

    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: Timer 2 Stopped
    // Mode: Normal top=FFh
    // OC2 output: Disconnected
    TCCR2=0x00;
    TCNT2=0x00;
    OCR2=0x00;

    // Timer/Counter 3 initialization
    // Clock source: System Clock
    // Clock value: Timer 3 Stopped
    // Mode: Normal top=FFFFh
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // OC3A output: Discon.
    // OC3B output: Discon.
    // OC3C output: Discon.
    TCCR3A=0x00;
    TCCR3B=0x00;
    TCNT3H=0x00;
    TCNT3L=0x00;
    ICR3H=0x00;
    ICR3L=0x00;
    OCR3AH=0x00;
    OCR3AL=0x00;
    OCR3BH=0x00;
    OCR3BL=0x00;
    OCR3CH=0x00;
    OCR3CL=0x00;

    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    // INT2: Off
    // INT3: Off
    // INT4: Off
    // INT5: Off
    // INT6: On
    // INT6 Mode: Rising Edge
    // INT7: On
    // INT7 Mode: Falling Edge
    EICRA=0x00;
    EICRB=0xB0;
    EIMSK=0xC0;
    EIFR=0xC0;

    // External SRAM page configuration:
    // / 0000h - FFFFh
    // Upper page wait state(s): None
    //MCUCR=0x80;
    //XMCRA=0x00;

    // Upper page wait state(s): 1r/w
    MCUCR=0xC0;
    XMCRA=0x00;

    // WA3100 Natwork Module need Wait State 2
    // Upper page wait state(s): 2r/w
    //MCUCR=0x80;
    //XMCRA=0x02;

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK=0x04;
    ETIMSK=0x00;

    // USART0 initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART0 Receiver: On
    // USART0 Transmitter: On
    // USART0 Mode: Asynchronous
    // USART0 Baud rate: 9600   [0x67]
    // USART0 Baud rate: 14400  [0x44]
    // USART0 Baud rate: 19200  [0x33]
    // USART0 Baud rate: 38400  [0x19]
    // USART0 Baud rate: 57600  [0x10]
    // USART0 Baud rate: 115200 [0x08]
    UCSR0A=0x00;
    UCSR0B=0xD8;
    UCSR0C=0x06;
    UBRR0H=0x00;
    UBRR0L=0x67;

    // Analog Comparator initialization
    // Analog Comparator: Off
    // Analog Comparator Input Capture by Timer/Counter 1: Off
    ACSR=0x80;
    SFIOR=0x00;

    // ADC initialization
    // ADC Clock frequency: 125.000 kHz
    // ADC Voltage Reference: xxx pin
    ADMUX=ADC_VREF_AVCC;
    ADCSRA=0xCF;   
}

///////////////////////////////////////////////////////////////
// Calibration
void Cal_SetGenSens(int ch, float Value)
{
    if((Value <= 0) || (Value > 100)) Value = 1;

    Cal.Gen_Sens_Volt[ch] = Value;
}

////////////////////////////////////////////////////////////////////////////////
//LCD Function
#define LCD_CMD_INST    0
#define LCD_CMD_DATA    1
#define LCD_CMD_DISABLE 1
#define LCD_CMD_ENABLE  0

#define LCD_BYTE_DELAY  50 // us
#define LCD_BLANK       (0x20)
#define LCD_COL         (20)
#define LCD_ROW         (2)

/*void LCDWriteInst_R102(char data)
{
    char _data;

    _data = 0x00;
    _data |= (data << 1) & 0xAA;
    _data |= (data >> 1) & 0x55;

    REG_LCD_EN   = LCD_CMD_DISABLE;
    REG_LCD_A0   = LCD_CMD_INST;
    REG_LCD_DATA = _data;
    REG_LCD_EN   = LCD_CMD_ENABLE;
    REG_LCD_EN   = LCD_CMD_DISABLE;

    delay_us(LCD_BYTE_DELAY);
}*/

/*void LCDWriteByte_R102(char data)
{
    char _data;

    _data = 0x00;
    _data |= (data << 1) & 0xAA;
    _data |= (data >> 1) & 0x55;

    REG_LCD_EN   = LCD_CMD_DISABLE;
    REG_LCD_A0   = LCD_CMD_DATA;
    REG_LCD_DATA = _data;
    REG_LCD_EN   = LCD_CMD_ENABLE;
    REG_LCD_EN   = LCD_CMD_DISABLE;

	delay_us(LCD_BYTE_DELAY);
}*/
/*
void LCDWriteInst(char data){
    REG_LCD_EN   = LCD_CMD_DISABLE;
    REG_LCD_A0   = LCD_CMD_INST;
    REG_LCD_DATA = data;
    REG_LCD_EN   = LCD_CMD_ENABLE;
    REG_LCD_EN   = LCD_CMD_DISABLE;

    delay_us(LCD_BYTE_DELAY);
}
*/

void LCDWriteInst(char data)
{
    // VFD = 20T202DA1E, 2011년 이전
    /*
    REG_LCD_EN   = LCD_CMD_DISABLE;
    REG_LCD_A0   = LCD_CMD_INST;
    REG_LCD_DATA = data;
    REG_LCD_EN   = LCD_CMD_ENABLE;
    REG_LCD_EN   = LCD_CMD_DISABLE;
    delay_us(LCD_BYTE_DELAY);
    */
    // VFD = 20T202DA1E, 2011년 이후
    REG_LCD_A0   = 0;
    REG_LCD_DATA = data;
    REG_LCD_EN   = 1;
    REG_LCD_EN   = 0;
    delay_us(LCD_BYTE_DELAY);
}

void LCDWriteByte(char data)
{
    // VFD = 20T202DA1E, 2011년 이전
    /*
    REG_LCD_EN   = LCD_CMD_DISABLE;
    REG_LCD_A0   = LCD_CMD_DATA;
    REG_LCD_DATA = data;
    REG_LCD_EN   = LCD_CMD_ENABLE;
    REG_LCD_EN   = LCD_CMD_DISABLE;
    delay_us(LCD_BYTE_DELAY);
    */
    // VFD = 20T202DA1E, 2011년 이후
    REG_LCD_A0   = 1;
    REG_LCD_DATA = data;
    REG_LCD_EN   = 1;
    REG_LCD_EN   = 0;
    delay_us(LCD_BYTE_DELAY);
}

void Init_LCD(void)
{
    //REG_LCD_CONTROL =  LCD_CMD_EN;

    LCDWriteInst(0x38); // 8 bits, 2 row, 5 x 8 dots
    delay_ms(10);
    LCDWriteInst(0x38); // 8 bits, 2 row, 5 x 8 dots
    delay_ms(10);
    LCDWriteInst(0x0C); // display on, cursor off
    delay_ms(10);
    LCDWriteInst(0x06); // auto-increment, shift cursor
    delay_ms(10);
    LCDWriteInst(0x02); // Cursor Home
    delay_ms(10);
    LCDWriteInst(0x01); // clear display, return
    delay_ms(10);
}

void LCDCharPosition( char x,  char y)
{
    unsigned char position;

    if (y < 1) y = 1;
    if (x < 1) x = 1;

    if (y > LCD_ROW) y = LCD_ROW;
    if (x > LCD_COL) x = 20;

    x--;
    y--;
    position = y ? x + 0xc0 : x + 0x80;
    LCDWriteInst(position);
}

void LCDCursorOn(void)
{
    LCDWriteInst(LCD_CUR_ON);
}

void LCDCursorBlink(void)
{
    LCDWriteInst(LCD_CUR_BLK_ON);
}

void LCDCursorOff(void)
{
    LCDWriteInst(LCD_CUR_OFF);
}

void LCDSetCursor(char x, char y, char cursor)
{
    LCDCharPosition(x,y);

    if(cursor == LCD_CUR_ON)
        LCDCursorOn();
    else if(cursor == LCD_CUR_BLK_ON)
        LCDCursorBlink();
    else //if(cursor ==LCD_CUR_OFF)
        LCDCursorOff();
}

void LCDWriteStr( char x,  char y, char *text )
{
    char tpos,limit;

    tpos = 0;
    limit = LCD_COL;

    LCDCharPosition ( x, y );
    while ((*text != 0) && ((x+tpos) <= limit))
    {
        LCDWriteByte( *(text++));
        tpos++;
    }
}

void LCDUpdate(char *STR1, char *STR2)
{
    char a;
    char len1,len2;

    len1 = strlen(STR1);
    len2 = strlen(STR2);

    for(a=len1; a<LCD_COL; a++) STR1[a] = LCD_BLANK;
    for(a=len2; a<LCD_COL; a++) STR2[a] = LCD_BLANK;
    STR1[a] = 0;
    STR2[a] = 0;

    LCDCursorOff();
    LCDWriteStr(1,1,STR1);
    LCDWriteStr(1,2,STR2);
}

/////////////////////////////////////////////////////////////////
// Error Message
#define ERROR_GENRATOR    (1)

void LCDErrorMsg(int eNum)
{
    if(eNum == ERROR_GENRATOR)
    {
        //12345678901234567890
        sprintf(VSTR2, "ERROR_GENRATOR");
    }

    sprintf(VSTR1,"[MESSSAGE]");
    LCDUpdate(VSTR1,VSTR2);

    BuzzerNumber(500, 500, 4);
}

/////////////////////////////////////////////////////////////
#define LED_ANODE       0
#define LED_CATHODE     1

void LED(unsigned int number, unsigned int value)
{
    static unsigned int PORT1_R1 = 0x00;
    static unsigned int PORT1_R2 = 0x00;
    static unsigned int PORT1_G1 = 0x00;
    static unsigned int PORT1_G2 = 0x00;
	
    static unsigned int PORT2_R1 = 0x00;
    static unsigned int PORT2_R2 = 0x00;
    static unsigned int PORT2_G1 = 0x00;
    static unsigned int PORT2_G2 = 0x00;

    unsigned int port;
    unsigned int num;
    unsigned int *port_r;
    unsigned int *port_g;
    //unsigned char vbit;
    unsigned int vbit;

    if((number >= 0) && (number <= 9))
    {
        port = 0;
        num = number;
    }
    else if((number >= 10) && (number <= 19))
    {
        port = 1;
        num = number - 10;
    }
    else
    {
        port = LED_ALL;
    }

    if(port == LED_ALL)
    {
        if(value == LED_RED)
        {
            PORT1_R1 = 0xff;
            PORT1_R2 = 0xff;
            PORT1_G1 = 0x00;
            PORT1_G2 = 0x00;
#ifdef BK2039_5CH
            PORT2_R1 = 0x00;
            PORT2_R2 = 0x00;
            PORT2_G1 = 0x00;
            PORT2_G2 = 0x00;
#else
			PORT2_R1 = 0xff;
            PORT2_R2 = 0xff;
            PORT2_G1 = 0x00;
            PORT2_G2 = 0x00;
#endif
        }
        else if(value == LED_GREEN)
        {
            PORT1_R1 = 0x00;
            PORT1_R2 = 0x00;
            PORT1_G1 = 0xff;
            PORT1_G2 = 0xff;
#ifdef BK2039_5CH
			PORT2_R1 = 0x00;
            PORT2_R2 = 0x00;
            PORT2_G1 = 0x00;
            PORT2_G2 = 0x00;
#else			
			PORT2_R1 = 0x00;
            PORT2_R2 = 0x00;
            PORT2_G1 = 0xff;
            PORT2_G2 = 0xff;
#endif
        }
        else if(value == LED_OFF)
        {
            PORT1_R1 = 0x00;
            PORT1_R2 = 0x00;
            PORT1_G1 = 0x00;
            PORT1_G2 = 0x00;
			
            PORT2_R1 = 0x00;
            PORT2_R2 = 0x00;
            PORT2_G1 = 0x00;
            PORT2_G2 = 0x00;
        }
    }
    else
    {
        if(port == 0)
        {
            vbit = 0x01;
            // BK2019D, BX6138
            //if((num >= 0) && (num <= 4)){
            // BK2039, BX6244R10
            if((num >= 0) && (num <= 7))
            {
                port_r = (unsigned int*)&PORT1_R1;
                port_g = (unsigned int*)&PORT1_G1;
                vbit = vbit << num;
            }
            //else if((num >= 5) && (num <= 9)){
            else if((num >= 8) && (num <= 9))
            {
                port_r = (unsigned int*)&PORT1_R2;
                port_g = (unsigned int*)&PORT1_G2;
                //vbit = vbit << (num - 5);
                vbit = vbit << (num - 8);
            }
        }
        else if(port == 1)
        {
            vbit = 0x01;
            //if((num >= 0) && (num <= 4)){
            if((num >= 0) && (num <= 7))
            {
                port_r = (unsigned int*)&PORT2_R1;
                port_g = (unsigned int*)&PORT2_G1;
                vbit = vbit << num;
            }
            //else if((num >= 5) && (num <= 9)){
            else if((num >= 8) && (num <= 9))
            {
                port_r = (unsigned int*)&PORT2_R2;
                port_g = (unsigned int*)&PORT2_G2;
                //vbit = vbit << (num - 5);
                vbit = vbit << (num - 8);
            }
        }

        if(value == LED_OFF)
        {
            *port_r = *port_r & ~vbit;
            *port_g = *port_g & ~vbit;
        }
        else if(value == LED_RED)
        {
            *port_r = *port_r |  vbit;
            *port_g = *port_g & ~vbit;
        }
        else if(value == LED_GREEN)
        {
            *port_r = *port_r & ~vbit;
            *port_g = *port_g |  vbit;
        }
        else if(value == LED_RED_GREEN)
        {
            *port_r = *port_r |  vbit;
            *port_g = *port_g |  vbit;
        }
    }

    if(Setup.LEDType == LED_ANODE)
    {
        //LED Type: Anode Type
        // yjjang, 2013-10-22, ==>
        REG_LED1_R1 = PORT1_R1;
        REG_LED1_R2 = PORT1_R2;
        REG_LED2_R1 = PORT2_R1;
        REG_LED2_R2 = PORT2_R2;
        REG_LED1_G1 = PORT1_G1;
        REG_LED1_G2 = PORT1_G2;
        REG_LED2_G1 = PORT2_G1;
        REG_LED2_G2 = PORT2_G2;
    }
    else if(Setup.LEDType == LED_CATHODE)
    {
        // yjjang, 2013-10-22, <==
        //LED Type: Cathode Type
        // yjjang, 2010-05-25, ==>
        REG_LED1_R1 = ~PORT1_R1;
        REG_LED1_R2 = ~PORT1_R2;
        REG_LED2_R1 = ~PORT2_R1;
        REG_LED2_R2 = ~PORT2_R2;
        REG_LED1_G1 = ~PORT1_G1;
        REG_LED1_G2 = ~PORT1_G2;
        REG_LED2_G1 = ~PORT2_G1;
        REG_LED2_G2 = ~PORT2_G2;
        // yjjang, 2010-05-25, <==
    }
}

void Set_LED_RA_Result(unsigned int led)
{
    REG_LED1_R1  = led & 0x00ff;
    REG_LED1_R2  = (led >> 8) & 0x0003;
}

void Set_LED_GA_Result(unsigned int led)
{
    REG_LED1_G1  = led & 0x00ff;
    REG_LED1_G2  = (led >> 8) & 0x0003;
}

void Set_LED_RB_Result(unsigned int led)
{
    REG_LED2_R1  = led & 0x00ff;
    REG_LED2_R2  = (led >> 8) & 0x0003;
}

void Set_LED_GB_Result(unsigned int led)
{
    REG_LED2_G1  = led & 0x00ff;
    REG_LED2_G2  = (led >> 8) & 0x0003;
}


//********************************************
//* Generator
//********************************************
void GEN_DUMMY(void)
{
    int temp1;
    temp1 = DAC_RAM_CTL;
}

//////////////////////////////////////////////
// IDX Control Resistor
#define IDX_ATT_L		16
#define IDX_ATT_R		17
#define IDX_MUTE		18
#define IDX_OPER		19
#define IDX_FORMAT		20
#define IDX_RESER 		21
#define IDX_PHASE		22

void GEN_SetIDX(unsigned char idx, unsigned char data)
{
    DAC_IDX_CTL = data; //GEN_DUMMY();
    DAC_IDX_CTL = idx;  //GEN_DUMMY();
    delay_us(3);
}

void GEN_InitControl(void)
{
    GEN_SetIDX(IDX_ATT_L,  0xff);
    GEN_SetIDX(IDX_ATT_R,  0xff);
    GEN_SetIDX(IDX_MUTE,   0x00);
    GEN_SetIDX(IDX_OPER,   0x00);	// De-Emphasis Function 48kHz Disable
    GEN_SetIDX(IDX_FORMAT, 0x04); 	//I2S Format
    GEN_SetIDX(IDX_PHASE,  0x00);
}

void GEN_DACSamplingFreq(unsigned long sF)
{
    int GEN_SampFreq_ID;

    if		(sF == GEN_FREQ_192K) GEN_SampFreq_ID = GEN2_FREQ_ID_DIV1;	//192 kHz
    else if	(sF == GEN_FREQ_96K)  GEN_SampFreq_ID = GEN2_FREQ_ID_DIV2;	//96 kHz
    else if	(sF == GEN_FREQ_48K)  GEN_SampFreq_ID = GEN2_FREQ_ID_DIV4;	//48 kHz
    else
    {
        GEN_SampFreq_ID = GEN2_FREQ_ID_DIV4;
    }

    DAC_SamplingFreq = sF;
    DAC_SAMP_FREQ = GEN_SampFreq_ID;  //GEN_DUMMY();

    // Bug: DAC_SAMP_FREQ 설정이
    // GEN2_FREQ_ID_DIV4 = 192000
    // GEN2_FREQ_ID_DIV2 = 192000 X 2
    // GEN2_FREQ_ID_DIV1 = 192000 X 4
    //DAC_SamplingFreq = 192000;
    //DAC_SAMP_FREQ = GEN2_FREQ_ID_DIV1;//0
}

unsigned char GEN_CheckGenModule(void)
{
    unsigned char chk;
    unsigned int tmp1,tmp2;

    tmp1 = GEN_ACT_1 | GEN_ACT_2 | GEN_REAL_A_EN | GEN_REAL_B_EN;
    tmp2 = 0x00;

    DAC_RAM_CTL = tmp1;
    tmp2 = DAC_RAM_CTL & 0x3f;

    if(tmp1 == tmp2) chk = 1;
    else chk = 0;

    return(chk);
}

void GEN_SetAmplitudeMode(int ch, int value)
{
    int mode;

    mode = DAQ_AMPL_MODE & 0x03;
    if(ch == 1)
    {
        if(value == 0)
            DAQ_AMPL_MODE = mode & 0x02;
        else
            DAQ_AMPL_MODE = mode | 0x01;
    }
    else if(ch == 0)
    {
        if(value == 0)
            DAQ_AMPL_MODE = mode & 0x01;
        else
            DAQ_AMPL_MODE = mode | 0x02;
    }

    //GEN_DUMMY();
}

//////////////////////////////////////////////////////////////
// Generator Table 2
void GEN_MakeSineTable_2(unsigned long address, unsigned long length, unsigned long Bit, int type)
{
    unsigned long 	i;
    int 			iTmp;
    unsigned long 	index;
    unsigned long 	Fract_Num;
    float 			_delta;
    char 			GenLcdPosX = 0;
    unsigned int 	led[11] = {~0x0001, ~0x0002, ~0x0004, ~0x0008, ~0x0010, ~0x0020, ~0x0040, ~0x0080, ~0x0100, ~0x0200, ~0x0400};

    Fract_Num  = length << Bit;
    Angle_Max  = Fract_Num - 1;
    Angle_Step = (float)Fract_Num / DAC_SamplingFreq;

    // Control Mode
    DAC_RAM_CTL = GEN_CTL_MODE | GEN_CTL_AUTO;
    GEN_DUMMY();

    index = address * 2;

    DAC_INDEX_RESET = 0x00;
    GEN_DUMMY();
    DAC_RAM_ADD = BYTE_LL(index);
    DAC_RAM_ADD = BYTE_LH(index);
    DAC_RAM_ADD = BYTE_HL(index);

    _delta = _TWOPI / length;
	
    for(i = 0; i < length; i++)
    {
        iTmp = (int)(32767.0 * -sin(_delta * i));

        if(type == WAVE_RECT)
        {
            if(iTmp >= 0) 	iTmp =  32767;
            else 			iTmp = -32767;
        }

        DAC_RAM_DATA = BYTE_LOW(iTmp);
        DAC_RAM_DATA = BYTE_HIGH(iTmp);

        if((int)(i%1638) == 0)
        {
            GenLcdPosX++;
            LCDCharPosition(GenLcdPosX, 2);
            LCDWriteByte('>');

            LED(LED_ALL, LED_OFF);

            if(GenLcdPosX <= 10)
            {
#ifdef BK2039_5CH
				Set_LED_GA_Result(led[GenLcdPosX - 1]);
#else
				Set_LED_GA_Result(led[GenLcdPosX - 1]);
				Set_LED_GB_Result(led[GenLcdPosX - 1]);
#endif
            }
            else
            {
#ifdef BK2039_5CH
				Set_LED_RA_Result(led[GenLcdPosX - 1 - 10]);
#else				
                Set_LED_RA_Result(led[GenLcdPosX - 1 - 10]);
                Set_LED_RB_Result(led[GenLcdPosX - 1 - 10]);
#endif
            }
        }
    }
    // GEN RunMode
    DAC_RAM_CTL = GEN_RUN_MODE;
    GEN_DUMMY();

    DAC_INDEX_RESET = 0x00;
    GEN_DUMMY();
    DAC_TABLE_LEN = BYTE_LL(length);    //GEN_DUMMY();
    DAC_TABLE_LEN = BYTE_LH(length);    //GEN_DUMMY();
    DAC_TABLE_LEN = BYTE_HL(length);    //GEN_DUMMY();

    //SetDAQStatus(DAQ_GENOK);
}

void GEN_VerifySineTable_2(unsigned long address, unsigned long length, unsigned long Bit, int type)
{
    unsigned long 	i;
    int 			iTmp;
    int 			_iTmp;
    unsigned long	index;
    float 			_delta;
    unsigned long 	err;
    char 			GenLcdPosX = 0;

    // Control Mode
    DAC_RAM_CTL = GEN_CTL_MODE | GEN_CTL_AUTO;
    GEN_DUMMY();

    index = address;

    DAC_INDEX_RESET = 0x00;
    GEN_DUMMY();
    DAC_RAM_ADD = BYTE_LL(index);
    DAC_RAM_ADD = BYTE_LH(index);
    DAC_RAM_ADD = BYTE_HL(index);

    _delta = _TWOPI / length;
    err    = 0;
    i      = 0;
    
    do
    {
        iTmp = (int)(32767.0 * -sin(_delta * i));

        if(type == WAVE_RECT)
        {
            if(iTmp >= 0) iTmp =  32767;
            else          iTmp = -32767;
        }

        DAC_RAM_CTL &= ~0x40;
        *((unsigned char*)&_iTmp+0) = DAC_RAM_DATA & 0xff;
        DAC_RAM_CTL |= 0x40;
        *((unsigned char*)&_iTmp+1) = DAC_RAM_DATA & 0xff;

        if(iTmp != _iTmp)
        {
            err++;
        }

        if((int)(i%1638) == 0)
        {
            GenLcdPosX++;
            LCDCharPosition(GenLcdPosX, 2);
            LCDWriteByte('>');
        }
    }
    while((++i < length) && (err == 0));
	
    // GEN RunMode
    DAC_RAM_CTL = GEN_RUN_MODE;
    GEN_DUMMY();

    if(err == 0)
        SetDAQStatus(DAQ_GENOK);
    else
        ClrDAQStatus(DAQ_GENOK);
}

void GEN_InitSetup(void)
{
    if(GEN_CheckGenModule())
    {
        GEN_InitControl();
		
        GEN_DACSamplingFreq(GEN_FREQ_192K);
        //GEN_DACSamplingFreq(GEN_FREQ_96K);
        //GEN_DACSamplingFreq(GEN_FREQ_48K);
		
        sprintf(VSTR1, "System Booting");
        sprintf(VSTR2, " ");
        LCDUpdate(VSTR1, VSTR2);
        GEN_MakeSineTable_2(GEN_TABLE_ADDRESS, GEN_TABLE_NUM, GEN_FRACT_BITS, WAVE_SINE);
		
        SetDAQStatus(DAQ_GENOK);
    }
    else
    {
        ClrDAQStatus(DAQ_GENOK);
    }
}

void GEN_VerifySetup(void)
{
    char key, keyV, keyE;

    if((DAQStatus & DAQ_GENOK) != DAQ_GENOK) return;

    key  = ReadRegKeyButton();
    keyV = key & 0x0F;
    keyE = key & KEY_ACTIVE;

    if((keyV == KEY_ENT) && (keyE == KEY_ACTIVE))
    {
        sprintf(VSTR1, "System Testing");
        sprintf(VSTR2, " ");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerNumber(50, 50, 2);
        GEN_VerifySineTable_2(GEN_TABLE_ADDRESS, GEN_TABLE_NUM, GEN_FRACT_BITS, WAVE_SINE);
    }
}

/////////////////////////////////////////////////////////////////////
// Real Time Generator signal
void GEN_SetRealSignalFrequency(int ch, float freq)
{
    unsigned long d_angle;

    d_angle = (unsigned long)(Angle_Step * freq);
    if(ch == 0)
    {
        DAC_INDEX_RESET    = 0x00;              GEN_DUMMY();
        DAC_D_ANGLE_A_LOW  = BYTE_LL(d_angle);  //GEN_DUMMY();
        DAC_D_ANGLE_A_LOW  = BYTE_LH(d_angle);  //GEN_DUMMY();
        DAC_D_ANGLE_A_HIGH = BYTE_HL(d_angle);  //GEN_DUMMY();
        DAC_D_ANGLE_A_HIGH = BYTE_HH(d_angle);  //GEN_DUMMY();

        // Real Mode
        //DAC_RAM_CTL |= GEN_REAL_A_EN;
    }
    else if(ch == 1)
    {
        DAC_INDEX_RESET    = 0x00;              GEN_DUMMY();
        DAC_D_ANGLE_B_LOW  = BYTE_LL(d_angle);  //GEN_DUMMY();
        DAC_D_ANGLE_B_LOW  = BYTE_LH(d_angle);  //GEN_DUMMY();
        DAC_D_ANGLE_B_HIGH = BYTE_HL(d_angle);  //GEN_DUMMY();
        DAC_D_ANGLE_B_HIGH = BYTE_HH(d_angle);  //GEN_DUMMY();

        // Real Mode
        //DAC_RAM_CTL |= GEN_REAL_B_EN;         //GEN_DUMMY();
    }
}

void GEN_SetRealSignalAmplitude(int ch, float ampl)
{
    int iTmp;
    float tmpSens;

    if(GenReal[ch].SignalType == WAVE_RECT)
        ampl = ampl / 1.414213562;

    tmpSens = Cal.Gen_Sens_Volt[ch];

    if((ampl > tmpSens) || (ampl < 0)) ampl = tmpSens;
    iTmp = (int)(ampl / tmpSens * 32767);

    if(iTmp > 32767) iTmp = 32767;

    if(ch == 0)
    {
        DAC_INDEX_RESET = 0x00;
        GEN_DUMMY();
        DAC_AMPL_A = BYTE_LOW(iTmp);    //GEN_DUMMY();
        DAC_AMPL_A = BYTE_HIGH(iTmp);   //GEN_DUMMY();
    }
    else if(ch == 1)
    {
        DAC_INDEX_RESET = 0x00;
        GEN_DUMMY();
        DAC_AMPL_B = BYTE_LOW(iTmp);    //GEN_DUMMY();
        DAC_AMPL_B = BYTE_HIGH(iTmp);   //GEN_DUMMY();
    }
}

void GEN_SetRealSignalType(int ch, int value)
{
    GenReal[ch].SignalType = value;

    if(ch == 0)
    {
        DAC_SIGNAL_TYPE_A = value;
    }
    else if(ch == 1)
    {
        DAC_SIGNAL_TYPE_B = value;
    }

    GEN_DUMMY();
}

void GEN_RealSignalTrigger(int ch, int value)
{
    unsigned char gen_loc = 0;

    GEN_SetAmplitudeMode(ch, 0);    

    if(ch == 0)
    {
        gen_loc 	=   DAC_RAM_CTL;
        gen_loc 	&= ~GEN_CTL_MODE;
        gen_loc 	|=  GEN_CTL_AUTO;
        gen_loc 	|=  GEN_REAL_A_EN;
        DAC_RAM_CTL  =  gen_loc;

        if(value == GEN_READY)
        {            
            DAC_TRIG_A = GEN_TRIG_EN;
        }
        else if(value == GEN_SINGLE)
        {
            DAC_TRIG_A = GEN_TRIG_EN | GEN_TRIG_SIG;

        }
        else if(value == GEN_CONTINUE)
        {
            DAC_TRIG_A = 0x00;
        }
    }
    else if(ch == 1)
    {
        gen_loc 	=   DAC_RAM_CTL;
        gen_loc 	&= ~GEN_CTL_MODE;
        gen_loc 	|=  GEN_CTL_AUTO;
        gen_loc 	|=  GEN_REAL_B_EN;
        DAC_RAM_CTL  =  gen_loc;

        if(value == GEN_READY)
        {
            DAC_TRIG_B = GEN_TRIG_EN;
        }
        else if(value == GEN_SINGLE)
        {
            DAC_TRIG_B = GEN_TRIG_EN | GEN_TRIG_SIG;
        }
        else if(value == GEN_CONTINUE)
        {
            DAC_TRIG_B = 0x00;
        }
    }
}

unsigned long GEN_CalcIdx_SineM(int ch, PSIGNAL_WAVE pWave, unsigned long index, float TimeResolution)
{
    unsigned int    i;
    unsigned int    Index_Cnt;
    unsigned int    Index_Len;
    unsigned long   d_angle;
    unsigned long   length;
    unsigned long   _add;
    float           freq;
    float           ampl;   
    int             iAmpl;
    float           tmpSens;

    //char str[20];
    //unsigned int ampl_sine;

    if(pWave->Time <= 0) return(0);

    freq = pWave->Sine_Freq;
    if(freq > 32700) freq = 32700;

    d_angle = (unsigned long)(Angle_Step * freq);

    length    = (unsigned long)(pWave->Time * DAC_SamplingFreq);         
    Index_Cnt = (unsigned int)(DAC_SamplingFreq * TimeResolution);
    //Index_Cnt = (unsigned int)(DAC_SamplingFreq * GEN_TIME_RESOLUTION);
    Index_Len = (unsigned int)(length / Index_Cnt);// + 1;
    
//debug ///////////////////////////////////////////////////////////
/*
    sprintf(VSTR1, "Wave Time: %6.4f" , pWave->Time);
    sprintf(VSTR2, "Index_Len: %5d", Index_Len);
    LCDUpdate(VSTR1, VSTR2);
    BuzzerNumber(200, 200, 1);
    delay_ms(500);
*/
//debug ///////////////////////////////////////////////////////////

    //length    = (pWave->Time + GEN_TIME_RESOLUTION) * DAC_SamplingFreq;
    //length    = (unsigned long)((pWave->Time + GEN_TIME_RESOLUTION) * DAC_SamplingFreq);
    //Index_Cnt = (unsigned long)(DAC_SamplingFreq * GEN_TIME_RESOLUTION);
    //Index_Len = (unsigned long)(length / Index_Cnt);// + 1;
    
    // Control Mode
    DAC_RAM_CTL |= GEN_CTL_MODE | GEN_CTL_AUTO;
    GEN_DUMMY();

    _add = index * 2;
    ////////////////////////////////
    DAC_INDEX_RESET = 0x00;
    GEN_DUMMY();
    DAC_RAM_ADD = BYTE_LL(_add);
    DAC_RAM_ADD = BYTE_LH(_add);
    DAC_RAM_ADD = BYTE_HL(_add);

    for(i = 0; i < Index_Len; i++)
    {
        DAC_RAM_DATA = BYTE_LL(d_angle);
        DAC_RAM_DATA = BYTE_LH(d_angle);
        DAC_RAM_DATA = BYTE_HL(d_angle);
        DAC_RAM_DATA = BYTE_HH(d_angle);
    }

    // Run Mode
    DAC_RAM_CTL &= ~GEN_CTL_MODE;
    GEN_DUMMY();

    ////////////////////////////////////////////////////////////////
    // Amplitude
    // Control Mode
    DAC_RAM_CTL |= GEN_CTL_MODE | GEN_CTL_AUTO;
    GEN_DUMMY();

    if(pWave->SignalType == WAVE_RECT)
        ampl = pWave->Amplitude / 1.414213562;
    else
        ampl = pWave->Amplitude;

    tmpSens = Cal.Gen_Sens_Volt[ch];

    if(ampl > tmpSens) ampl = tmpSens; 
    else if(ampl < -tmpSens) ampl = -tmpSens; 
        
	iAmpl = (int)(ampl / tmpSens * 32767);
	if      (iAmpl > 32767)     iAmpl = 32767;
	else if (iAmpl < -32767)    iAmpl = -32767;
     
//debug ///////////////////////////////////////////////////////////
    /*
    sprintf(VSTR1, "%4.2f, %4.2f, %4.2f", pWave->Amplitude, ampl, Cal.Gen_Sens_Volt[ch]);
    sprintf(VSTR2, "%5d      ", iAmpl);
    LCDUpdate(VSTR1, VSTR2);
    BuzzerOn(5000);
    */
//debug ///////////////////////////////////////////////////////////

    _add = (index + GEN_AMPL_ADDRESS_OFFSET) * 2;
    DAC_INDEX_RESET = 0x00;
    GEN_DUMMY();
    DAC_RAM_ADD = BYTE_LL(_add);
    DAC_RAM_ADD = BYTE_LH(_add);
    DAC_RAM_ADD = BYTE_HL(_add);

    for(i = 0; i < Index_Len; i++)
    {
        DAC_RAM_DATA = BYTE_LL(iAmpl);
        DAC_RAM_DATA = BYTE_LH(iAmpl);
        DAC_RAM_DATA = BYTE_HL(iAmpl);
        DAC_RAM_DATA = BYTE_HH(iAmpl);
    }

    /////////////////////////////////////////////////
    // Run Mode
    DAC_RAM_CTL &= ~GEN_CTL_MODE;
    GEN_DUMMY();

    return(Index_Len);
}

unsigned long GEN_CalcIdx_SweepM(int ch, PSIGNAL_WAVE pWave, unsigned long index, float TimeResolution)
{
    float           freq;
    float           f_step;
    float           f_ratio;
    unsigned int    nFreq;
    unsigned long   d_angle;
    unsigned long   _add;
    unsigned long   sample;
    unsigned long   length;
    unsigned int    Index_Cnt;
    int             iAmpl;
    float           ampl;
    float           tmpSens;

    //int nBase;
    //unsigned int nOtv;
    //unsigned int chunk_sample;
    //char str[10];

    if(pWave->Time <= 0) return(0);

    // 2017-09-27, yjjang
    //length    = (unsigned long)((pWave->Time + GEN_TIME_RESOLUTION) * DAC_SamplingFreq);
    //Index_Cnt = (unsigned int)(DAC_SamplingFreq * GEN_TIME_RESOLUTION);

    length    = (unsigned long)(pWave->Time * DAC_SamplingFreq);         
    Index_Cnt = (unsigned int)(DAC_SamplingFreq * TimeResolution);
    nFreq     = (unsigned int)(length / Index_Cnt);// + 1;

    //nFreq = pWave->Sweep_nFreq;
    f_step  = (pWave->Sweep_F2 - pWave->Sweep_F1) / (nFreq-1);
    f_ratio = exp(log(pWave->Sweep_F2/pWave->Sweep_F1) / (nFreq-1));

    if((pWave->Sweep_Mode==SWEEP_LOG_UP)||(pWave->Sweep_Mode==SWEEP_LIN_UP))
    {
        freq = pWave->Sweep_F1;
    }
    else if((pWave->Sweep_Mode==SWEEP_LOG_DN)||(pWave->Sweep_Mode==SWEEP_LIN_DN))
    {
        freq = pWave->Sweep_F2;
    }
    else if((pWave->Sweep_Mode==SWEEP_LOG_UP_DN)||(pWave->Sweep_Mode==SWEEP_LIN_UP_DN))
    {
        freq = pWave->Sweep_F1;

        f_step  = (pWave->Sweep_F2 - pWave->Sweep_F1) / ((nFreq/2)-1);
        f_ratio = exp(log(pWave->Sweep_F2/pWave->Sweep_F1) / ((nFreq/2)-1));
    }

    // Control Mode
    DAC_RAM_CTL |= GEN_CTL_MODE | GEN_CTL_AUTO;
    GEN_DUMMY();

    _add = index * 2;
    DAC_INDEX_RESET = 0x00;
    GEN_DUMMY();
    DAC_RAM_ADD = BYTE_LL(_add);    //GEN_DUMMY();
    DAC_RAM_ADD = BYTE_LH(_add);    //GEN_DUMMY();
    DAC_RAM_ADD = BYTE_HL(_add);    //GEN_DUMMY();

    sample = 0;
    while(sample < length)
    {
        d_angle = (unsigned long)(Angle_Step * freq);
        sample += Index_Cnt;

        DAC_RAM_DATA = BYTE_LL(d_angle);    //GEN_DUMMY();
        DAC_RAM_DATA = BYTE_LH(d_angle);    //GEN_DUMMY();
        DAC_RAM_DATA = BYTE_HL(d_angle);    //GEN_DUMMY();
        DAC_RAM_DATA = BYTE_HH(d_angle);    //GEN_DUMMY();

        if(pWave->Sweep_Mode == SWEEP_LOG_UP) 		freq = freq * f_ratio;
        else if(pWave->Sweep_Mode == SWEEP_LIN_UP) 	freq = freq + f_step;
        else if(pWave->Sweep_Mode == SWEEP_LOG_DN) 	freq = freq / f_ratio;
        else if(pWave->Sweep_Mode == SWEEP_LIN_DN) 	freq = freq - f_step;
        else if(pWave->Sweep_Mode == SWEEP_LOG_UP_DN)
        {
            if(sample < (length/2))
                freq = freq * f_ratio;
            else
                freq = freq / f_ratio;
        }
        else if(pWave->Sweep_Mode == SWEEP_LIN_UP_DN)
        {
            if(sample < (length/2))
                freq = freq + f_step;
            else
                freq = freq - f_step;
        }
        //else
        //{
        //	freq = pow(10, (float)nOtv / nBase);
        //	nOtv++;
        //}
    }          
    
    // Run Mode
    DAC_RAM_CTL &= ~GEN_CTL_MODE;
    GEN_DUMMY();

    ////////////////////////////////////////////////////////////////
    // Amplitude
    // Control Mode
    DAC_RAM_CTL |= GEN_CTL_MODE | GEN_CTL_AUTO;
    GEN_DUMMY();

    _add = (index + GEN_AMPL_ADDRESS_OFFSET) * 2;
    DAC_INDEX_RESET = 0x00;
    GEN_DUMMY();
    DAC_RAM_ADD = BYTE_LL(_add);
    DAC_RAM_ADD = BYTE_LH(_add);
    DAC_RAM_ADD = BYTE_HL(_add);

    if(pWave->SignalType == WAVE_RECT)
        ampl = pWave->Amplitude / 1.414213562;
    else
        ampl = pWave->Amplitude;

    tmpSens = Cal.Gen_Sens_Volt[ch];

    if(ampl > tmpSens) ampl = tmpSens; 
    else if(ampl < -tmpSens) ampl = -tmpSens; 
    
	iAmpl = (int)(ampl / tmpSens * 32767);
	if      (iAmpl > 32767)     iAmpl = 32767;
	else if (iAmpl < -32767)    iAmpl = -32767;

    sample = 0;
    while(sample < length)
    {
        sample += Index_Cnt;

        DAC_RAM_DATA = BYTE_LL(iAmpl);
        DAC_RAM_DATA = BYTE_LH(iAmpl);
        DAC_RAM_DATA = BYTE_HL(iAmpl);
        DAC_RAM_DATA = BYTE_HH(iAmpl);
    }

    /////////////////////////////////////////////////
    // Run Mode
    DAC_RAM_CTL &= ~GEN_CTL_MODE;
    GEN_DUMMY();

    return(nFreq);
}

unsigned long GEN_CalcIdx_SignalM(int ch, PGENERATOR_SIGNAL_MULTI pSignalM, unsigned long index)
{
    int             i;
    unsigned long   gIndex;
    unsigned long   gLength;
    unsigned long   locLength;
    PSIGNAL_WAVE    pWave;

    gIndex      = index;
    locLength   = 0;
    for(i = 0; i < SIG_WAVE_NUM; i++)
    {
        pWave = (PSIGNAL_WAVE)&pSignalM->Wave[i];

        if(pWave->Func == GEN_NON)
        {
            gLength = 0;
        }
        else if(pWave->Func == GEN_SINE)
        {
            gLength = GEN_CalcIdx_SineM(ch, pWave, gIndex, pSignalM->TimeResolution);
        }
        else if(pWave->Func == GEN_SWEEP)
        {
            gLength = GEN_CalcIdx_SweepM(ch, pWave, gIndex, pSignalM->TimeResolution);
        }

        gIndex += gLength;
        locLength += gLength;
    }

    return(locLength);
}


unsigned int GEN_Calcuration_SignalM(int ch)
{
    int 					i;
    unsigned long 			gIndex;
    unsigned long 			gLength;
    PGENERATOR_SIGNAL_MULTI pSignalM;

    if      (ch == 0) gIndex = GEN_INDEX_ADDRESS_A;
    else if (ch == 1) gIndex = GEN_INDEX_ADDRESS_B; //GEN_INDEX_ADDRESS_B;
	
    gLength = 0;
    for(i = 0; i < GEN_SIGNAL_NUM; i++)
    {
        pSignalM 		= GetGenSignalM(ch, i);
        pSignalM->Index = gIndex;

        gLength = GEN_CalcIdx_SignalM(ch, pSignalM, gIndex);

        gIndex 			+= gLength;
        pSignalM->Length = gLength;
        pSignalM->Count  = (unsigned int)(DAC_SamplingFreq * pSignalM->TimeResolution);

//debug ///////////////////////////////////////////////////////////
    //sprintf(VSTR1, "Sig Count: %5d", pSignalM->Count);
    //sprintf(VSTR2, "Total Len: %5d", pSignalM->Length);
    //LCDUpdate(VSTR1, VSTR2);
    //BuzzerNumber(200, 200, 1);
    //delay_ms(500);
//debug ///////////////////////////////////////////////////////////
    }

    return(1);
}

void GEN_SignalMTrigger(int ch, int Sig, int value)
{
    unsigned char           gen_loc;
    PGENERATOR_SIGNAL_MULTI pSignalM;
    unsigned long           _add;

    pSignalM = GetGenSignalM(ch, Sig);

    GEN_SetAmplitudeMode(ch, 1);    

    if(ch == 0)
    {
        gen_loc     =   DAC_RAM_CTL;
        gen_loc     &= ~GEN_CTL_MODE;
        gen_loc     |=  GEN_CTL_AUTO;
        DAC_RAM_CTL =   gen_loc;
        
        DAC_SIGNAL_TYPE_A = pSignalM->Wave[0].SignalType;
        GEN_DUMMY();

        DAC_INDEX_RESET = 0x00;
        GEN_DUMMY();
        DAC_INDEX_LEN_A = BYTE_LOW(pSignalM->Length);
        DAC_INDEX_LEN_A = BYTE_HIGH(pSignalM->Length);

        DAC_INDEX_RESET = 0x00;
        GEN_DUMMY();
        DAC_INDEX_CNT_A = BYTE_LOW(pSignalM->Count);
        DAC_INDEX_CNT_A = BYTE_HIGH(pSignalM->Count);

        _add = pSignalM->Index * 2;
        DAC_INDEX_RESET = 0x00;
        GEN_DUMMY();
        DAC_INDEX_ADD_A = BYTE_LL(_add);
        DAC_INDEX_ADD_A = BYTE_LH(_add);
        DAC_INDEX_ADD_A = BYTE_HL(_add);

        _add = (pSignalM->Index + GEN_AMPL_ADDRESS_OFFSET) * 2;
        DAC_INDEX_RESET = 0x00;
        GEN_DUMMY();
        DAC_AMPL_INDEX_ADD_A = BYTE_LL(_add);
        DAC_AMPL_INDEX_ADD_A = BYTE_LH(_add);
        DAC_AMPL_INDEX_ADD_A = BYTE_HL(_add);

        gen_loc     =   DAC_RAM_CTL;
        gen_loc     &= ~GEN_CTL_MODE;
        gen_loc     |=  GEN_CTL_AUTO;
        gen_loc     &= ~GEN_REAL_A_EN;
        DAC_RAM_CTL =   gen_loc;

        if(value == GEN_READY)
        {
            DAC_TRIG_A = GEN_TRIG_EN;
        }
        else if(value == GEN_SINGLE)
        {
            DAC_TRIG_A = GEN_TRIG_EN | GEN_TRIG_SIG;
        }
        else if(value == GEN_CONTINUE)
        {
            DAC_TRIG_A = 0x00;
        }
    }
    else if(ch == 1)
    {
        gen_loc     =   DAC_RAM_CTL;
        gen_loc     &= ~GEN_CTL_MODE;
        gen_loc     |=  GEN_CTL_AUTO;
        DAC_RAM_CTL =   gen_loc;
        
        DAC_SIGNAL_TYPE_B = pSignalM->Wave[0].SignalType;

        DAC_INDEX_RESET = 0x00;
        GEN_DUMMY();
        DAC_INDEX_LEN_B = BYTE_LOW(pSignalM->Length);
        DAC_INDEX_LEN_B = BYTE_HIGH(pSignalM->Length);

        DAC_INDEX_RESET = 0x00;
        GEN_DUMMY();
        DAC_INDEX_CNT_B = BYTE_LOW(pSignalM->Count);
        DAC_INDEX_CNT_B = BYTE_HIGH(pSignalM->Count);

        _add = pSignalM->Index * 2;
        DAC_INDEX_RESET = 0x00;
        GEN_DUMMY();
        DAC_INDEX_ADD_B = BYTE_LL(_add);                   
        DAC_INDEX_ADD_B = BYTE_LH(_add);		           
        DAC_INDEX_ADD_B = BYTE_HL(_add);                   

        _add = (pSignalM->Index +  GEN_AMPL_ADDRESS_OFFSET) * 2;
        DAC_INDEX_RESET = 0x00;
        GEN_DUMMY();
        DAC_AMPL_INDEX_ADD_B = BYTE_LL(_add);              
        DAC_AMPL_INDEX_ADD_B = BYTE_LH(_add);  
        DAC_AMPL_INDEX_ADD_B = BYTE_HL(_add);

        gen_loc     =   DAC_RAM_CTL;
        gen_loc     &= ~GEN_CTL_MODE;
        gen_loc     |=  GEN_CTL_AUTO;
        gen_loc     &= ~GEN_REAL_B_EN;
        DAC_RAM_CTL =   gen_loc;

        if(value == GEN_READY)
        {
            DAC_TRIG_B = GEN_TRIG_EN;
        }
        else if(value == GEN_SINGLE)
        {
            DAC_TRIG_B = GEN_TRIG_EN | GEN_TRIG_SIG;
        }
        else if(value == GEN_CONTINUE)
        {
            DAC_TRIG_B = 0x00;
        }
    }
}

void GEN_SetSignalCycleLength(unsigned long aLength)
{
    if(aLength > MAX_SIGNAL_REPEAT_NUMBER) aLength = MAX_SIGNAL_REPEAT_NUMBER;
     
    DAC_CYCLE_LENGTH = BYTE_LOW(aLength);
    DAC_CYCLE_LENGTH = BYTE_HIGH(aLength);
}

void GEN_SetSignalCycleActive(unsigned int value)
{
    DAC_CYCLE_CONRTOL = value & 0x01;
}

unsigned int GEN_ReadSignalCycleCount(void)
{
    unsigned int ldata;
    unsigned int hdata;
    unsigned int count;
    
    ldata = DAC_CYCLE_COUNTER_LOW  & 0x00ff;
    hdata = DAC_CYCLE_COUNTER_HIGH & 0x00ff;
    count = ((hdata << 8) & 0xff00) + ldata;
    
    return(count);
}
// Haptic Generator /////////////////////////////////////////////
void HAP_SetGeneratorSignal(unsigned int nSig)
{
    int nWave;
    PHAPTIC_SIGNAL pSig;
    PGENERATOR_SIGNAL_MULTI pSignalM;

    pSignalM = GetGenSignalM(GEN_CH, 0);
    pSig     = GetHapticSignal(nSig);
    
    pSignalM->TimeResolution = GEN_TIME_RESOLUTION_HAPTIC;
    for(nWave = 0; nWave < HAPTIC_WAVE_NUMBER; nWave++)
    {
        pSignalM->Wave[nWave].Func       = GEN_SINE;
        pSignalM->Wave[nWave].SignalType = WAVE_SINE;
        pSignalM->Wave[nWave].Sine_Freq  = pSig->Wave[nWave].Freq;
        pSignalM->Wave[nWave].Amplitude  = pSig->Wave[nWave].Level;
        pSignalM->Wave[nWave].Time       = pSig->Wave[nWave].Time;
    }   
    
    GEN_Calcuration_SignalM(GEN_CH);
    GEN_SetSignalCycleActive(1);
}

// Haptic Repeat Counter ///////////////////////////////////////
void EEP_LoadHapticCount(void)
{
    HapticCount.Test            = eepHapticCount.Test;
    HapticCount.SigCurrent      = eepHapticCount.SigCurrent;
    HapticCount.SigRepeat       = eepHapticCount.SigRepeat;
    HapticCount.SigRepeatStart  = eepHapticCount.SigRepeatStart;      
}

void EEP_SaveHapticCount(void)
{
    if(HapticCount.Test != eepHapticCount.Test)
    {
        eepHapticCount.Test            = HapticCount.Test;
        eepHapticCount.SigCurrent      = HapticCount.SigCurrent;
        eepHapticCount.SigRepeat       = HapticCount.SigRepeat;
        eepHapticCount.SigRepeatStart  = HapticCount.SigRepeatStart; 
    
        sprintf(VSTR1,"Test Count is Saved");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerNumber(300, 200, 3);
        delay_ms(200);
    }
}

void HAP_CheckSaveCount(void)
{
    if((HapticCount.Test % 1000) == 0)
    {   
        DisplayTestHaptic();
        EEP_SaveHapticCount();
    } 
}

unsigned long HAP_GetSignalRepeatCount(void)
{
    return(GEN_ReadSignalCycleCount());
}

void HAP_ClearSignalRepeatCount(void)
{
    HapticCount.SigRepeat       = 0;
    HapticCount.SigRepeatStart  = 0;
}

void HAP_SetSignalRepeatCount(void)
{
    //char lstr1[10],lstr2[10],lstr3[10],lstr4[10],lstr5[10];
    unsigned int nSig;
    unsigned long RepeatNumber;
    PHAPTIC_SIGNAL pSig;

    nSig = HapticCount.SigCurrent;
    pSig = GetHapticSignal(nSig);
    
    if(HapticCount.SigRepeat >= pSig->RepeatNumber)
    {
        HapticCount.SigRepeat       = pSig->RepeatNumber;
        HapticCount.SigRepeatStart  = HapticCount.SigRepeat; 
        GEN_SetSignalCycleLength(pSig->RepeatNumber);    
    }
    else
    {
        RepeatNumber                = pSig->RepeatNumber - HapticCount.SigRepeat;
        HapticCount.SigRepeatStart  = HapticCount.SigRepeat;
        GEN_SetSignalCycleLength(RepeatNumber);    
    }    

/*
    ltoa(pSig->RepeatNumber, lstr2);
    ltoa(HapticCount.SigRepeatStart, lstr3);
    ltoa(HapticCount.SigRepeat, lstr4);
    ltoa(RepeatNumber, lstr5);
    
    sprintf(VSTR1,"%%d/%s", nSig, lstr2);
    sprintf(VSTR2,"%s/%s/%s", lstr3, lstr4, lstr5);
    LCDUpdate(VSTR1, VSTR2);
    BuzzerNumber(300, 200, 3);
    delay_ms(3000);
*/
}
 
int HAP_CheckSignalRepeatCountEnd(void)
{
    unsigned int    nSig;            
    PHAPTIC_SIGNAL  pSig;
    
    nSig = HapticCount.SigCurrent;
    pSig = GetHapticSignal(nSig);

    HapticCount.SigRepeat = HAP_GetSignalRepeatCount() + HapticCount.SigRepeatStart;
        
    if(HapticCount.SigRepeat >= pSig->RepeatNumber)
        return(1);
    else 
        return(0);
}

//////////////////////////////////////////////////////////////
void Init_Cal(void)
{
    int i;

    for(i = 0; i < GEN_CH_NUM; i++)
    {
        Cal.Gen_Sens_Volt[i] = GEN_SENS_CAL;
    }
}

void Init_Cal_Current(void)
{
    int i;
#if defined BK2039 || defined BK2039X || defined BK2039_5CH
    for(i = 0; i < CUR_TEST_CHANNEL; i++)
    {
        if((Cal.Cur_Offset[i] < -100) || (Cal.Cur_Offset[i] > 100))  Cal.Cur_Offset[i] = 0;
        if((Cal.Cur_Sens[i]   < 0.1)  || (Cal.Cur_Sens[i]   > 9.99)) Cal.Cur_Sens[i]   = 0.935;
    }
#elif BK2039A
    for(i = 0; i < CUR_TEST_CHANNEL; i++)
    {
        Cal.Cur_Offset[i] = 0;
        Cal.Cur_Sens[i]   = 1.0;
    }
#endif
}

/*
void Debug_Init_SetupHaptic(void)
{
    int nSig;  
    int nWave;
    PHAPTIC_SIGNAL pSig; 
    
    // Initialize Haptic Counter /////////
    HapticCount.Test = 0;
    HapticCount.SigCurrent = 0;
    HapticCount.SigRepeatStart = 0;
    HapticCount.SigRepeat = 0;       
    //EEP_SaveHapticCounter();
    EEP_SaveHapticCount();
    
    // Initialize Haptic Setup //////////      
    HapticSetup.TestNumber = MAX_TEST_NUMBER;
    
    nSig=0;
    pSig = GetHapticSignal(nSig); 
    pSig->Active = 1;
    pSig->RepeatNumber = 1; 
    for(nWave=0; nWave<HAPTIC_WAVE_NUMBER; nWave++)
    { 
        pSig->Wave[nWave].Freq  = 200;
        pSig->Wave[nWave].Level = nWave;
        pSig->Wave[nWave].Time  = 0.010; 
    }
    
    nSig=1;
    pSig = GetHapticSignal(nSig); 
    pSig->Active = 0;//1;
    pSig->RepeatNumber = 10; 
    for(nWave=0; nWave<HAPTIC_WAVE_NUMBER; nWave++)
    { 
        pSig->Wave[nWave].Freq  = 200;
        pSig->Wave[nWave].Level = HAPTIC_WAVE_NUMBER - (nWave+1);
        pSig->Wave[nWave].Time  = 0.010; 
    } 
        
    nSig=2;
    pSig = GetHapticSignal(nSig); 
    pSig->Active = 0;//1;
    pSig->RepeatNumber = 10; 
    for(nWave=0; nWave<HAPTIC_WAVE_NUMBER; nWave++)
    { 
        pSig->Wave[nWave].Freq  = 200;
        pSig->Wave[nWave].Level = nWave * 0.5;
        pSig->Wave[nWave].Time  = 0.010; 
    }
    
    nSig=3;
    pSig = GetHapticSignal(nSig); 
    pSig->Active = 0;//1;
    pSig->RepeatNumber = 10; 
    for(nWave=0; nWave<HAPTIC_WAVE_NUMBER; nWave++)
    { 
        pSig->Wave[nWave].Freq  = 200;
        pSig->Wave[nWave].Level = (HAPTIC_WAVE_NUMBER - (nWave+1)) * 0.5;
        pSig->Wave[nWave].Time  = 0.010; 
    } 
        
    nSig=4;
    pSig = GetHapticSignal(nSig); 
    pSig->Active = 0;//1;
    pSig->RepeatNumber = 10;     
        
    pSig->Wave[nWave].Freq  = (nWave * 100);
    pSig->Wave[nWave].Level = 0;
    pSig->Wave[nWave].Time  = 0.010;
            
    for(nWave=1; nWave<HAPTIC_WAVE_NUMBER; nWave++)
    { 
        pSig->Wave[nWave].Freq  = (nWave * 100);
        pSig->Wave[nWave].Level = 3;
        pSig->Wave[nWave].Time  = 0.010; 
    }
        
   EEP_SaveHapticSetup();        
}
*/

/*void Debug_Init_SetupHaptic_2(void)
{
    int nSig;  
    int nWave;
    PHAPTIC_SIGNAL pSig; 
    
    // Initialize Haptic Counter /////////
    HapticCount.Test = 0;
    HapticCount.SigCurrent = 0;
    HapticCount.SigRepeatStart = 0;
    HapticCount.SigRepeat = 0; 
    EEP_SaveHapticCounter();
    
    // Initialize Haptic Setup //////////      
    HapticSetup.TestNumber = MAX_TEST_NUMBER;
    
        nSig=0;
        pSig = GetHapticSignal(nSig); 
        pSig->Active = 1;
        pSig->RepeatNumber = 1; 
        for(nWave=0; nWave<HAPTIC_WAVE_NUMBER; nWave++)
        { 
            pSig->Wave[nWave].Freq  = 200;
            pSig->Wave[nWave].Level = nWave;
            pSig->Wave[nWave].Time  = 0.5; 
        }
    
        nSig=1;
        pSig = GetHapticSignal(nSig); 
        pSig->Active = 0;
        pSig->RepeatNumber = 10; 
        for(nWave=0; nWave<HAPTIC_WAVE_NUMBER; nWave++)
        { 
            pSig->Wave[nWave].Freq  = 200;
            pSig->Wave[nWave].Level = HAPTIC_WAVE_NUMBER - (nWave+1);
            pSig->Wave[nWave].Time  = 0.010; 
        } 
        
        nSig=2;
        pSig = GetHapticSignal(nSig); 
        pSig->Active = 0;
        pSig->RepeatNumber = 10; 
        for(nWave=0; nWave<HAPTIC_WAVE_NUMBER; nWave++)
        { 
            pSig->Wave[nWave].Freq  = 200;
            pSig->Wave[nWave].Level = nWave * 0.5;
            pSig->Wave[nWave].Time  = 0.010; 
        }
    
        nSig=3;
        pSig = GetHapticSignal(nSig); 
        pSig->Active = 0;
        pSig->RepeatNumber = 10; 
        for(nWave=0; nWave<HAPTIC_WAVE_NUMBER; nWave++)
        { 
            pSig->Wave[nWave].Freq  = 200;
            pSig->Wave[nWave].Level = (HAPTIC_WAVE_NUMBER - (nWave+1)) * 0.5;
            pSig->Wave[nWave].Time  = 0.010; 
        } 
        
        nSig=4;
        pSig = GetHapticSignal(nSig); 
        pSig->Active = 0;
        pSig->RepeatNumber = 10;     
        
        pSig->Wave[nWave].Freq  = (nWave * 100);
        pSig->Wave[nWave].Level = 0;
        pSig->Wave[nWave].Time  = 0.010;
            
        for(nWave=1; nWave<HAPTIC_WAVE_NUMBER; nWave++)
        { 
            pSig->Wave[nWave].Freq  = (nWave * 100);
            pSig->Wave[nWave].Level = 3;
            pSig->Wave[nWave].Time  = 0.010; 
        }
        
       EEP_SaveHapticSetup(); 
        
}*/

void Init_SetupHaptic(void)
{
    int nSig;  
    int nWave;
    PHAPTIC_SIGNAL pSig; 
    
    // Debug ////////////////////
    //Debug_Init_SetupHaptic();    
    //Debug_Init_SetupHaptic_2();
    //return;
    // Debug ////////////////////
    
    // Initialize Haptic Counter /////////
    HapticCount.Test            = 0;
    HapticCount.SigCurrent      = 0;
    HapticCount.SigRepeatStart  = 0;
    HapticCount.SigRepeat       = 0;
    EEP_SaveHapticCount();
    
    // Initialize Haptic Setup //////////      
    HapticSetup.TestNumber = MAX_TEST_NUMBER;
    
    for(nSig = 0; nSig < HAPTIC_SIG_NUMBER; nSig++)
    {
        pSig = GetHapticSignal(nSig); 
        pSig->Active        = 0;
        pSig->RepeatNumber  = 3; 
        for(nWave = 0; nWave < HAPTIC_WAVE_NUMBER; nWave++)
        { 
            pSig->Wave[nWave].Freq  = 200;
            pSig->Wave[nWave].Level = 0.0;
            pSig->Wave[nWave].Time  = 0.0; 
        }
    }
        
    // SEQ1 ///////////////////////////
    nSig                    = 0;
    pSig                    = GetHapticSignal(nSig); 
    pSig->Active            = 1;
    pSig->RepeatNumber      = 10;//200;     
    nWave                   = 0;
    pSig->Wave[nWave].Freq  = 100;  
    pSig->Wave[nWave].Level = 9.0;
    pSig->Wave[nWave].Time  = 0.100;    
    nWave                   = 1;
    pSig->Wave[nWave].Freq  = 100;  
    pSig->Wave[nWave].Level = -9.0;
    pSig->Wave[nWave].Time  = 0.100;     
    nWave                   = 2;
    pSig->Wave[nWave].Freq  = 400;  
    pSig->Wave[nWave].Level = 3.0;
    pSig->Wave[nWave].Time  = 0.000;     
    nWave                   = 3;
    pSig->Wave[nWave].Freq  = 500;  
    pSig->Wave[nWave].Level = 4.0;
    pSig->Wave[nWave].Time  = 0.000; 
    
    // SEQ2 ///////////////////////////
    nSig                    = 1;
    pSig                    = GetHapticSignal(nSig); 
    pSig->Active            = 1;
    pSig->RepeatNumber      = 10;     
    nWave                   = 0;
    pSig->Wave[nWave].Freq  = 100;  
    pSig->Wave[nWave].Level = 9.0;
    pSig->Wave[nWave].Time  = 0.100;    
    nWave                   = 1;
    pSig->Wave[nWave].Freq  = 400;  
    pSig->Wave[nWave].Level = 3.0;
    pSig->Wave[nWave].Time  = 0.000;     
    nWave                   = 2;
    pSig->Wave[nWave].Freq  = 300;  
    pSig->Wave[nWave].Level = 2.0;
    pSig->Wave[nWave].Time  = 0.000;     
    nWave                   = 3;
    pSig->Wave[nWave].Freq  = 200;  
    pSig->Wave[nWave].Level = 1.0;
    pSig->Wave[nWave].Time  = 0.000; 

    // SEQ3 ///////////////////////////
    nSig = 2;
    pSig = GetHapticSignal(nSig); 
    pSig->Active            = 0;//1;
    pSig->RepeatNumber      = 100;     
    nWave                   = 0;
    pSig->Wave[nWave].Freq  = 200;  
    pSig->Wave[nWave].Level = 4.0;
    pSig->Wave[nWave].Time  = 0.010;    
    nWave                   = 1;
    pSig->Wave[nWave].Freq  = 300;  
    pSig->Wave[nWave].Level = 3.0;
    pSig->Wave[nWave].Time  = 0.010;     
    nWave                   = 2;
    pSig->Wave[nWave].Freq  = 400;  
    pSig->Wave[nWave].Level = 2.0;
    pSig->Wave[nWave].Time  = 0.010;     
    nWave                   = 3;
    pSig->Wave[nWave].Freq  = 500;  
    pSig->Wave[nWave].Level = 1.0;
    pSig->Wave[nWave].Time  = 0.010; 

    // SEQ4 ///////////////////////////
    nSig                    = 3;
    pSig                    = GetHapticSignal(nSig); 
    pSig->Active            = 0;//1;
    pSig->RepeatNumber      = 100; 
    nWave                   = 0;
    pSig->Wave[nWave].Freq  = 500;  
    pSig->Wave[nWave].Level = 4.0;
    pSig->Wave[nWave].Time  = 0.010;    
    nWave                   = 1;
    pSig->Wave[nWave].Freq  = 400;  
    pSig->Wave[nWave].Level = 3.0;
    pSig->Wave[nWave].Time  = 0.010;     
    nWave                   = 2;
    pSig->Wave[nWave].Freq  = 300;  
    pSig->Wave[nWave].Level = 2.0;
    pSig->Wave[nWave].Time  = 0.010;     
    nWave                   = 3;
    pSig->Wave[nWave].Freq  = 200;  
    pSig->Wave[nWave].Level = 1.0;
    pSig->Wave[nWave].Time  = 0.010; 
    
    // SEQ5 ///////////////////////////
    nSig                    = 4;
    pSig                    = GetHapticSignal(nSig); 
    pSig->Active            = 0;//1;
    pSig->RepeatNumber      = 100; 
    nWave                   = 0;
    pSig->Wave[nWave].Freq  = 200;  
    pSig->Wave[nWave].Level = 4.0;
    pSig->Wave[nWave].Time  = 0.010;    
    nWave                   = 1;
    pSig->Wave[nWave].Freq  = 300;  
    pSig->Wave[nWave].Level = 3.0;
    pSig->Wave[nWave].Time  = 0.010;     
    nWave                   = 2;
    pSig->Wave[nWave].Freq  = 400;  
    pSig->Wave[nWave].Level = 2.0;
    pSig->Wave[nWave].Time  = 0.010;     
    nWave                   = 3;
    pSig->Wave[nWave].Freq  = 500;  
    pSig->Wave[nWave].Level = 1.0;
    pSig->Wave[nWave].Time  = 0.010;   
    
    EEP_SaveHapticSetup();
}

void Init_Setup(void)
{
    int i;
    
    // Test Setup             
    Setup.TestMode          = TEST_HAPTIC;
    Setup.TestNumber        = MAX_TEST_NUMBER;
	
    Setup.CurLimitLower 	= 40;
    Setup.CurLimitUpper 	= 90;

    Setup.TestBeepActive 	= 0;
    Setup.LEDType 			= LED_CATHODE; //LED_ANODE;
	
    // Test Count
    Setup.TestSeq 			= 0;
    Setup.SeqTestCount 		= 0;
    Setup.TestCount 		= 0;

#if defined BK2039 || defined BK2039A || defined BK2039_5CH
    // Continue Mode Setup
    Cont.Gen_Func           = GEN_SINE;
    Cont.Gen_Sweep_Mode     = SWEEP_LIN_UP;
    Cont.Gen_Sweep_F1       = 200;
    Cont.Gen_Sweep_F2       = 300;
    Cont.Gen_Sweep_Time     = 2;
    Cont.Gen_Amplitude      = 2;
#elif BK2039X
    // Continue Mode Setup
	i = 0;
    Cont[i].Gen_Func       = GEN_SINE;
    Cont[i].Gen_Sweep_Mode = SWEEP_LIN_UP;
    Cont[i].Gen_Sweep_F1   = 200;
    Cont[i].Gen_Sweep_F2   = 300;
    Cont[i].Gen_Sweep_Time = 2;
    Cont[i].Gen_Amplitude  = 2;                                   
    
	i = 1;
    Cont[i].Gen_Func       = GEN_SINE;
    Cont[i].Gen_Sweep_Mode = SWEEP_LIN_UP;
    Cont[i].Gen_Sweep_F1   = 200;
    Cont[i].Gen_Sweep_F2   = 300;
    Cont[i].Gen_Sweep_Time = 2;
    Cont[i].Gen_Amplitude  = 2;
#endif
    
    // ON-OFF Mode Setup 
    i = 0;
    Seq[i].Active           = 1;
    Seq[i].OnTime           = 2;
    Seq[i].OffTime          = 1;
    Seq[i].RepeatNumber     = 1;
    Seq[i].Gen_Func         = GEN_SINE;    
    Seq[i].Gen_Sine_Freq    = 200;
    Seq[i].Gen_Sweep_Mode   = SWEEP_LIN_UP;
    Seq[i].Gen_Sweep_F1     = 100;
    Seq[i].Gen_Sweep_F2     = 300;
    Seq[i].Gen_Amplitude    = 2;
    Seq[i].Gen_Time         = 2;

    i = 1;
    Seq[i].Active           = 1;
    Seq[i].OnTime           = 2;
    Seq[i].OffTime          = 1;
    Seq[i].RepeatNumber     = 1;
    Seq[i].Gen_Func         = GEN_SWEEP;    
    Seq[i].Gen_Sine_Freq    = 200;
    Seq[i].Gen_Sweep_Mode   = SWEEP_LIN_UP;
    Seq[i].Gen_Sweep_F1     = 100;
    Seq[i].Gen_Sweep_F2     = 300;
    Seq[i].Gen_Amplitude    = 2;
    Seq[i].Gen_Time         = 2;

    i = 2;
    Seq[i].Active           = 1;
    Seq[i].OnTime           = 2;
    Seq[i].OffTime          = 2;
    Seq[i].RepeatNumber     = 1;
    Seq[i].Gen_Func         = GEN_SINE;
    Seq[i].Gen_Sine_Freq    = 200;
    Seq[i].Gen_Sweep_Mode   = SWEEP_LIN_DN;
    Seq[i].Gen_Sweep_F1     = 200;
    Seq[i].Gen_Sweep_F2     = 300;
    Seq[i].Gen_Amplitude    = 2;
    Seq[i].Gen_Time         = 2;

    i = 3;
    Seq[i].Active           = 1;
    Seq[i].OnTime           = 2;
    Seq[i].OffTime          = 2;
    Seq[i].RepeatNumber     = 1;
    Seq[i].Gen_Func         = GEN_SWEEP;
    Seq[i].Gen_Sine_Freq    = 200;
    Seq[i].Gen_Sweep_Mode   = SWEEP_LIN_DN;
    Seq[i].Gen_Sweep_F1     = 200;
    Seq[i].Gen_Sweep_F2     = 300;
    Seq[i].Gen_Amplitude    = 2;
    Seq[i].Gen_Time         = 2;
    
    // 2017-09-08, yjjang, =>
    Init_Cal();
    Init_Cal_Current();
    // 2017-09-08, yjjang, <=
}

void Reset_HapticCount(void)
{
    char key;

    sprintf(VSTR1,"Reset Test Count?");
    (DISPLAY == VFD) ? sprintf(VSTR2,"YES: %c   NO: RUN", 0x17) : sprintf(VSTR2,"YES: ENT   NO: RUN");
    LCDUpdate(VSTR1, VSTR2);
    BuzzerNumber(300, 200, 3);

    while(ReadRegKeyButton() != 0x00) {}
    key = Get_KEY();

    while((key != KEY_ENT) && (key != KEY_RUN))
    {
        key = Get_KEY();
    }

    if(key == KEY_ENT)
    {
        //Haptic Counter ////////////
        HapticCount.Test            = 0;
        HapticCount.SigCurrent      = 0;
        HapticCount.SigRepeatStart  = 0;
        HapticCount.SigRepeat       = 0;

        sprintf(VSTR1, "Reset Test Count");
        sprintf(VSTR2, "Please Wait");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerNumber(300, 200, 3);

        sprintf(VSTR1, "New Count is Saved");
        sprintf(VSTR2, "Push RUN");
        LCDUpdate(VSTR1, VSTR2);

        key = Get_KEY();
        while(key != KEY_RUN)
        {
            key = Get_KEY();
        }
    }
}

void Reset_Count(void)
{
    char key;

    sprintf(VSTR1,"Reset Test Count?");
    (DISPLAY == VFD) ? sprintf(VSTR2,"YES: %c   NO: RUN", 0x17) : sprintf(VSTR2,"YES: ENT   NO: RUN");
    LCDUpdate(VSTR1, VSTR2);
    BuzzerNumber(300, 200, 3);

    while(ReadRegKeyButton() != 0x00) {}
    key = Get_KEY();

    while((key != KEY_ENT) && (key != KEY_RUN))
    {
        key = Get_KEY();
    }

    if(key == KEY_ENT)
    {
        Setup.TestSeq       = 0;
        Setup.SeqTestCount  = 0;
        Setup.TestCount     = 0;

        //Haptic Counter ////////////
        //HapticCount.Test = 0;
        //HapticCount.SigCurrent = 0;
        //HapticCount.SigRepeatStart = 0;
        //HapticCount.SigRepeat = 0;

        sprintf(VSTR1, "Reset Test Count");
        sprintf(VSTR2, "Please Wait");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerNumber(300, 200, 3);

        sprintf(VSTR1, "New Count is Saved");
        sprintf(VSTR2, "Push RUN");
        LCDUpdate(VSTR1, VSTR2);

        key = Get_KEY();
        while(key != KEY_RUN)
        {
            key = Get_KEY();
        }
    }
}

void Init_Result(void)
{
    int i;

    for(i = 0; i < CUR_TEST_CHANNEL; i++)
    {
        Res[i].Cur = 0;
        Res[i].TestCur = CUR_OPN;
    }
    TestResAll = CUR_OPN;
}

void Setup_GeneratorContinue(void)
{
#if defined BK2039 || defined BK2039A || defined BK2039_5CH
    PGENERATOR_SIGNAL_MULTI		pDAC0;
    
    // GEN, DAC 0
    pDAC0 						= GetGenSignalM(0, 0);  
    pDAC0->TimeResolution       = GEN_TIME_RESOLUTION_SIGNAL;
    
    pDAC0->Wave[0].SignalType	= WAVE_SINE;
    pDAC0->Wave[0].Func 		= Cont.Gen_Func;
    pDAC0->Wave[0].Sine_Freq 	= Cont.Gen_Sweep_F1;
    pDAC0->Wave[0].Sweep_Mode 	= Cont.Gen_Sweep_Mode;
    pDAC0->Wave[0].Sweep_F1 	= Cont.Gen_Sweep_F1;
    pDAC0->Wave[0].Sweep_F2 	= Cont.Gen_Sweep_F2;
    pDAC0->Wave[0].Amplitude 	= Cont.Gen_Amplitude;
    pDAC0->Wave[0].Time 		= Cont.Gen_Sweep_Time;            
	
    pDAC0->Wave[1].Time 		= 0;
    pDAC0->Wave[2].Time 		= 0;
    pDAC0->Wave[3].Time 		= 0;
    
    GEN_Calcuration_SignalM(0);
    GEN_SetSignalCycleActive(0);
    
#elif BK2039X
    int                         i = 0;
    PGENERATOR_SIGNAL_MULTI		pDAC0, pDAC1;
    
    // GEN, DAC 0               
    i                           = 0;
    pDAC0 						= GetGenSignalM(i, 0);  
    pDAC0->TimeResolution       = GEN_TIME_RESOLUTION_SIGNAL;
    
    pDAC0->Wave[0].SignalType	= WAVE_SINE;
    pDAC0->Wave[0].Func 		= Cont[i].Gen_Func;
    pDAC0->Wave[0].Sine_Freq 	= Cont[i].Gen_Sweep_F1;
    pDAC0->Wave[0].Sweep_Mode 	= Cont[i].Gen_Sweep_Mode;
    pDAC0->Wave[0].Sweep_F1 	= Cont[i].Gen_Sweep_F1;
    pDAC0->Wave[0].Sweep_F2 	= Cont[i].Gen_Sweep_F2;
    pDAC0->Wave[0].Amplitude 	= Cont[i].Gen_Amplitude;
    pDAC0->Wave[0].Time 		= Cont[i].Gen_Sweep_Time;            
	
    pDAC0->Wave[1].Time 		= 0;
    pDAC0->Wave[2].Time 		= 0;
    pDAC0->Wave[3].Time 		= 0;
    
    GEN_Calcuration_SignalM(i);
    GEN_SetSignalCycleActive(0);
    
    // GEN, DAC 1                                
    i                           = 1;
    pDAC1 						= GetGenSignalM(i, 0);  
    pDAC1->TimeResolution       = GEN_TIME_RESOLUTION_SIGNAL;
    
    pDAC1->Wave[0].SignalType 	= WAVE_SINE;
    pDAC1->Wave[0].Func 		= Cont[i].Gen_Func;
    pDAC1->Wave[0].Sine_Freq 	= Cont[i].Gen_Sweep_F1;
    pDAC1->Wave[0].Sweep_Mode 	= Cont[i].Gen_Sweep_Mode;
    pDAC1->Wave[0].Sweep_F1 	= Cont[i].Gen_Sweep_F1;
    pDAC1->Wave[0].Sweep_F2 	= Cont[i].Gen_Sweep_F2;
    pDAC1->Wave[0].Amplitude 	= Cont[i].Gen_Amplitude;
    pDAC1->Wave[0].Time 		= Cont[i].Gen_Sweep_Time;            
	
    pDAC1->Wave[1].Time 		= 0;
    pDAC1->Wave[2].Time 		= 0;
    pDAC1->Wave[3].Time 		= 0;
    
    GEN_Calcuration_SignalM(i);
    GEN_SetSignalCycleActive(0);
#endif
}

void Setup_GeneratorONOFF(unsigned int nSig)
{
    PGENERATOR_SIGNAL_MULTI         pSignalM;
	
    pSignalM 						= GetGenSignalM(GEN_CH, 0); 
    pSignalM->TimeResolution        = GEN_TIME_RESOLUTION_SIGNAL;
    
    pSignalM->Wave[0].SignalType 	= WAVE_SINE;
    pSignalM->Wave[0].Func 			= Seq[nSig].Gen_Func;
    pSignalM->Wave[0].Sine_Freq 	= Seq[nSig].Gen_Sweep_F1;//Seq[nSig].Gen_Sine_Freq;
    pSignalM->Wave[0].Sweep_Mode 	= Seq[nSig].Gen_Sweep_Mode;
    pSignalM->Wave[0].Sweep_F1 		= Seq[nSig].Gen_Sweep_F1;
    pSignalM->Wave[0].Sweep_F2 		= Seq[nSig].Gen_Sweep_F2;
    pSignalM->Wave[0].Amplitude 	= Seq[nSig].Gen_Amplitude;
    pSignalM->Wave[0].Time 			= Seq[nSig].Gen_Time; 
	
    pSignalM->Wave[1].Time 			= 0;
    pSignalM->Wave[2].Time 			= 0;
    pSignalM->Wave[3].Time 			= 0;

    GEN_Calcuration_SignalM(GEN_CH);
    GEN_SetSignalCycleActive(0);      
}

#define AMP_STANDBY     1
#define AMP_ON          0

void GEN_MUX_ON(void)
{
    REG_AMP_OUT_MUX = AMP_ON;
}
void GEN_MUX_OFF(void)
{
    REG_AMP_OUT_MUX = AMP_ON;
}

void GEN_AMP_STANDBY(void)
{
    REG_AMP_OUT_MUX = AMP_STANDBY;
}

void GEN_AMP_ON(void)
{
    REG_AMP_OUT_MUX = AMP_ON;
}

/////////////////////////////////////////////////////////////////////////////////

void StartTest_Continue(void)
{
    Setup_GeneratorContinue();
    
    GEN_MUX_ON();
    
    GEN_SignalMTrigger(0, 0, GEN_CONTINUE);
    GEN_SignalMTrigger(1, 0, GEN_CONTINUE);
	
    OnMotorPower  = 1;
    OnTestProcess = 1;
	
    delay_ms(100);
	
    CurTestCh = 0;
	
    Init_Result();
	
    RecordStart(CurTestCh);
}

void StartTest_OnOff(void)
{
    int 	snum;
    char 	key;
    //float 	ontime;
    //float 	offtime;
 
    if(Setup.TestCount <= 0) 
    {
        Setup.TestCount = 1;
    }
    else if(Setup.TestCount >= Setup.TestNumber)
    {
        sprintf(VSTR1,"Test is Over: ON_OFF");
        (DISPLAY == VFD) ? sprintf(VSTR2, "Push %c", 0x17) : sprintf(VSTR2, "Push ENT");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerNumber(300, 200, 3);
        key = Get_KEY();
        while(key != KEY_ENT)
        {
            key = Get_KEY();
        }
        Reset_Count();
        return;
    }

    snum = Setup.TestSeq;
        
    //Setup_Generator();
    Setup_GeneratorONOFF(Setup.TestSeq);
    GEN_MUX_ON();
    
    GEN_SignalMTrigger(0, 0, GEN_CONTINUE);
    GEN_SignalMTrigger(1, 0, GEN_CONTINUE);

    SetTimerInterval(TIMER_EVENT_MOTOR_ON , Seq[snum].OnTime+Seq[snum].OffTime);
    SetTimerInterval(TIMER_EVENT_MOTOR_OFF, Seq[snum].OnTime);
    SetTimerActive(TIMER_EVENT_MOTOR_ON , 1);
    SetTimerActive(TIMER_EVENT_MOTOR_OFF, 1);

    OnMotorPower  = 1;
    OnTestProcess = 1;

    delay_ms(100);
    CurTestCh = 0;
    Init_Result();
    RecordStart(CurTestCh);
}

void StartTest_Haptic(void)
{
    char 	key;
    //unsigned int nSig;
    //PHAPTIC_SIGNAL pSig;
   
    if(HapticCount.Test <= 0)
    {
        HapticCount.Test = 0;
    }
    else if(HapticCount.Test >= HapticSetup.TestNumber)
    {
        sprintf(VSTR1,"Haptic Test is Over");
        (DISPLAY == VFD) ? sprintf(VSTR2, "Push %c", 0x17) : sprintf(VSTR2, "Push ENT");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerNumber(300, 200, 3);
        key = Get_KEY();
        while(key != KEY_ENT)
        {
            key = Get_KEY();
        }
        Reset_HapticCount();
        return;
    }
                                 
    HAP_SetGeneratorSignal(HapticCount.SigCurrent);
    HAP_SetSignalRepeatCount(); 
   
    GEN_MUX_ON();
    GEN_SignalMTrigger(GEN_CH, 0, GEN_CONTINUE);  
    
    OnMotorPower  = 1;
    OnTestProcess = 1;
}

void StopTest_Haptic(void)
{
    GEN_SignalMTrigger(GEN_CH, 0, GEN_READY);
    GEN_MUX_OFF();    
}

#define DEBUG_STARTTEST
void StartTest(void)
{
    if(Setup.TestMode == TEST_CONTINUE)
    {
        StartTest_Continue();   
    }
    else if(Setup.TestMode == TEST_ON_OFF)
    {
        StartTest_OnOff();
    } 
    else if(Setup.TestMode == TEST_HAPTIC)
    {
        StartTest_Haptic();  
       
        SetTimerInterval(TIMER_EVENT_FAST_ON,  0.6);
        SetTimerInterval(TIMER_EVENT_FAST_OFF, 0.3);
        SetTimerActive(TIMER_EVENT_FAST_ON,  1);
        SetTimerActive(TIMER_EVENT_FAST_OFF, 1);
    }    
}

#define DEBUG_STOPTEST
void StopTest(void)
{
    if(Setup.TestMode == TEST_CONTINUE)
    {
        GEN_SignalMTrigger(0, 0, GEN_READY);
        GEN_SignalMTrigger(1, 0, GEN_READY);
        GEN_MUX_OFF();
    }
    else if(Setup.TestMode == TEST_ON_OFF)
    {
        SetTimerActive(TIMER_EVENT_MOTOR_ON,  0);
        SetTimerActive(TIMER_EVENT_MOTOR_OFF, 0);

        GEN_SignalMTrigger(0, 0, GEN_READY);
        GEN_SignalMTrigger(1, 0, GEN_READY);
        GEN_MUX_OFF();
    }  
    else if(Setup.TestMode == TEST_HAPTIC)
    {
        StopTest_Haptic();
        SetTimerActive(TIMER_EVENT_FAST_ON,  0);
        SetTimerActive(TIMER_EVENT_FAST_OFF, 0);
        EEP_SaveHapticCount();
    }    

    OnMotorPower  = 0;
    OnTestProcess = 0;
    RecordStop();

    LED(LED_ALL, LED_OFF);
    Init_Result();
}

void Check_SetupMemory(void)
{
    char key,keyV, keyE;

    key = ReadRegKeyButton();
    keyV = key & 0x0F;
    keyE = key & KEY_ACTIVE;

    if((keyV == KEY_SET) && (keyE == KEY_ACTIVE))
    {
        sprintf(VSTR1,"Initilize Setup?");
        (DISPLAY == VFD) ? sprintf(VSTR2,"YES: %c   NO: RUN", 0x17) : sprintf(VSTR2,"YES: ENT   NO: RUN");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerNumber(50, 50, 2);
        while(ReadRegKeyButton() != 0x00) {}

        key = Get_KEY();
        while((key != KEY_ENT) && (key != KEY_RUN))
        {
            key = Get_KEY();
        }

        if(key == KEY_ENT)
        {
            sprintf(VSTR1, "Initialize Setup");
            sprintf(VSTR2, "Please Wait");
            LCDUpdate(VSTR1, VSTR2);
            BuzzerOnOff(500,1000);

            Init_Setup(); 
            Init_SetupHaptic();

            sprintf(VSTR1, "New Setup is Saved");
            sprintf(VSTR2, "Push RUN");
            LCDUpdate(VSTR1, VSTR2);

            key = Get_KEY();
            while(key != KEY_RUN)
            {
                key = Get_KEY();
            }
        }
        while(ReadRegKeyButton() != 0x00) {}
    }
}

void Check_CalMemory(void)
{
    char key,keyV, keyE;

    key = ReadRegKeyButton();
    keyV = key & 0x0F;
    keyE = key & KEY_ACTIVE;


    if((keyV == KEY_ENT) &&  (keyE == KEY_ACTIVE))
    {
        sprintf(VSTR1,"Initilize CAL?");
        (DISPLAY == VFD) ? sprintf(VSTR2,"YES: %c   NO: RUN", 0x17) : sprintf(VSTR2,"YES: ENT   NO: RUN");
        LCDUpdate(VSTR1, VSTR2);
        while(ReadRegKeyButton() != 0x00) {}

        key = Get_KEY();
        while((key != KEY_ENT) && (key != KEY_RUN))
        {
            key = Get_KEY();
        }

        if(key == KEY_ENT)
        {
            sprintf(VSTR1, "Initialize CAL");
            sprintf(VSTR2, "Please Wait");
            LCDUpdate(VSTR1, VSTR2);
            BuzzerOnOff(500,1000);

            Init_Cal();
            Init_Cal_Current();

            sprintf(VSTR1, "New CAL is Saved");
            sprintf(VSTR2, "Push RUN");
            LCDUpdate(VSTR1, VSTR2);

            key = Get_KEY();
            while(key != KEY_RUN)
            {
                key = Get_KEY();
            }
        }
        while(ReadRegKeyButton() != 0x00) {}
    }
}

int Check_SystemInitialRegister(void)
{
    int init;

    init = 0;
    if(SystemInitialRegister == 0)
    {
        sprintf(VSTR1, "Initialize CAL");
        sprintf(VSTR2, "Please Wait");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerOnOff(500,1000);

        Init_Cal();
        Init_Cal_Current();

        sprintf(VSTR1, "Initialize Setup");
        sprintf(VSTR2, "Please Wait");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerOnOff(500,1000);

        Init_Setup();
        Init_SetupHaptic();
        SystemInitialRegister = PROGRAM_VERSION;

        sprintf(VSTR1, "Version Check");
        sprintf(VSTR2, "      Ver %4.2f      ", SystemInitialRegister);
        LCDUpdate(VSTR1, VSTR2);
        BuzzerOnOff(500,1000);

        init = 1;
    }
    if(SystemInitialRegister != PROGRAM_VERSION)
    {
        sprintf(VSTR1, "Initialize Setup");
        sprintf(VSTR2, "Please Wait");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerOnOff(500,1000);

        Init_Setup();   
        Init_SetupHaptic();
        SystemInitialRegister = PROGRAM_VERSION;

        sprintf(VSTR1, "Version Check");
        sprintf(VSTR2, "      Ver %4.2f      ", SystemInitialRegister);
        LCDUpdate(VSTR1, VSTR2);
        BuzzerOnOff(500,1000);

        init = 1;
    }

    return(init);
}

void Check_SystemMemory(void)
{
    Check_SystemInitialRegister();
    Check_CalMemory();
    Check_SetupMemory();  
                          
    EEP_LoadHapticCount();
    EEP_LoadHapticSetup();
}

void Init_Var(void)
{
    int i, j, k;
    PGENERATOR_SIGNAL_MULTI pSignalM;
    PSIGNAL_WAVE 			pWave;

    Init_Result();
    RecordStop();

    OnTestProcess = 0;
    OnDisplayMain = 1;
    OnMotorPower = 0;
    CurTestCh = 0;

    /*for(i = 0; i < CUR_TEST_CHANNEL; ch++)
    {
        Cal.Cur_Offset[ch] = 0;
    }*/

    // Genertor Time Initialize
    for(i = 0; i < GEN_CH_NUM; i++)
    {
        for(j = 0; j < GEN_SIGNAL_NUM; j++)
        {
            for(k = 0; k < SIG_WAVE_NUM; k++)
            {
                pWave = GetGenSignalWave(i, j, k);
                pWave->Time = 0;
            }
            pSignalM = GetGenSignalM(i, j);
            pSignalM->Index = 0;
            pSignalM->Length = 0;
        }
    }

    for(i = 0; i < GEN_CH_NUM; i++)
    {
        GenReal[i].SignalType = GEN_SINE;
        GenReal[i].Frequency  = 175;
        GenReal[i].Amplitude  = 2;
    }

    DebugStatus = DBG_NONE;

    UART_RX_MODE = UART_RX_COMMAND;  
}

int CheckSystemStatus(void)
{
    if((DAQStatus & DAQ_GENOK) != DAQ_GENOK)
    {
        LCDErrorMsg(ERROR_GENRATOR);
        return(0);
    }
    else
    {
        sprintf(VSTR1, "System Ready...");
        sprintf(VSTR2, " ");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerNumber(50, 50, 2);
        return(1);
    }
}

void Intro_System(void)
{
    int i;

    LED(LED_ALL, LED_OFF);

    //12345678901234567890
    //sprintf(VSTR1, " Aging Test ", DEVICE_ID);
#ifdef BK2039
    sprintf(VSTR1, " Aging Test BK2039  ");
#endif
#ifdef  BK2039_5CH
    sprintf(VSTR1, " Aging Test BK2039_5");
#endif
#ifdef  BK2039A
    sprintf(VSTR1, " Aging Test BK2039A ");
#endif
#ifdef  BK2039X
    sprintf(VSTR1, " Aging Test BK2039X ");
#endif

    sprintf(VSTR2, "      Ver %4.2f     ", PROGRAM_VERSION);    
    LCDUpdate(VSTR1, VSTR2);

    BuzzerOnOff(50,0);

    for(i = 0; i < 10; i++)
    {
        LED(i, LED_RED);
        LED(i+10, LED_RED);
        delay_ms(100);
    }

    LED(LED_ALL, LED_OFF);

    sprintf(VSTR2, "     %04d.%02d.%02d     ", PROGRAM_YEAR, PROGRAM_MONTH, PROGRAM_DAY);
    LCDUpdate(VSTR1, VSTR2);

    BuzzerOnOff(50,0);

    for(i = 0; i < 10; i++)
    {
        LED(i, LED_GREEN);
        LED(i+10, LED_GREEN);
        delay_ms(100);
    }

    LED(LED_ALL, LED_OFF);
}

int Set_Mode(char set_val, char set_pos, int *CurMode, int *OpMode, int *OpModeSub)
{
    static char     premode = 0;
    static int 	    cur_xpos;
    static int 	    seq_num = 0;
    static int      nSig = 0;
    static int      nWave = 0;
    
    PHAPTIC_SIGNAL  pSig;
    PHAPTIC_WAVE    pWave;
    
    int     	    dot_pos;
    int     	    num_pos;
    int     	    OnCursor;     
    char    	    lstr[10], lstr2[10];
	
	int				i = 0;

    switch(*OpMode)
    {
    case MODE_SYS_LED_TYPE:
        if(premode != *OpMode)
        {
            cur_xpos = 6;
            seq_num = 0;
        }
        if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            num_pos = 1;
            Setup.LEDType = (int)Calc_Number(Setup.LEDType, num_pos, set_val);
            if		(Setup.LEDType > LED_CATHODE)   Setup.LEDType = LED_CATHODE;
            else if	(Setup.LEDType < LED_ANODE)	    Setup.LEDType = LED_ANODE;
        }
        LED(LED_ALL, LED_GREEN);

        sprintf(VSTR1,"SYS> LED TYPE");
        if(Setup.LEDType == LED_ANODE)
        {
            sprintf(VSTR2,"MODE:Anode   [Green]");
        }
        else if(Setup.LEDType == LED_CATHODE)
        {
            sprintf(VSTR2,"MODE:Cathode [Green]");
        }

        if(*CurMode == CUR_MODE_CURSOR) *CurMode = CUR_MODE_VALUE;
        OnCursor = *CurMode;
        break;

    case MODE_SET_TEST_MODE:
        if(premode != *OpMode)
        {
            cur_xpos = 7;
            seq_num = 0;
        }

        if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            num_pos = 1;
            Setup.TestMode = (int)Calc_Number(Setup.TestMode, num_pos, set_val);
            if	    (Setup.TestMode < TEST_CONTINUE)	Setup.TestMode = TEST_CONTINUE;
            else if (Setup.TestMode > TEST_HAPTIC) 		Setup.TestMode = TEST_HAPTIC;
        }

        sprintf(VSTR1,"SET> TEST MODE");

        if(Setup.TestMode == TEST_CONTINUE)
        {
            sprintf(VSTR2,"MODE: CONTINUE");
        }
        else if(Setup.TestMode == TEST_ON_OFF)
        {
            sprintf(VSTR2,"MODE: ON-OFF");
        }
        else if(Setup.TestMode == TEST_HAPTIC)
        {
            sprintf(VSTR2,"MODE: HAPTIC");
        }        

        if(*CurMode == CUR_MODE_CURSOR) *CurMode = CUR_MODE_VALUE;

        OnCursor = *CurMode;
        break;        
        
// MODE_SET_HAPTIC /////////////////////////////////////////////
    case MODE_SET_HAPTIC_TEST_NUMBER:
        if(premode != *OpMode)
        {
            cur_xpos = 10;
        }
        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if(cur_xpos > 11) cur_xpos = 11;
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if(cur_xpos < 5) cur_xpos = 5;
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if((cur_xpos >= 5) && (cur_xpos <= 11))
            {
                dot_pos = 12;
                num_pos = dot_pos - cur_xpos;
                HapticSetup.TestNumber = (long)Calc_Number(HapticSetup.TestNumber, num_pos, set_val);
                
                if		(HapticSetup.TestNumber > MAX_TEST_NUMBER) HapticSetup.TestNumber = MAX_TEST_NUMBER;
                else if	(HapticSetup.TestNumber < 1) HapticSetup.TestNumber = 1;
                
            }
        }
        //12345678901234567890
        //HAPTIC> TEST NUMBER
        //NUM:9999999
        sprintf(VSTR1,"HAPTIC> TEST NUMBER");
        ltoa(HapticSetup.TestNumber, lstr);
        if     (HapticSetup.TestNumber > 999999)  sprintf(VSTR2,"NUM:%s", lstr);
        else if(HapticSetup.TestNumber >  99999)  sprintf(VSTR2,"NUM: %s", lstr);
        else if(HapticSetup.TestNumber >   9999)  sprintf(VSTR2,"NUM:  %s", lstr);
        else if(HapticSetup.TestNumber >    999)  sprintf(VSTR2,"NUM:   %s", lstr);
        else if(HapticSetup.TestNumber >     99)  sprintf(VSTR2,"NUM:    %s", lstr);
        else if(HapticSetup.TestNumber >      9)  sprintf(VSTR2,"NUM:     %s", lstr);
        else                                      sprintf(VSTR2,"NUM:      %s", lstr);
        OnCursor = *CurMode;
        break;                      

    case MODE_SET_HAPTIC_SIG_NUMBER(0):
    case MODE_SET_HAPTIC_SIG_NUMBER(1):
    case MODE_SET_HAPTIC_SIG_NUMBER(2):
    case MODE_SET_HAPTIC_SIG_NUMBER(3): 
    case MODE_SET_HAPTIC_SIG_NUMBER(4):
    case MODE_SET_HAPTIC_SIG_NUMBER(5):
        if(premode != *OpMode)
        {
            cur_xpos    = 8;         
            nSig        = (int)((*OpMode - MODE_SET_HAPTIC_TEST_NUMBER - 1) / MODE_HAPTIC_SIG_LENGTH);
            nWave       = (int)((*OpMode - MODE_SET_HAPTIC_TEST_NUMBER - 1) % MODE_HAPTIC_SIG_LENGTH) - 1;                           
        }
        
        pSig  = GetHapticSignal(nSig);
        pWave = GetHapticSigWave(nSig, nWave);
        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if      (cur_xpos >  13) cur_xpos = 13;
            else if (cur_xpos == 10) cur_xpos = 13;            
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if      (cur_xpos <   5) cur_xpos = 5;
            else if (cur_xpos == 12) cur_xpos = 9; 
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if((cur_xpos >= 5) && (cur_xpos <= 9))
            {
                dot_pos = 10;
                num_pos = dot_pos - cur_xpos;
                pSig->RepeatNumber = (long)Calc_Number(pSig->RepeatNumber, num_pos, set_val);
                
                if		(pSig->RepeatNumber > MAX_SIGNAL_REPEAT_NUMBER) pSig->RepeatNumber = MAX_SIGNAL_REPEAT_NUMBER;
                else if	(pSig->RepeatNumber < 1) pSig->RepeatNumber = 1;
            }
            else if((cur_xpos == 13))
            {
                if(nSig == 0)
                {
                    pSig->Active = 1;
                }
                else 
                {
                    num_pos = 1;
                    pSig->Active = (int)Calc_Number(pSig->Active, num_pos, set_val);
                    if	    (pSig->Active < 0)	pSig->Active = 0;
                    else if (pSig->Active > 1) 	pSig->Active = 1;
                }    
            }
        }
        
        ltoa(pSig->RepeatNumber, lstr);      
        if(pSig->Active == 1) sprintf(lstr2,"ON");
        else  sprintf(lstr2,"OFF");

        //12345678901234567890
        //HAP1> REPEAT/ONOFF
        //NUM:60000 / OFF
        sprintf(VSTR1,"HAP%d> REPEAT/ONOFF",nSig+1);
        if     (pSig->RepeatNumber > 9999) sprintf(VSTR2,"NUM:%s / %s", lstr, lstr2);
        else if(pSig->RepeatNumber >  999) sprintf(VSTR2,"NUM: %s / %s", lstr, lstr2);
        else if(pSig->RepeatNumber >   99) sprintf(VSTR2,"NUM:  %s / %s", lstr, lstr2);
        else if(pSig->RepeatNumber >    9) sprintf(VSTR2,"NUM:   %s / %s", lstr, lstr2);
        else                               sprintf(VSTR2,"NUM:    %s / %s", lstr, lstr2);
        OnCursor = *CurMode;

        break;    
    
    case MODE_SET_HAPTIC_SIG_WAVE(0, 0): 
    case MODE_SET_HAPTIC_SIG_WAVE(0, 1): 
    case MODE_SET_HAPTIC_SIG_WAVE(0, 2):
    case MODE_SET_HAPTIC_SIG_WAVE(0, 3):
    case MODE_SET_HAPTIC_SIG_WAVE(0, 4):
    case MODE_SET_HAPTIC_SIG_WAVE(0, 5):
    case MODE_SET_HAPTIC_SIG_WAVE(0, 6):    
    case MODE_SET_HAPTIC_SIG_WAVE(0, 7):    
    case MODE_SET_HAPTIC_SIG_WAVE(0, 8):    
    case MODE_SET_HAPTIC_SIG_WAVE(0, 9):    
    case MODE_SET_HAPTIC_SIG_WAVE(0, 10):    
     
    case MODE_SET_HAPTIC_SIG_WAVE(1, 0): 
    case MODE_SET_HAPTIC_SIG_WAVE(1, 1): 
    case MODE_SET_HAPTIC_SIG_WAVE(1, 2): 
    case MODE_SET_HAPTIC_SIG_WAVE(1, 3):
    case MODE_SET_HAPTIC_SIG_WAVE(1, 4):
    case MODE_SET_HAPTIC_SIG_WAVE(1, 5):
    case MODE_SET_HAPTIC_SIG_WAVE(1, 6):    
    case MODE_SET_HAPTIC_SIG_WAVE(1, 7):    
    case MODE_SET_HAPTIC_SIG_WAVE(1, 8):    
    case MODE_SET_HAPTIC_SIG_WAVE(1, 9):    
    case MODE_SET_HAPTIC_SIG_WAVE(1, 10):    

    case MODE_SET_HAPTIC_SIG_WAVE(2, 0): 
    case MODE_SET_HAPTIC_SIG_WAVE(2, 1): 
    case MODE_SET_HAPTIC_SIG_WAVE(2, 2): 
    case MODE_SET_HAPTIC_SIG_WAVE(2, 3):
    case MODE_SET_HAPTIC_SIG_WAVE(2, 4):
    case MODE_SET_HAPTIC_SIG_WAVE(2, 5):
    case MODE_SET_HAPTIC_SIG_WAVE(2, 6):    
    case MODE_SET_HAPTIC_SIG_WAVE(2, 7):    
    case MODE_SET_HAPTIC_SIG_WAVE(2, 8):    
    case MODE_SET_HAPTIC_SIG_WAVE(2, 9):    
    case MODE_SET_HAPTIC_SIG_WAVE(2, 10):    

    case MODE_SET_HAPTIC_SIG_WAVE(3, 0): 
    case MODE_SET_HAPTIC_SIG_WAVE(3, 1): 
    case MODE_SET_HAPTIC_SIG_WAVE(3, 2): 
    case MODE_SET_HAPTIC_SIG_WAVE(3, 3):
    case MODE_SET_HAPTIC_SIG_WAVE(3, 4):
    case MODE_SET_HAPTIC_SIG_WAVE(3, 5):
    case MODE_SET_HAPTIC_SIG_WAVE(3, 6):    
    case MODE_SET_HAPTIC_SIG_WAVE(3, 7):    
    case MODE_SET_HAPTIC_SIG_WAVE(3, 8):    
    case MODE_SET_HAPTIC_SIG_WAVE(3, 9):    
    case MODE_SET_HAPTIC_SIG_WAVE(3, 10):    

    case MODE_SET_HAPTIC_SIG_WAVE(4, 0): 
    case MODE_SET_HAPTIC_SIG_WAVE(4, 1): 
    case MODE_SET_HAPTIC_SIG_WAVE(4, 2): 
    case MODE_SET_HAPTIC_SIG_WAVE(4, 3):
    case MODE_SET_HAPTIC_SIG_WAVE(4, 4):
    case MODE_SET_HAPTIC_SIG_WAVE(4, 5):
    case MODE_SET_HAPTIC_SIG_WAVE(4, 6):    
    case MODE_SET_HAPTIC_SIG_WAVE(4, 7):    
    case MODE_SET_HAPTIC_SIG_WAVE(4, 8):    
    case MODE_SET_HAPTIC_SIG_WAVE(4, 9):    
    case MODE_SET_HAPTIC_SIG_WAVE(4, 10):    

    /*
    case MODE_SET_HAPTIC_SIG_WAVE(5, 0): 
    case MODE_SET_HAPTIC_SIG_WAVE(5, 1): 
    case MODE_SET_HAPTIC_SIG_WAVE(5, 2): 
    case MODE_SET_HAPTIC_SIG_WAVE(5, 3):
    case MODE_SET_HAPTIC_SIG_WAVE(5, 4):
    case MODE_SET_HAPTIC_SIG_WAVE(5, 5):
    case MODE_SET_HAPTIC_SIG_WAVE(5, 6):    
    case MODE_SET_HAPTIC_SIG_WAVE(5, 7):    
    case MODE_SET_HAPTIC_SIG_WAVE(5, 8):    
    case MODE_SET_HAPTIC_SIG_WAVE(5, 9):
    */    
        if(premode != *OpMode)
        {
            cur_xpos = 4;         
            nSig  = (int)((*OpMode - MODE_SET_HAPTIC_TEST_NUMBER - 1) / MODE_HAPTIC_SIG_LENGTH);
            nWave = (int)((*OpMode - MODE_SET_HAPTIC_TEST_NUMBER - 1) % MODE_HAPTIC_SIG_LENGTH) - 1;                           
        }
        
        //pSig  = GetHapticSignal(nSig);
        pWave = GetHapticSigWave(nSig, nWave);
        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if      (cur_xpos >  19) cur_xpos = 19;
            else if (cur_xpos == 12) cur_xpos = 17; 
            else if (cur_xpos ==  7) cur_xpos = 8;           
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if      (cur_xpos <   3) cur_xpos = 3;
            else if (cur_xpos ==  7) cur_xpos = 6; 
            else if (cur_xpos == 16) cur_xpos = 11;
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if((cur_xpos >= 3) && (cur_xpos <= 6))
            {
                dot_pos = 2;
                num_pos = dot_pos - cur_xpos;
                pWave->Time = (float)Calc_Number(pWave->Time, num_pos, set_val); 
                if(cur_xpos == 6)
                { 
                    pWave->Time = (float)Calc_Number(pWave->Time, num_pos, set_val);  
                }                              
                if		(pWave->Time > GEN_MAX_TIME_WAVE)   pWave->Time = GEN_MAX_TIME_WAVE;
                else if	(pWave->Time < 0.000)               pWave->Time = 0.000;
            }
            else if((cur_xpos >= 8) && (cur_xpos <= 11))
            {
                dot_pos = 12;
                num_pos = dot_pos - cur_xpos;
                pWave->Freq = (float)Calc_Number(pWave->Freq, num_pos, set_val);                
                if		(pWave->Freq > 9999) pWave->Freq = 9999;
                else if	(pWave->Freq < 10)   pWave->Freq = 10;
            }
            else if((cur_xpos >= 17) && (cur_xpos <= 19))
            {
                dot_pos = 18;
                num_pos = dot_pos - cur_xpos;
                pWave->Level = (float)Calc_Number_NEG(pWave->Level, num_pos, set_val);                
                if		(pWave->Level >  GEN_VOLT_MAX) pWave->Level =  GEN_VOLT_MAX;
                else if	(pWave->Level < -GEN_VOLT_MAX) pWave->Level = -GEN_VOLT_MAX;
            }
        }
         
        //12345678901234567890
        //HAP1-WAV1> T/F/V
        //1.0000/9999Hz/ 10.0V
        sprintf(VSTR1,"HAP%d-WAV%02d> T/F/V", nSig + 1, nWave + 1);
        sprintf(VSTR2,"%6.4f/%4.0fHz/%5.1fV", pWave->Time, pWave->Freq, pWave->Level);
        OnCursor = *CurMode;
        break;

////////////////////////////////////////////////////////////////////////
    case MODE_SET_TEST_NUMBER:        
        if(premode != *OpMode)
        {
            cur_xpos = 7;
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if(cur_xpos > 11) 	cur_xpos = 11;
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if(cur_xpos < 5)	cur_xpos = 5;
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if((cur_xpos >= 5) && (cur_xpos <= 11))
            {
                dot_pos = 12;
                num_pos = dot_pos - cur_xpos;
                Setup.TestNumber = (long)Calc_Number(Setup.TestNumber, num_pos, set_val);
                if		(Setup.TestNumber > 9999999) 	Setup.TestNumber = 9999999;
                else if	(Setup.TestNumber < 1) 			Setup.TestNumber = 1;
            }
        }

        //12345678901234567890
        //NUM:1234567
        sprintf(VSTR1,"SET> TEST NUMBER");
        ltoa(Setup.TestNumber, lstr);
        if     (Setup.TestNumber > 999999)  sprintf(VSTR2,"NUM:%s", lstr);
        else if(Setup.TestNumber >  99999)  sprintf(VSTR2,"NUM: %s", lstr);
        else if(Setup.TestNumber >   9999)  sprintf(VSTR2,"NUM:  %s", lstr);
        else if(Setup.TestNumber >    999)  sprintf(VSTR2,"NUM:   %s", lstr);
        else if(Setup.TestNumber >     99)  sprintf(VSTR2,"NUM:    %s", lstr);
        else if(Setup.TestNumber >      9)  sprintf(VSTR2,"NUM:     %s", lstr);
        else                                sprintf(VSTR2,"NUM:      %s", lstr);
        //sprintf(VSTR2,"NUM:%3d%04d", (unsigned int)(Setup.TestNumber/10000), (unsigned int)(Setup.TestNumber-(Setup.TestNumber/10000)));

        OnCursor = *CurMode;
        break;

    case MODE_SET_TEST_CUR_LIMIT:
    case MODE_SET_CONT_CUR_LIMIT:
        if(premode != *OpMode)
        {
            cur_xpos = 5;
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if      (cur_xpos > 15) cur_xpos = 15;
            else if (cur_xpos == 6) cur_xpos = 13;
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if(cur_xpos < 3)
            {
                cur_xpos = 3;
            }
            else if(cur_xpos == 12) cur_xpos = 5;
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if((cur_xpos >= 3) && (cur_xpos <= 5))
            {               
                dot_pos = 6;
                num_pos = dot_pos - cur_xpos;
                Setup.CurLimitLower = (int)Calc_Number(Setup.CurLimitLower, num_pos, set_val);               
                if		(Setup.CurLimitLower > Setup.CurLimitUpper) Setup.CurLimitLower = Setup.CurLimitUpper;
                else if	(Setup.CurLimitLower < CUR_OPEN_LIMIT)		Setup.CurLimitLower = 0;
            }
            else if((cur_xpos >= 13) && (cur_xpos <= 15))
            {              
                dot_pos = 16;
                num_pos = dot_pos - cur_xpos;
                Setup.CurLimitUpper = (int)Calc_Number(Setup.CurLimitUpper, num_pos, set_val);
                if		(Setup.CurLimitUpper > CUR_SHORT) 			Setup.CurLimitUpper = CUR_SHORT;
                else if	(Setup.CurLimitUpper < Setup.CurLimitLower) Setup.CurLimitUpper = Setup.CurLimitLower;
            }
        }
        sprintf(VSTR1, "SET> CURRENT LIMIT");     
        sprintf(VSTR2, "L:%3dmA   H:%3dmA", (int)Setup.CurLimitLower, (int)Setup.CurLimitUpper);

        OnCursor = *CurMode;
        break;

#ifdef BK2039X
    case MODE_SET_CONT_CUR_LIMIT_B:
        if(premode != *OpMode)
        {
            cur_xpos = 5;
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if      (cur_xpos > 15) cur_xpos = 15;
            else if (cur_xpos == 6) cur_xpos = 13;
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if(cur_xpos < 3)
            {
                cur_xpos = 3;
            }
            else if(cur_xpos == 12) cur_xpos = 5;
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if((cur_xpos >= 3) && (cur_xpos <= 5))
            {               
                dot_pos = 6;
                num_pos = dot_pos - cur_xpos;
                Setup.CurLimitLower = (int)Calc_Number(Setup.CurLimitLower, num_pos, set_val);               
                if		(Setup.CurLimitLower > Setup.CurLimitUpper) Setup.CurLimitLower = Setup.CurLimitUpper;
                else if	(Setup.CurLimitLower < CUR_OPEN_LIMIT)		Setup.CurLimitLower = 0;
            }
            else if((cur_xpos >= 13) && (cur_xpos <= 15))
            {              
                dot_pos = 16;
                num_pos = dot_pos - cur_xpos;
                Setup.CurLimitUpper = (int)Calc_Number(Setup.CurLimitUpper, num_pos, set_val);
                if		(Setup.CurLimitUpper > CUR_SHORT) 			Setup.CurLimitUpper = CUR_SHORT;
                else if	(Setup.CurLimitUpper < Setup.CurLimitLower) Setup.CurLimitUpper = Setup.CurLimitLower;
            }
        }
        sprintf(VSTR1, "SET_B> CURRENT LIMIT");     
        sprintf(VSTR2, "L:%3dmA   H:%3dmA", (int)Setup.CurLimitLower, (int)Setup.CurLimitUpper);

        OnCursor = *CurMode;
        break;
#endif

    ///////////////////////////////////////////////////////////////////////////////////////////
    case MODE_SET_SEQ_REPEAT_ACTIVE(0):
    case MODE_SET_SEQ_REPEAT_ACTIVE(1):
    case MODE_SET_SEQ_REPEAT_ACTIVE(2):
    case MODE_SET_SEQ_REPEAT_ACTIVE(3):
        if(premode != *OpMode)
        {
            cur_xpos = 12;
            seq_num  = (int)((*OpMode - MODE_SET_SEQ_TEST) / SEQ_ITEM_NUMBER);
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if(cur_xpos > 16)  cur_xpos = 16;
            else if(cur_xpos == 13)  cur_xpos = 16;
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if(cur_xpos < 8) cur_xpos = 8; 
            else if(cur_xpos == 15) cur_xpos = 12; 
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if((cur_xpos >= 8) && (cur_xpos <= 12))
            {
                dot_pos = 13;
                num_pos = dot_pos - cur_xpos;
                Seq[seq_num].RepeatNumber = (long)Calc_Number(Seq[seq_num].RepeatNumber, num_pos, set_val);
                if      (Seq[seq_num].RepeatNumber > MAX_SIGNAL_REPEAT_NUMBER)  Seq[seq_num].RepeatNumber = MAX_SIGNAL_REPEAT_NUMBER;
                else if (Seq[seq_num].RepeatNumber < 1)                         Seq[seq_num].RepeatNumber = 1;
            }
            else if(cur_xpos == 16)
            {
                num_pos = 1;
                Seq[seq_num].Active = (int)Calc_Number(Seq[seq_num].Active, num_pos, set_val);
                if      (Seq[seq_num].Active > 1) Seq[seq_num].Active = 1;
                else if (Seq[seq_num].Active < 0) seq_num = 0;
            }
        }
        
        ltoa(Seq[seq_num].RepeatNumber, lstr);      
        if(Seq[seq_num].Active == 1) sprintf(lstr2,"ON");
        else  sprintf(lstr2,"OFF");

        //12345678901234567890
        //SEQ1> REPEAT / ONOFF 
        //NUM:   60000 / OFF
        sprintf(VSTR1,"SEQ%d> REPEAT / ONOFF", seq_num+1);        
        if     (Seq[seq_num].RepeatNumber > 9999) sprintf(VSTR2,"NUM:   %s / %s", lstr, lstr2);
        else if(Seq[seq_num].RepeatNumber >  999) sprintf(VSTR2,"NUM:    %s / %s", lstr, lstr2);
        else if(Seq[seq_num].RepeatNumber >   99) sprintf(VSTR2,"NUM:     %s / %s", lstr, lstr2);
        else if(Seq[seq_num].RepeatNumber >    9) sprintf(VSTR2,"NUM:      %s / %s", lstr, lstr2);
        else                                      sprintf(VSTR2,"NUM:       %s / %s", lstr, lstr2);
        OnCursor = *CurMode;

        break;
        
    case MODE_SET_SEQ_ONOFF_TIME(0):
    case MODE_SET_SEQ_ONOFF_TIME(1):
    case MODE_SET_SEQ_ONOFF_TIME(2):
    case MODE_SET_SEQ_ONOFF_TIME(3):
        if(premode != *OpMode)
        {
            cur_xpos = 7;
            seq_num  = (int)((*OpMode - MODE_SET_SEQ_TEST) / SEQ_ITEM_NUMBER);
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if		(cur_xpos >  17) cur_xpos = 17;
            else if	(cur_xpos ==  8) cur_xpos = 13;
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if		(cur_xpos <   3)  cur_xpos = 3;
            else if	(cur_xpos == 12)  cur_xpos = 7;
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if((cur_xpos >= 3) && (cur_xpos <= 7))
            {
                dot_pos = 6;
                num_pos = dot_pos - cur_xpos;
                Seq[seq_num].OnTime = (float)Calc_Number(Seq[seq_num].OnTime, num_pos, set_val);
                if      (Seq[seq_num].OnTime > 3600)    Seq[seq_num].OnTime = 3600;
                else if (Seq[seq_num].OnTime < 0.5)     Seq[seq_num].OnTime = 0.5; 
            } 
            else if((cur_xpos >= 13) && (cur_xpos <= 17)) 
            {
                dot_pos = 16;
                num_pos = dot_pos - cur_xpos;
                Seq[seq_num].OffTime = (float)Calc_Number(Seq[seq_num].OffTime, num_pos, set_val);
                if      (Seq[seq_num].OffTime > 3600)   Seq[seq_num].OffTime = 3600;
                else if (Seq[seq_num].OffTime < 0.5)    Seq[seq_num].OffTime = 0.5; 
            }
            
            if(Seq[seq_num].OnTime > GEN_MAX_TIME_SIGNAL) 
                Seq[seq_num].Gen_Time = GEN_MAX_TIME_SIGNAL;
            else
                Seq[seq_num].Gen_Time = Seq[seq_num].OnTime; 
        }                                         
                                           
        //12345678901234567890
        //SEQ1> ON/OFF TIME
        // 3600.1s / 3600.1s
        sprintf(VSTR1,"SEQ%d> ON/OFF TIME", seq_num + 1);                                       
        sprintf(VSTR2," %6.1fs / %6.1fs ", Seq[seq_num].OnTime, Seq[seq_num].OffTime);
        OnCursor = *CurMode;                                      
        break;

    case MODE_SET_SEQ_SIGNAL_FUNC(0):
    case MODE_SET_SEQ_SIGNAL_FUNC(1):
    case MODE_SET_SEQ_SIGNAL_FUNC(2):
    case MODE_SET_SEQ_SIGNAL_FUNC(3):        
        if(premode != *OpMode)
        {
            cur_xpos = 7;
            seq_num  = (int)((*OpMode - MODE_SET_SEQ_TEST) / SEQ_ITEM_NUMBER);
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++; 
            if(Seq[seq_num].Gen_Func == GEN_SWEEP)
            {
                if(cur_xpos > 11)       cur_xpos = 11;
                else if(cur_xpos == 8)  cur_xpos = 11;
            }
            else
            {
                if(cur_xpos > 7)  cur_xpos = 7;
            }
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if(Seq[seq_num].Gen_Func == GEN_SWEEP)
            {
                if(cur_xpos < 7)        cur_xpos = 7; 
                else if(cur_xpos == 10) cur_xpos = 7;
            }
            else
            {
                if(cur_xpos < 7)        cur_xpos = 7;
            } 
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if(cur_xpos == 7)
            {
                num_pos = 1;
                Seq[seq_num].Gen_Func = (int)Calc_Number(Seq[seq_num].Gen_Func, num_pos, set_val);
                if(Seq[seq_num].Gen_Func > GEN_SWEEP) Seq[seq_num].Gen_Func = GEN_SWEEP;
                else if(Seq[seq_num].Gen_Func < GEN_SINE) Seq[seq_num].Gen_Func = GEN_SINE;
            }
            else if(cur_xpos == 11)
            {
                num_pos = 1;
                Seq[seq_num].Gen_Sweep_Mode = (int)Calc_Number(Seq[seq_num].Gen_Sweep_Mode, num_pos, set_val);
                if(Seq[seq_num].Gen_Sweep_Mode > SWEEP_LOG_UP_DN) Seq[seq_num].Gen_Sweep_Mode = SWEEP_LOG_UP_DN;
                else if(Seq[seq_num].Gen_Sweep_Mode < SWEEP_LIN_UP) Seq[seq_num].Gen_Sweep_Mode = SWEEP_LIN_UP;
                else if(Seq[seq_num].Gen_Sweep_Mode == SWEEP_LOG_DN+1) Seq[seq_num].Gen_Sweep_Mode = SWEEP_LIN_UP_DN;
                else if(Seq[seq_num].Gen_Sweep_Mode == SWEEP_LIN_UP_DN-1) Seq[seq_num].Gen_Sweep_Mode = SWEEP_LOG_DN;
            }
        }

        if(Seq[seq_num].Gen_Func == GEN_SWEEP) sprintf(lstr, "  SWEEP");
        else sprintf(lstr, "   SINE");
        
        if(Seq[seq_num].Gen_Sweep_Mode == SWEEP_LIN_UP)         sprintf(lstr2, "LIN UP");
        else if(Seq[seq_num].Gen_Sweep_Mode == SWEEP_LIN_DN)    sprintf(lstr2, "LIN DN");
        else if(Seq[seq_num].Gen_Sweep_Mode == SWEEP_LIN_UP_DN) sprintf(lstr2, "LIN UP-DN");
        else if(Seq[seq_num].Gen_Sweep_Mode == SWEEP_LOG_UP)    sprintf(lstr2, "LOG UP");
        else if(Seq[seq_num].Gen_Sweep_Mode == SWEEP_LOG_DN)    sprintf(lstr2, "LOG DN");
        else if(Seq[seq_num].Gen_Sweep_Mode == SWEEP_LOG_UP_DN) sprintf(lstr2, "LOG UP-DN");

        //12345678901234567890
        //SEQ1> FUNC / MODE  
        //   SINE / 
        //  SWEEP / LIN UP-DN
        sprintf(VSTR1,"SEQ%d> FUNC / MODE", seq_num+1);  
        if(Seq[seq_num].Gen_Func == GEN_SWEEP)               
            sprintf(VSTR2,"%s / %s", lstr, lstr2);
        else
            sprintf(VSTR2,"%s", lstr);

        OnCursor = *CurMode;
        break;
    
    case MODE_SET_SEQ_FREQ_LEVEL(0):
    case MODE_SET_SEQ_FREQ_LEVEL(1):
    case MODE_SET_SEQ_FREQ_LEVEL(2):
    case MODE_SET_SEQ_FREQ_LEVEL(3):
        if(premode != *OpMode)
        {
            cur_xpos = 4;
            seq_num  = (int)((*OpMode - MODE_SET_SEQ_TEST) / SEQ_ITEM_NUMBER);
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if(Seq[seq_num].Gen_Func == GEN_SWEEP)
            {
                if		(cur_xpos >  19)	cur_xpos = 19;
                else if	(cur_xpos == 12)	cur_xpos = 17;
                else if	(cur_xpos ==  6)	cur_xpos =  8;
            }
            else
            {
                if		(cur_xpos >  19)	cur_xpos = 19;
                else if	(cur_xpos ==  6)	cur_xpos = 17;
            }
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if(Seq[seq_num].Gen_Func == GEN_SWEEP)
            {
                if		(cur_xpos <   2)	cur_xpos =  2;
                else if	(cur_xpos ==  7)	cur_xpos =  5;
                else if	(cur_xpos == 16)	cur_xpos = 11;
            }
            else
            {
                if		(cur_xpos <   2)	cur_xpos =  2;
                else if	(cur_xpos == 16)	cur_xpos =  5;
            }
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            
            if(Seq[seq_num].Gen_Func == GEN_SWEEP)
            {
                if((cur_xpos >= 2) && (cur_xpos <= 5))
                {
                    dot_pos = 6;
                    num_pos = dot_pos - cur_xpos;
                    Seq[seq_num].Gen_Sweep_F1 = (float)Calc_Number(Seq[seq_num].Gen_Sweep_F1, num_pos, set_val);
                    if		(Seq[seq_num].Gen_Sweep_F1 > Seq[seq_num].Gen_Sweep_F2) Seq[seq_num].Gen_Sweep_F1 = Seq[seq_num].Gen_Sweep_F2;
                    else if	(Seq[seq_num].Gen_Sweep_F1 < MIN_FRQ) 					Seq[seq_num].Gen_Sweep_F1 = MIN_FRQ;
                }
                else if((cur_xpos >= 8) && (cur_xpos <= 11))
                {
                    dot_pos = 12;
                    num_pos = dot_pos - cur_xpos;
                    Seq[seq_num].Gen_Sweep_F2 = (float)Calc_Number(Seq[seq_num].Gen_Sweep_F2, num_pos, set_val);
                    if		(Seq[seq_num].Gen_Sweep_F2 > MAX_FRQ) 					Seq[seq_num].Gen_Sweep_F2 = MAX_FRQ;
                    else if	(Seq[seq_num].Gen_Sweep_F2 < Seq[seq_num].Gen_Sweep_F1) Seq[seq_num].Gen_Sweep_F2 = Seq[seq_num].Gen_Sweep_F1;
                }
                if((cur_xpos >= 17) && (cur_xpos <= 19))
                {
                    dot_pos = 18;
                    num_pos = dot_pos - cur_xpos;
                    Seq[seq_num].Gen_Amplitude = (float)Calc_Number(Seq[seq_num].Gen_Amplitude, num_pos, set_val);
                    if(Seq[seq_num].Gen_Amplitude > GEN_VOLT_MAX) Seq[seq_num].Gen_Amplitude = GEN_VOLT_MAX;
                    else if(Seq[seq_num].Gen_Amplitude < 0.0) Seq[seq_num].Gen_Amplitude = 0.0;
                }
            }
            else
            {
                if((cur_xpos >= 1) && (cur_xpos <= 5))
                {
                    dot_pos = 6;
                    num_pos = dot_pos - cur_xpos;
                    Seq[seq_num].Gen_Sweep_F1 = (float)Calc_Number(Seq[seq_num].Gen_Sweep_F1, num_pos, set_val);
                    if		(Seq[seq_num].Gen_Sweep_F1 > MAX_FRQ) Seq[seq_num].Gen_Sweep_F1 = MAX_FRQ;
                    else if	(Seq[seq_num].Gen_Sweep_F1 < MIN_FRQ) Seq[seq_num].Gen_Sweep_F1 = MIN_FRQ;
                }
                if((cur_xpos >= 17) && (cur_xpos <= 19))
                {
                    dot_pos = 18;
                    num_pos = dot_pos - cur_xpos;
                    Seq[seq_num].Gen_Amplitude = (float)Calc_Number(Seq[seq_num].Gen_Amplitude, num_pos, set_val);
                    if      (Seq[seq_num].Gen_Amplitude > GEN_VOLT_MAX) Seq[seq_num].Gen_Amplitude = GEN_VOLT_MAX;
                    else if (Seq[seq_num].Gen_Amplitude < 0.0)          Seq[seq_num].Gen_Amplitude = 0.0;
                }
            }
        }

        //12345678901234567890
        //SEQ1> F1-F2 / LEVEL
        //20000-20000Hz/ 10.1V
        //20000Hz      / 10.1V
        if(Seq[seq_num].Gen_Func == GEN_SWEEP)
        {
            sprintf(VSTR1,"SEQ%d> F1-F2 / LEVEL", seq_num+1);
            sprintf(VSTR2,"%5.0f-%5.0fHz/ %4.1fV", Seq[seq_num].Gen_Sweep_F1, Seq[seq_num].Gen_Sweep_F2, Seq[seq_num].Gen_Amplitude);
        }
        else
        {
            sprintf(VSTR1,"SEQ%d> FREQ  / LEVEL", seq_num+1);
            sprintf(VSTR2,"%5.0fHz      / %4.1fV", Seq[seq_num].Gen_Sweep_F1, Seq[seq_num].Gen_Amplitude);
        }

        OnCursor = *CurMode;
        break;
               
    case MODE_SET_CONT_SIGNAL_FUNC:
		i = 0;
        if(premode != *OpMode)
        {
            cur_xpos = 7;
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++; 
#if defined BK2039 || defined BK2039A || defined BK2039_5CH
            if(Cont.Gen_Func == GEN_SWEEP)
#elif BK2039X
            if(Cont[i].Gen_Func == GEN_SWEEP)
#endif
            {
                if      (cur_xpos > 11)  cur_xpos = 11;
                else if (cur_xpos == 8)  cur_xpos = 11;
            }
            else
            {
                if      (cur_xpos >  7)  cur_xpos = 7;
            }
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
#if defined BK2039 || defined BK2039A || defined BK2039_5CH
            if(Cont.Gen_Func == GEN_SWEEP)
#elif BK2039X
            if(Cont[i].Gen_Func == GEN_SWEEP)
#endif
            {
                if      (cur_xpos <   7) cur_xpos = 7; 
                else if (cur_xpos == 10) cur_xpos = 7;
            }
            else
            {
                if      (cur_xpos <   7) cur_xpos = 7;
            } 
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if(cur_xpos == 7)
            {
                num_pos = 1;
#if defined BK2039 || defined BK2039A || defined BK2039_5CH
                Cont.Gen_Func = (int)Calc_Number(Cont.Gen_Func, num_pos, set_val);
                if      (Cont.Gen_Func > GEN_SWEEP) Cont.Gen_Func = GEN_SWEEP;
                else if (Cont.Gen_Func < GEN_SINE)  Cont.Gen_Func = GEN_SINE;
#elif BK2039X
                Cont[i].Gen_Func = (int)Calc_Number(Cont[i].Gen_Func, num_pos, set_val);
                if      (Cont[i].Gen_Func > GEN_SWEEP) Cont[i].Gen_Func = GEN_SWEEP;
                else if (Cont[i].Gen_Func < GEN_SINE)  Cont[i].Gen_Func = GEN_SINE;
#endif
            }
            else if(cur_xpos == 11)
            {
                num_pos = 1;
#if defined BK2039 || defined BK2039A || defined BK2039_5CH
                Cont.Gen_Sweep_Mode = (int)Calc_Number(Cont.Gen_Sweep_Mode, num_pos, set_val);
                if      (Cont.Gen_Sweep_Mode >  SWEEP_LOG_UP_DN)    Cont.Gen_Sweep_Mode = SWEEP_LOG_UP_DN;
                else if (Cont.Gen_Sweep_Mode <  SWEEP_LIN_UP)       Cont.Gen_Sweep_Mode = SWEEP_LIN_UP;
                else if (Cont.Gen_Sweep_Mode == SWEEP_LOG_DN+1)     Cont.Gen_Sweep_Mode = SWEEP_LIN_UP_DN;
                else if (Cont.Gen_Sweep_Mode == SWEEP_LIN_UP_DN-1)  Cont.Gen_Sweep_Mode = SWEEP_LOG_DN;
            }
        }

        if(Cont.Gen_Func == GEN_SWEEP)   sprintf(lstr, "  SWEEP");
        else                             sprintf(lstr, "   SINE");
        
        if      (Cont.Gen_Sweep_Mode == SWEEP_LIN_UP)       sprintf(lstr2, "LIN UP");
        else if (Cont.Gen_Sweep_Mode == SWEEP_LIN_DN)       sprintf(lstr2, "LIN DN");
        else if (Cont.Gen_Sweep_Mode == SWEEP_LIN_UP_DN)    sprintf(lstr2, "LIN UP-DN");
        else if (Cont.Gen_Sweep_Mode == SWEEP_LOG_UP)       sprintf(lstr2, "LOG UP");
        else if (Cont.Gen_Sweep_Mode == SWEEP_LOG_DN)       sprintf(lstr2, "LOG DN");
        else if (Cont.Gen_Sweep_Mode == SWEEP_LOG_UP_DN)    sprintf(lstr2, "LOG UP-DN");
        
        sprintf(VSTR1,"CONT> FUNC / MODE");  
        if(Cont.Gen_Func == GEN_SWEEP)	sprintf(VSTR2,"%s / %s", lstr, lstr2);
        else                                sprintf(VSTR2,"%s", lstr);
#elif BK2039X
                Cont[i].Gen_Sweep_Mode = (int)Calc_Number(Cont[i].Gen_Sweep_Mode, num_pos, set_val);
                if      (Cont[i].Gen_Sweep_Mode >  SWEEP_LOG_UP_DN)    Cont[i].Gen_Sweep_Mode = SWEEP_LOG_UP_DN;
                else if (Cont[i].Gen_Sweep_Mode <  SWEEP_LIN_UP)       Cont[i].Gen_Sweep_Mode = SWEEP_LIN_UP;
                else if (Cont[i].Gen_Sweep_Mode == SWEEP_LOG_DN+1)     Cont[i].Gen_Sweep_Mode = SWEEP_LIN_UP_DN;
                else if (Cont[i].Gen_Sweep_Mode == SWEEP_LIN_UP_DN-1)  Cont[i].Gen_Sweep_Mode = SWEEP_LOG_DN;
            }
        }

        if(Cont[i].Gen_Func == GEN_SWEEP)   sprintf(lstr, "  SWEEP");
        else                                sprintf(lstr, "   SINE");
        
        if      (Cont[i].Gen_Sweep_Mode == SWEEP_LIN_UP)       sprintf(lstr2, "LIN UP");
        else if (Cont[i].Gen_Sweep_Mode == SWEEP_LIN_DN)       sprintf(lstr2, "LIN DN");
        else if (Cont[i].Gen_Sweep_Mode == SWEEP_LIN_UP_DN)    sprintf(lstr2, "LIN UP-DN");
        else if (Cont[i].Gen_Sweep_Mode == SWEEP_LOG_UP)       sprintf(lstr2, "LOG UP");
        else if (Cont[i].Gen_Sweep_Mode == SWEEP_LOG_DN)       sprintf(lstr2, "LOG DN");
        else if (Cont[i].Gen_Sweep_Mode == SWEEP_LOG_UP_DN)    sprintf(lstr2, "LOG UP-DN");

        sprintf(VSTR1,"CONT> FUNC / MODE");  
        if(Cont[i].Gen_Func == GEN_SWEEP)	sprintf(VSTR2,"%s / %s", lstr, lstr2);
        else                                sprintf(VSTR2,"%s", lstr);
#endif
        OnCursor = *CurMode;
        break;
        
#ifdef BK2039X
        case MODE_SET_CONT_SIGNAL_FUNC_B:
            i = 1;
            if(premode != *OpMode)
            {
                cur_xpos = 7;
            }

            if(set_pos == KEY_RIGHT)
            {
                cur_xpos++; 
                if(Cont[i].Gen_Func == GEN_SWEEP)
                {
                    if      (cur_xpos > 11)  cur_xpos = 11;
                    else if (cur_xpos == 8)  cur_xpos = 11;
                }
                else
                {
                    if      (cur_xpos >  7)  cur_xpos = 7;
                }
            }
            else if(set_pos == KEY_LEFT)
            {
                cur_xpos--;
                if(Cont[i].Gen_Func == GEN_SWEEP)
                {
                    if      (cur_xpos <   7) cur_xpos = 7; 
                    else if (cur_xpos == 10) cur_xpos = 7;
                }
                else
                {
                    if      (cur_xpos <   7) cur_xpos = 7;
                } 
            }
            else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
            {
                if(cur_xpos == 7)
                {
                    num_pos = 1;
                    Cont[i].Gen_Func = (int)Calc_Number(Cont[i].Gen_Func, num_pos, set_val);
                    if      (Cont[i].Gen_Func > GEN_SWEEP) Cont[i].Gen_Func = GEN_SWEEP;
                    else if (Cont[i].Gen_Func < GEN_SINE)  Cont[i].Gen_Func = GEN_SINE;
                }
                else if(cur_xpos == 11)
                {
                    num_pos = 1;
                    Cont[i].Gen_Sweep_Mode = (int)Calc_Number(Cont[i].Gen_Sweep_Mode, num_pos, set_val);
                    if      (Cont[i].Gen_Sweep_Mode >  SWEEP_LOG_UP_DN)    Cont[i].Gen_Sweep_Mode = SWEEP_LOG_UP_DN;
                    else if (Cont[i].Gen_Sweep_Mode <  SWEEP_LIN_UP)       Cont[i].Gen_Sweep_Mode = SWEEP_LIN_UP;
                    else if (Cont[i].Gen_Sweep_Mode == SWEEP_LOG_DN+1)     Cont[i].Gen_Sweep_Mode = SWEEP_LIN_UP_DN;
                    else if (Cont[i].Gen_Sweep_Mode == SWEEP_LIN_UP_DN-1)  Cont[i].Gen_Sweep_Mode = SWEEP_LOG_DN;
                }
            }

            if(Cont[i].Gen_Func == GEN_SWEEP) sprintf(lstr, "  SWEEP");
            else sprintf(lstr, "   SINE");
            
            if      (Cont[i].Gen_Sweep_Mode == SWEEP_LIN_UP)       sprintf(lstr2, "LIN UP");
            else if (Cont[i].Gen_Sweep_Mode == SWEEP_LIN_DN)       sprintf(lstr2, "LIN DN");
            else if (Cont[i].Gen_Sweep_Mode == SWEEP_LIN_UP_DN)    sprintf(lstr2, "LIN UP-DN");
            else if (Cont[i].Gen_Sweep_Mode == SWEEP_LOG_UP)       sprintf(lstr2, "LOG UP");
            else if (Cont[i].Gen_Sweep_Mode == SWEEP_LOG_DN)       sprintf(lstr2, "LOG DN");
            else if (Cont[i].Gen_Sweep_Mode == SWEEP_LOG_UP_DN)    sprintf(lstr2, "LOG UP-DN");
            
            //12345678901234567890
            //CONT> FUNC / MODE  
            //   SINE / 
            //  SWEEP / LIN UP-DN
            sprintf(VSTR1,"CONT_B> FUNC / MODE");
            if(Cont[i].Gen_Func == GEN_SWEEP)	sprintf(VSTR2,"%s / %s", lstr, lstr2);
            else                                sprintf(VSTR2,"%s", lstr);

            OnCursor = *CurMode;
            break;
#endif
    
    case MODE_SET_CONT_FREQ_LEVEL:
        if(premode != *OpMode)
        {
            cur_xpos = 4;
        }
#if defined BK2039 || defined BK2039A || defined BK2039_5CH
        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if(Cont.Gen_Func == GEN_SWEEP)
            {
                if		(cur_xpos >  19)	cur_xpos = 19;
                else if	(cur_xpos == 12)	cur_xpos = 17;
                else if	(cur_xpos ==  6)	cur_xpos =  8;
            }
            else
            {
                if		(cur_xpos >  19)	cur_xpos = 19;
                else if	(cur_xpos ==  6)	cur_xpos = 17;
            }
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if(Cont.Gen_Func == GEN_SWEEP)
            {
                if		(cur_xpos <   2)	cur_xpos =  2;
                else if	(cur_xpos ==  7)	cur_xpos =  5;
                else if	(cur_xpos == 16)	cur_xpos = 11;
            }
            else
            {
                if		(cur_xpos <   2)	cur_xpos =  2;
                else if	(cur_xpos == 16)	cur_xpos =  5;
            }
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {            
            if(Cont.Gen_Func == GEN_SWEEP)
            {
                if((cur_xpos >= 2) && (cur_xpos <= 5))
                {
                    dot_pos = 6;
                    num_pos = dot_pos - cur_xpos;
                    Cont.Gen_Sweep_F1 = (float)Calc_Number(Cont.Gen_Sweep_F1, num_pos, set_val);
                    if		(Cont.Gen_Sweep_F1 > Cont.Gen_Sweep_F2)	Cont.Gen_Sweep_F1 = Cont.Gen_Sweep_F2;
                    else if	(Cont.Gen_Sweep_F1 < MIN_FRQ) 				Cont.Gen_Sweep_F1 = MIN_FRQ;
                }
                else if((cur_xpos >= 8) && (cur_xpos <= 11))
                {
                    dot_pos = 12;
                    num_pos = dot_pos - cur_xpos;
                    Cont.Gen_Sweep_F2 = (float)Calc_Number(Cont.Gen_Sweep_F2, num_pos, set_val);
                    if		(Cont.Gen_Sweep_F2 > MAX_FRQ) 				Cont.Gen_Sweep_F2 = MAX_FRQ;
                    else if	(Cont.Gen_Sweep_F2 < Cont.Gen_Sweep_F1) 	Cont.Gen_Sweep_F2 = Cont.Gen_Sweep_F1;
                }
                if((cur_xpos >= 17) && (cur_xpos <= 19))
                {
                    dot_pos = 18;
                    num_pos = dot_pos - cur_xpos;
                    Cont.Gen_Amplitude = (float)Calc_Number(Cont.Gen_Amplitude, num_pos, set_val);
                    if		(Cont.Gen_Amplitude > GEN_VOLT_MAX) 		Cont.Gen_Amplitude = GEN_VOLT_MAX;
                    else if	(Cont.Gen_Amplitude < 0.0) 				Cont.Gen_Amplitude = 0.0;
                }
            }
            else
            {
                if((cur_xpos >= 1) && (cur_xpos <= 5))
                {
                    dot_pos = 6;
                    num_pos = dot_pos - cur_xpos;
                    Cont.Gen_Sweep_F1 = (float)Calc_Number(Cont.Gen_Sweep_F1, num_pos, set_val);
                    if		(Cont.Gen_Sweep_F1 > MAX_FRQ) 			Cont.Gen_Sweep_F1 = MAX_FRQ;
                    else if	(Cont.Gen_Sweep_F1 < MIN_FRQ) 			Cont.Gen_Sweep_F1 = MIN_FRQ;
                }
                if((cur_xpos >= 17) && (cur_xpos <= 19))
                {
                    dot_pos = 18;
                    num_pos = dot_pos - cur_xpos;
                    Cont.Gen_Amplitude = (float)Calc_Number(Cont.Gen_Amplitude, num_pos, set_val);
                    if		(Cont.Gen_Amplitude > GEN_VOLT_CONT_MAX) Cont.Gen_Amplitude = GEN_VOLT_CONT_MAX;
                    else if	(Cont.Gen_Amplitude < 0.0) 				Cont.Gen_Amplitude = 0.0;
                }
            }
        }

        if(Cont.Gen_Func == GEN_SWEEP)
        {
            sprintf(VSTR1, "CONT> F1-F2  / LEVEL");
            sprintf(VSTR2, "%5.0f-%5.0fHz/ %4.1fV", Cont.Gen_Sweep_F1, Cont.Gen_Sweep_F2, Cont.Gen_Amplitude);
        }
        else
        {
            sprintf(VSTR1, "CONT> FREQ   / LEVEL");
            sprintf(VSTR2, "%5.0fHz      / %4.1fV", Cont.Gen_Sweep_F1, Cont.Gen_Amplitude);
        }
#elif BK2039X
		i = 0;
        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if(Cont[i].Gen_Func == GEN_SWEEP)
            {
                if		(cur_xpos >  19)	cur_xpos = 19;
                else if	(cur_xpos == 12)	cur_xpos = 17;
                else if	(cur_xpos ==  6)	cur_xpos =  8;
            }
            else
            {
                if		(cur_xpos >  19)	cur_xpos = 19;
                else if	(cur_xpos ==  6)	cur_xpos = 17;
            }
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if(Cont[i].Gen_Func == GEN_SWEEP)
            {
                if		(cur_xpos <   2)	cur_xpos =  2;
                else if	(cur_xpos ==  7)	cur_xpos =  5;
                else if	(cur_xpos == 16)	cur_xpos = 11;
            }
            else
            {
                if		(cur_xpos <   2)	cur_xpos =  2;
                else if	(cur_xpos == 16)	cur_xpos =  5;
            }
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {            
            if(Cont[i].Gen_Func == GEN_SWEEP)
            {
                if((cur_xpos >= 2) && (cur_xpos <= 5))
                {
                    dot_pos = 6;
                    num_pos = dot_pos - cur_xpos;
                    Cont[i].Gen_Sweep_F1 = (float)Calc_Number(Cont[i].Gen_Sweep_F1, num_pos, set_val);
                    if		(Cont[i].Gen_Sweep_F1 > Cont[i].Gen_Sweep_F2)	Cont[i].Gen_Sweep_F1 = Cont[i].Gen_Sweep_F2;
                    else if	(Cont[i].Gen_Sweep_F1 < MIN_FRQ) 				Cont[i].Gen_Sweep_F1 = MIN_FRQ;
                }
                else if((cur_xpos >= 8) && (cur_xpos <= 11))
                {
                    dot_pos = 12;
                    num_pos = dot_pos - cur_xpos;
                    Cont[i].Gen_Sweep_F2 = (float)Calc_Number(Cont[i].Gen_Sweep_F2, num_pos, set_val);
                    if		(Cont[i].Gen_Sweep_F2 > MAX_FRQ) 				Cont[i].Gen_Sweep_F2 = MAX_FRQ;
                    else if	(Cont[i].Gen_Sweep_F2 < Cont[i].Gen_Sweep_F1) 	Cont[i].Gen_Sweep_F2 = Cont[i].Gen_Sweep_F1;
                }
                if((cur_xpos >= 17) && (cur_xpos <= 19))
                {
                    dot_pos = 18;
                    num_pos = dot_pos - cur_xpos;
                    Cont[i].Gen_Amplitude = (float)Calc_Number(Cont[i].Gen_Amplitude, num_pos, set_val);
                    if		(Cont[i].Gen_Amplitude > GEN_VOLT_MAX) 		Cont[i].Gen_Amplitude = GEN_VOLT_MAX;
                    else if	(Cont[i].Gen_Amplitude < 0.0) 				Cont[i].Gen_Amplitude = 0.0;
                }
            }
            else
            {
                if((cur_xpos >= 1) && (cur_xpos <= 5))
                {
                    dot_pos = 6;
                    num_pos = dot_pos - cur_xpos;
                    Cont[i].Gen_Sweep_F1 = (float)Calc_Number(Cont[i].Gen_Sweep_F1, num_pos, set_val);
                    if		(Cont[i].Gen_Sweep_F1 > MAX_FRQ) 			Cont[i].Gen_Sweep_F1 = MAX_FRQ;
                    else if	(Cont[i].Gen_Sweep_F1 < MIN_FRQ) 			Cont[i].Gen_Sweep_F1 = MIN_FRQ;
                }
                if((cur_xpos >= 17) && (cur_xpos <= 19))
                {
                    dot_pos = 18;
                    num_pos = dot_pos - cur_xpos;
                    Cont[i].Gen_Amplitude = (float)Calc_Number(Cont[i].Gen_Amplitude, num_pos, set_val);
                    if		(Cont[i].Gen_Amplitude > GEN_VOLT_CONT_MAX) Cont[i].Gen_Amplitude = GEN_VOLT_CONT_MAX;
                    else if	(Cont[i].Gen_Amplitude < 0.0) 				Cont[i].Gen_Amplitude = 0.0;
                }
            }
        }

        //12345678901234567890
        //CONT> F1-F2 / LEVEL
        //20000-20000Hz/ 10.1V
        //20000Hz      / 10.1V
        if(Cont[i].Gen_Func == GEN_SWEEP)
        {
            sprintf(VSTR1, "CONT> F1-F2  / LEVEL");
            sprintf(VSTR2, "%5.0f-%5.0fHz/ %4.1fV", Cont[i].Gen_Sweep_F1, Cont[i].Gen_Sweep_F2, Cont[i].Gen_Amplitude);
        }
        else
        {
            sprintf(VSTR1, "CONT> FREQ   / LEVEL");
            sprintf(VSTR2, "%5.0fHz      / %4.1fV", Cont[i].Gen_Sweep_F1, Cont[i].Gen_Amplitude);
        }
#endif
        OnCursor = *CurMode;
        break;

#ifdef BK2039X
	case MODE_SET_CONT_FREQ_LEVEL_B:
		i = 1;
        if(premode != *OpMode)
        {
            cur_xpos = 4;
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if(Cont[i].Gen_Func == GEN_SWEEP)
            {
                if		(cur_xpos >  19)	cur_xpos = 19;
                else if	(cur_xpos == 12)	cur_xpos = 17;
                else if	(cur_xpos ==  6)	cur_xpos =  8;
            }
            else
            {
                if		(cur_xpos >  19)	cur_xpos = 19;
                else if	(cur_xpos ==  6)	cur_xpos = 17;
            }
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if(Cont[i].Gen_Func == GEN_SWEEP)
            {
                if		(cur_xpos <   2)	cur_xpos =  2;
                else if	(cur_xpos ==  7)	cur_xpos =  5;
                else if	(cur_xpos == 16)	cur_xpos = 11;
            }
            else
            {
                if		(cur_xpos <   2)	cur_xpos =  2;
                else if	(cur_xpos == 16)	cur_xpos =  5;
            }
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {            
            if(Cont[i].Gen_Func == GEN_SWEEP)
            {
                if((cur_xpos >= 2) && (cur_xpos <= 5))
                {
                    dot_pos = 6;
                    num_pos = dot_pos - cur_xpos;
                    Cont[i].Gen_Sweep_F1 = (float)Calc_Number(Cont[i].Gen_Sweep_F1, num_pos, set_val);
                    if		(Cont[i].Gen_Sweep_F1 > Cont[i].Gen_Sweep_F2)	Cont[i].Gen_Sweep_F1 = Cont[i].Gen_Sweep_F2;
                    else if	(Cont[i].Gen_Sweep_F1 < MIN_FRQ) 				Cont[i].Gen_Sweep_F1 = MIN_FRQ;
                }
                else if((cur_xpos >= 8) && (cur_xpos <= 11))
                {
                    dot_pos = 12;
                    num_pos = dot_pos - cur_xpos;
                    Cont[i].Gen_Sweep_F2 = (float)Calc_Number(Cont[i].Gen_Sweep_F2, num_pos, set_val);
                    if		(Cont[i].Gen_Sweep_F2 > MAX_FRQ) 				Cont[i].Gen_Sweep_F2 = MAX_FRQ;
                    else if	(Cont[i].Gen_Sweep_F2 < Cont[i].Gen_Sweep_F1) 	Cont[i].Gen_Sweep_F2 = Cont[i].Gen_Sweep_F1;
                }
                if((cur_xpos >= 17) && (cur_xpos <= 19))
                {
                    dot_pos = 18;
                    num_pos = dot_pos - cur_xpos;
                    Cont[i].Gen_Amplitude = (float)Calc_Number(Cont[i].Gen_Amplitude, num_pos, set_val);
                    if		(Cont[i].Gen_Amplitude > GEN_VOLT_MAX) 		Cont[i].Gen_Amplitude = GEN_VOLT_MAX;
                    else if	(Cont[i].Gen_Amplitude < 0.0) 				Cont[i].Gen_Amplitude = 0.0;
                }
            }
            else
            {
                if((cur_xpos >= 1) && (cur_xpos <= 5))
                {
                    dot_pos = 6;
                    num_pos = dot_pos - cur_xpos;
                    Cont[i].Gen_Sweep_F1 = (float)Calc_Number(Cont[i].Gen_Sweep_F1, num_pos, set_val);
                    if		(Cont[i].Gen_Sweep_F1 > MAX_FRQ) 			Cont[i].Gen_Sweep_F1 = MAX_FRQ;
                    else if	(Cont[i].Gen_Sweep_F1 < MIN_FRQ) 			Cont[i].Gen_Sweep_F1 = MIN_FRQ;
                }
                if((cur_xpos >= 17) && (cur_xpos <= 19))
                {
                    dot_pos = 18;
                    num_pos = dot_pos - cur_xpos;
                    Cont[i].Gen_Amplitude = (float)Calc_Number(Cont[i].Gen_Amplitude, num_pos, set_val);
                    if		(Cont[i].Gen_Amplitude > GEN_VOLT_CONT_MAX) Cont[i].Gen_Amplitude = GEN_VOLT_CONT_MAX;
                    else if	(Cont[i].Gen_Amplitude < 0.0) 				Cont[i].Gen_Amplitude = 0.0;
                }
            }
        }

        //12345678901234567890
        //CONT> F1-F2 / LEVEL
        //20000-20000Hz/ 10.1V
        //20000Hz      / 10.1V
        if(Cont[i].Gen_Func == GEN_SWEEP)
        {
            sprintf(VSTR1, "CONT_B> F1-F2 / LEVEL");
            sprintf(VSTR2, "%5.0f-%5.0fHz/ %4.1fV", Cont[i].Gen_Sweep_F1, Cont[i].Gen_Sweep_F2, Cont[i].Gen_Amplitude);
        }
        else
        {
            sprintf(VSTR1, "CONT_B> FREQ / LEVEL");
            sprintf(VSTR2, "%5.0fHz      / %4.1fV", Cont[i].Gen_Sweep_F1, Cont[i].Gen_Amplitude);
        }

        OnCursor = *CurMode;
        break;
#endif

    case MODE_SET_CONT_SWEEP_TIME:
        if(premode != *OpMode)
        {
            cur_xpos = 7;
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if		(cur_xpos >  10) cur_xpos = 10;
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if		(cur_xpos <   8) cur_xpos = 8;
        }
#if defined BK2039 || defined BK2039A || defined BK2039_5CH
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if((cur_xpos >= 8) || (cur_xpos <= 10))
            {
                dot_pos = 9;
                num_pos = dot_pos - cur_xpos;
                Cont.Gen_Sweep_Time = (float)Calc_Number(Cont.Gen_Sweep_Time, num_pos, set_val);
                if		(Cont.Gen_Sweep_Time > GEN_MAX_TIME_SIGNAL)	Cont.Gen_Sweep_Time = GEN_MAX_TIME_SIGNAL;
                else if	(Cont.Gen_Sweep_Time < 0.5) 					Cont.Gen_Sweep_Time = 0.5; 
            }
        }
        
        if(Cont.Gen_Func == GEN_SWEEP)
        {
        sprintf(VSTR1,"CONT> SWEEP TIME");
        sprintf(VSTR2,"TIME: %4.1fs", Cont.Gen_Sweep_Time);
        }
#elif BK2039X
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {    
            i = 0;
            if((cur_xpos >= 8) || (cur_xpos <= 10))
            {
                dot_pos = 9;
                num_pos = dot_pos - cur_xpos;
                Cont[i].Gen_Sweep_Time = (float)Calc_Number(Cont[i].Gen_Sweep_Time, num_pos, set_val);
                if		(Cont[i].Gen_Sweep_Time > GEN_MAX_TIME_SIGNAL)	Cont[i].Gen_Sweep_Time = GEN_MAX_TIME_SIGNAL;
                else if	(Cont[i].Gen_Sweep_Time < 0.5) 					Cont[i].Gen_Sweep_Time = 0.5; 
            }
        }
        if(Cont[i].Gen_Func == GEN_SWEEP)
        {
        sprintf(VSTR1,"CONT> SWEEP TIME");
        sprintf(VSTR2,"TIME: %4.1fs", Cont[i].Gen_Sweep_Time);
        }
#endif
        OnCursor = *CurMode;                                      
        break;

#ifdef BK2039X
	case MODE_SET_CONT_SWEEP_TIME_B:
		i = 1;
        if(premode != *OpMode)
        {
            cur_xpos = 7;
        }

        if(set_pos == KEY_RIGHT)
        {
            cur_xpos++;
            if		(cur_xpos >  10) cur_xpos = 10;
        }
        else if(set_pos == KEY_LEFT)
        {
            cur_xpos--;
            if		(cur_xpos <   8)  cur_xpos = 8;
        }
        else if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            if((cur_xpos >= 8) || (cur_xpos <= 10))
            {
                dot_pos = 9;
                num_pos = dot_pos - cur_xpos;
                Cont[i].Gen_Sweep_Time = (float)Calc_Number(Cont[i].Gen_Sweep_Time, num_pos, set_val);
                if		(Cont[i].Gen_Sweep_Time > GEN_MAX_TIME_SIGNAL)	Cont[i].Gen_Sweep_Time = GEN_MAX_TIME_SIGNAL;
                else if	(Cont[i].Gen_Sweep_Time < 0.5) 					Cont[i].Gen_Sweep_Time = 0.5; 
            }
        }

        sprintf(VSTR1,"CONT_B> SWEEP TIME");
        sprintf(VSTR2,"TIME: %4.1fs", Cont[i].Gen_Sweep_Time);

        OnCursor = *CurMode;                                      
        break;
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
    case MODE_SET_TEST_BEEP:
    case MODE_SET_CONT_BEEP:
        if(premode != *OpMode)
        {
            cur_xpos = 7;
        }

        if((set_val == KEY_UP)||(set_val == KEY_DOWN))
        {
            num_pos = 1;
            Setup.TestBeepActive = (int)Calc_Number(Setup.TestBeepActive, num_pos, set_val);
            if(Setup.TestBeepActive > 1) Setup.TestBeepActive = 1;
            else if(Setup.TestBeepActive < 0) Setup.TestBeepActive = 0;
        }

        sprintf(VSTR1,"SET> TEST BEEP");

        if(Setup.TestBeepActive == 1)
        {
            sprintf(VSTR2,"RUN: ON");
        }
        else
        {
            sprintf(VSTR2,"RUN:OFF");
        }

        if(*CurMode == CUR_MODE_CURSOR) *CurMode = CUR_MODE_VALUE;
        OnCursor = *CurMode;
        break;
    }

    if(*OpMode != MODE_RUN)
    {
        LCDUpdate(VSTR1, VSTR2);
        LCDSetCursor(cur_xpos, 2, OnCursor);
    }
    premode = *OpMode;

    return(1);
}

void ModeProcess(void)
{
    char 		set_val,set_pos;
    int  		Param;
    char 		key;
    static int 	CursorMode = CUR_MODE_ITEM;
    int 		phase;
    static int 	SysMode = 0;

    set_val = KEY_NON;
    set_pos = KEY_NON;
    //Param 	= 0;
	
    ////////////////////////////////////////////////////////////////
    // Button Processor
    key = Get_KEY();
    if( key != KEY_NON )
    {
        if((OpMode == MODE_RUN) || ((OpMode >= MODE_RUN_SET) && (OpMode <= MODE_RUN_SET_END)))
        {
            switch(key)
            {
            case KEY_RUN:
                if(OnTestProcess == 1)
                {
                    StopTest();
                }
                else
                {
                    StartTest();
                    DisplayPage = 0;
                }

                OpMode = MODE_RUN;
                break;

            case KEY_SET:
                StopTest();

                CursorMode = CUR_MODE_ITEM;
                OpMode = MODE_SET_TEST_MODE;
                break;

            case KEY_CAL:
                break;
            case KEY_ENT:
                if(OnTestProcess == 0)
                {   
                    if(Setup.TestMode == TEST_HAPTIC)
                        Reset_HapticCount();
                    else
                        Reset_Count();
                }
                else
                {
                }

                OpMode = MODE_RUN;
                break;
            case KEY_LEFT:
            case KEY_RIGHT:
            case KEY_UP:
            case KEY_DOWN:

                break;
            case KEY_PUSH:
                if(OnTestProcess == 1)
                {
                    StopTest();
                }
                else
                {
                    StartTest();
                    DisplayPage = 0;
                }

                OpMode = MODE_RUN;

                delay_ms(100);
                ClearPhaseCount();
                break;
            }
        }
        else if (OpMode == MODE_SET_TEST_MODE)
        {
            switch(key)
            {
            case KEY_RUN:
                //Setup_System();
                StartTest();

                DisplayPage = 0;
                CursorMode = CUR_MODE_ITEM;
                OpMode = MODE_RUN;
                break;
            case KEY_LEFT:
            case KEY_RIGHT:
                CursorMode = CUR_MODE_CURSOR;
                break;
            case KEY_UP:
            case KEY_DOWN:
                CursorMode = CUR_MODE_VALUE;
                break;
            case KEY_SET:
                if(CursorMode != CUR_MODE_ITEM)
                {
                    CursorMode = CUR_MODE_ITEM;
                }
                else
                {
                    if(Setup.TestMode == TEST_CONTINUE)     OpMode = MODE_SET_CONTINUE;
                    else if(Setup.TestMode == TEST_ON_OFF)  OpMode = MODE_SET_TEST;  
                    else if(Setup.TestMode == TEST_HAPTIC)  OpMode = MODE_SET_HAPTIC;
                }

                break;
            case KEY_ENT:
                if(CursorMode == CUR_MODE_ITEM) CursorMode = CUR_MODE_CURSOR;
                else							CursorMode = CUR_MODE_ITEM;
                break;
            case KEY_PUSH:
                if(CursorMode == CUR_MODE_ITEM)			CursorMode = CUR_MODE_CURSOR;
                else if(CursorMode == CUR_MODE_CURSOR) 	CursorMode = CUR_MODE_VALUE;
                else if(CursorMode == CUR_MODE_VALUE)	CursorMode = CUR_MODE_CURSOR;
                else									CursorMode = CUR_MODE_ITEM;

                delay_ms(100);
                ClearPhaseCount();
                break;
            }
        }
        // Mode Set Continue //////////////////////////////////////////////////
        else if((OpMode >= MODE_SET_CONTINUE)&&(OpMode <= MODE_SET_CONTINUE_END))
        {                
            switch(key)
            {
            case KEY_RUN:
                //Setup_System();
                StartTest();

                DisplayPage = 0;
                CursorMode = CUR_MODE_ITEM;
                OpMode = MODE_RUN;
                break;
            case KEY_LEFT:
            case KEY_RIGHT:
                CursorMode = CUR_MODE_CURSOR;
                break;
            case KEY_UP:
            case KEY_DOWN:
                CursorMode = CUR_MODE_VALUE;
                break;
            case KEY_SET:
                if(CursorMode != CUR_MODE_ITEM)
                {
                    CursorMode = CUR_MODE_ITEM;
                }
                else
                {
                    if(++OpMode >= MODE_SET_CONTINUE_END)
                    {
                        if(++SysMode >= 5)  			OpMode = MODE_SYS_LED_TYPE;
                        else                			OpMode = MODE_SET_CONTINUE_END;
                    }
                }

                break;
            case KEY_ENT:
                if		(CursorMode == CUR_MODE_ITEM)	CursorMode = CUR_MODE_CURSOR;
                else									CursorMode = CUR_MODE_ITEM;
                break;
            case KEY_PUSH:
                if		(CursorMode == CUR_MODE_ITEM)	CursorMode = CUR_MODE_CURSOR;
                else if	(CursorMode == CUR_MODE_CURSOR)	CursorMode = CUR_MODE_VALUE;
                else if	(CursorMode == CUR_MODE_VALUE)	CursorMode = CUR_MODE_CURSOR;
                else									CursorMode = CUR_MODE_ITEM;

                delay_ms(100);
                ClearPhaseCount();
                break;
            }
        }
        
        // Mode Set ONOFF //////////////////////////////////////////////////
        else if((OpMode >= MODE_SET_TEST)&&(OpMode <= MODE_SET_TEST_END))
        {                
            switch(key)
            {
            case KEY_RUN:
                //Setup_System();
                StartTest();

                DisplayPage = 0;
                CursorMode = CUR_MODE_ITEM;
                OpMode = MODE_RUN;
                break;
            case KEY_LEFT:
            case KEY_RIGHT:
                CursorMode = CUR_MODE_CURSOR;
                break;
            case KEY_UP:
            case KEY_DOWN:
                CursorMode = CUR_MODE_VALUE;
                break;
            case KEY_SET:
                if(CursorMode != CUR_MODE_ITEM)
                {
                    CursorMode = CUR_MODE_ITEM;
                }
                else
                {
                    if(++OpMode >= MODE_SET_TEST_END)
                    {
                        if(++SysMode >= 5)  OpMode = MODE_SYS_LED_TYPE;
                        else                OpMode = MODE_SET_TEST_END;
                    }
                }

                break;
            case KEY_ENT:
                if(CursorMode == CUR_MODE_ITEM) CursorMode = CUR_MODE_CURSOR;
                else							CursorMode = CUR_MODE_ITEM;
                break;
            case KEY_PUSH:
                if(CursorMode == CUR_MODE_ITEM)			CursorMode = CUR_MODE_CURSOR;
                else if(CursorMode == CUR_MODE_CURSOR) 	CursorMode = CUR_MODE_VALUE;
                else if(CursorMode == CUR_MODE_VALUE)	CursorMode = CUR_MODE_CURSOR;
                else									CursorMode = CUR_MODE_ITEM;

                delay_ms(100);
                ClearPhaseCount();
                break;
            }
        }
        // Mode Set Haptic //////////////////////////////////////////////////
        else if((OpMode >= MODE_SET_HAPTIC)&&(OpMode <= MODE_SET_HAPTIC_END))
        {
            switch(key)
            {
            case KEY_RUN:   
                EEP_SaveHapticSetup();
                //Setup_System();
                StartTest();

                SysMode 	= 0;
                DisplayPage = 0;
                CursorMode 	= CUR_MODE_ITEM;
                OpMode 		= MODE_RUN;
                break;
            case KEY_LEFT:
            case KEY_RIGHT:                  
                CursorMode = CUR_MODE_CURSOR;
                break;
            case KEY_UP:
            case KEY_DOWN:
                CursorMode = CUR_MODE_VALUE;
                break;
            case KEY_SET:                 
                if(CursorMode != CUR_MODE_ITEM)
                {
                    CursorMode = CUR_MODE_ITEM;
                }
                else
                {
                    if(++OpMode >= MODE_SET_HAPTIC_END)
                    {
                        if(++SysMode >= 5)	OpMode = MODE_SYS_LED_TYPE;
                        else				OpMode = MODE_SET_HAPTIC_END;
                    }
                }

                break;
            case KEY_ENT:
                if(CursorMode == CUR_MODE_ITEM)	CursorMode = CUR_MODE_CURSOR;
                else							CursorMode = CUR_MODE_ITEM;
                break;
            case KEY_PUSH:                 
                if(CursorMode == CUR_MODE_ITEM)			CursorMode = CUR_MODE_CURSOR;
                else if(CursorMode == CUR_MODE_CURSOR)	CursorMode = CUR_MODE_VALUE;
                else if(CursorMode == CUR_MODE_VALUE)	CursorMode = CUR_MODE_CURSOR;
                else									CursorMode = CUR_MODE_ITEM;

                delay_ms(100);
                ClearPhaseCount();
                break;
            }
        }
        // Mode Sys //////////////////////////////////////////////////
        else if((OpMode >= MODE_SYS)&&(OpMode <= MODE_SYS_END))
        {
            switch(key)
            {
            case KEY_RUN:
                //Setup_System();
                StartTest();

                SysMode = 0;
                DisplayPage = 0;
                CursorMode = CUR_MODE_ITEM;
                OpMode = MODE_RUN;
                break;
            case KEY_LEFT:
            case KEY_RIGHT:
                CursorMode = CUR_MODE_CURSOR;
                break;
            case KEY_UP:
            case KEY_DOWN:
                CursorMode = CUR_MODE_VALUE;
                break;
            case KEY_SET:
                if(CursorMode != CUR_MODE_ITEM)
                {
                    CursorMode = CUR_MODE_ITEM;
                }
                else
                {
                    if(++OpMode >= MODE_SYS_END) OpMode = MODE_SYS_END;
                }
                break;
            case KEY_ENT:
                if(CursorMode == CUR_MODE_ITEM)	CursorMode = CUR_MODE_CURSOR;
                else							CursorMode = CUR_MODE_ITEM;
                break;
            case KEY_PUSH:
                if(CursorMode == CUR_MODE_ITEM)			CursorMode = CUR_MODE_CURSOR;
                else if(CursorMode == CUR_MODE_CURSOR)	CursorMode = CUR_MODE_VALUE;
                else if(CursorMode == CUR_MODE_VALUE)	CursorMode = CUR_MODE_CURSOR;
                else									CursorMode = CUR_MODE_ITEM;

                delay_ms(100);
                ClearPhaseCount();
                break;
            }
        }         
        else if((OpMode >= MODE_CAL) && (OpMode <= MODE_CAL_END))
        {
            switch(key)
            {
            case KEY_RUN:
                break;
            case KEY_UP:
                break;
            case KEY_DOWN:
                break;
            case KEY_CAL:
                break;
            case KEY_ENT:
                break;
            }
        }

        if((key == KEY_LEFT) || (key == KEY_RIGHT)) set_pos = key;
        else if((key == KEY_UP) || (key == KEY_DOWN)) set_val = key;

        //if(OpMode != MODE_RUN)
        if(OpMode > MODE_RUN_SET_END)
        {
            Set_Mode(set_val, set_pos, &CursorMode, &OpMode, &OpModeSub);
            /*
			while(!Set_Mode(set_val, set_pos, &CursorMode, &OpMode, &OpModeSub))
            {
                set_val = KEY_NON;
                set_pos = KEY_NON;
                Param = 0;
            }
			*/
        }

        ClearPhaseCount();
    }

    //if(OpMode == MODE_RUN)
    //{
    //    Set_Mode(KEY_NON, KEY_NON, 0, &OpMode, &OpModeSub);
    //}

    ////////////////////////////////////////////////////////////////
    // Encoder Processor
    if(CheckPhaseCount() != 0)
    {
        if(OpMode == MODE_RUN)
        {
            phase = Get_PhaseCount();
            if(phase > 0)
            {
                if(Setup.TestMode == TEST_CONTINUE)
                {
                    if(++DisplayPage > 4) DisplayPage = 4;                    
                }
                else if(Setup.TestMode == TEST_ON_OFF)
                {
#if defined BK2039 || defined BK2039_5CH
                    if(++DisplayPage > 3) DisplayPage = 3;
#endif
#if defined BK2039A || defined BK2039X
                    if(++DisplayPage > 5) DisplayPage = 5;
#endif                    
                }                
            }
            else if(phase < 0)
            {
                if(--DisplayPage < 0) DisplayPage = 0;
            }

            return;
        }
        else if (OpMode == MODE_SET_TEST_MODE)
        {
            phase = Get_PhaseCount();

            if(CursorMode == CUR_MODE_ITEM)
            {
                if(phase > 0)
                {
                    if      (Setup.TestMode == TEST_CONTINUE)   OpMode = MODE_SET_CONTINUE;
                    else if (Setup.TestMode == TEST_ON_OFF)     OpMode = MODE_SET_TEST;
                    else if (Setup.TestMode == TEST_HAPTIC)     OpMode = MODE_SET_HAPTIC;
                }
                else if(phase < 0)
                {
                    OpMode = MODE_SET_TEST_MODE;
                }
            }
            else if(CursorMode == CUR_MODE_CURSOR)
            {
                if(phase > 0)
                    set_pos = KEY_RIGHT;
                else if(phase < 0)
                    set_pos = KEY_LEFT;
            }
            else if(CursorMode == CUR_MODE_VALUE)
            {
                if(phase > 0)
                    set_val = KEY_UP;
                else if(phase < 0)
                    set_val = KEY_DOWN;
            }

            if(OpMode > MODE_RUN_SET_END)
            {
                while(!Set_Mode(set_val, set_pos, &CursorMode, &OpMode, &OpModeSub))
                {
                    set_val = KEY_NON;
                    set_pos = KEY_NON;
                    Param = 0;
                }
            }
        }  
        // Mode Set Continue //////////////////////////////////////////////////
        else if((OpMode >= MODE_SET_CONTINUE)&&(OpMode <= MODE_SET_CONTINUE_END))
        {
            phase = Get_PhaseCount();
            //if(phase != 0)
            //{
            if(CursorMode == CUR_MODE_ITEM)
            {
                if(phase > 0)
                {
                    set_pos = KEY_RIGHT;
                    if(++OpMode >= MODE_SET_CONTINUE_END) OpMode = MODE_SET_CONTINUE_END-1;
                }
                else if(phase < 0)
                {
                    set_pos = KEY_LEFT;
                    if(--OpMode <= MODE_SET_CONTINUE) OpMode = MODE_SET_TEST_MODE;
                }
            }
            else if(CursorMode == CUR_MODE_CURSOR)
            {
                if(phase > 0)
                    set_pos = KEY_RIGHT;
                else if(phase < 0)
                    set_pos = KEY_LEFT;
            }
            else if(CursorMode == CUR_MODE_VALUE)
            {
                if(phase > 0)
                    set_val = KEY_UP;
                else if(phase < 0)
                    set_val = KEY_DOWN;
            }

            if(OpMode > MODE_RUN_SET_END)
            {
                while(!Set_Mode(set_val, set_pos, &CursorMode, &OpMode, &OpModeSub))
                {
                    set_val = KEY_NON;
                    set_pos = KEY_NON;
                    Param = 0;
                }
            }
            //}
        }
        // Mode Set ONOFF //////////////////////////////////////////////////
        else if((OpMode >= MODE_SET_TEST)&&(OpMode <= MODE_SET_TEST_END))
        {
            phase = Get_PhaseCount();
            
            if(CursorMode == CUR_MODE_ITEM)
            {
                if(phase > 0)
                {
                    set_pos = KEY_RIGHT;
                    if(++OpMode >= MODE_SET_TEST_END) OpMode = MODE_SET_TEST_END-1;
                }
                else if(phase < 0)
                {
                    set_pos = KEY_LEFT;
                    if(--OpMode <= MODE_SET_TEST) OpMode = MODE_SET_TEST_MODE;
                }
            }
            else if(CursorMode == CUR_MODE_CURSOR)
            {
                if(phase > 0)
                    set_pos = KEY_RIGHT;
                else if(phase < 0)
                    set_pos = KEY_LEFT;
            }
            else if(CursorMode == CUR_MODE_VALUE)
            {
                if(phase > 0)
                    set_val = KEY_UP;
                else if(phase < 0)
                    set_val = KEY_DOWN;
            }

            if(OpMode > MODE_RUN_SET_END)
            {
                while(!Set_Mode(set_val, set_pos, &CursorMode, &OpMode, &OpModeSub))
                {
                    set_val = KEY_NON;
                    set_pos = KEY_NON;
                    Param = 0;
                }
            }
        }
        // Mode Set Haptic //////////////////////////////////////////////////
        else if((OpMode >= MODE_SET_HAPTIC)&&(OpMode <= MODE_SET_HAPTIC_END))
        {
            phase = Get_PhaseCount();

            if(CursorMode == CUR_MODE_ITEM)
            {
                if(phase > 0)
                {
                    set_pos = KEY_RIGHT;
                    if(++OpMode >= MODE_SET_HAPTIC_END) OpMode = MODE_SET_HAPTIC_END-1;
                }
                else if(phase < 0)
                {
                    set_pos = KEY_LEFT;
                    if(--OpMode < MODE_SET_HAPTIC) OpMode = MODE_SET_TEST_MODE;
                }
            }
            else if(CursorMode == CUR_MODE_CURSOR)
            {
                if(phase > 0)
                    set_pos = KEY_RIGHT;
                else if(phase < 0)
                    set_pos = KEY_LEFT;
            }
            else if(CursorMode == CUR_MODE_VALUE)
            {
                if(phase > 0)
                    set_val = KEY_UP;
                else if(phase < 0)
                    set_val = KEY_DOWN;
            }

            if(OpMode > MODE_RUN_SET_END)
            {
                while(!Set_Mode(set_val, set_pos, &CursorMode, &OpMode, &OpModeSub))
                {
                    set_val = KEY_NON;
                    set_pos = KEY_NON;
                    Param = 0;
                }
            }
        }  
        // Mode Sys //////////////////////////////////////////////////
        else if((OpMode >= MODE_SYS)&&(OpMode <= MODE_SYS_END))
        {
            phase = Get_PhaseCount();

            if(CursorMode == CUR_MODE_ITEM)
            {
                if(phase > 0)
                {
                    set_pos = KEY_RIGHT;
                    if(++OpMode >= MODE_SYS_END) OpMode = MODE_SYS_END-1;
                }
                else if(phase < 0)
                {
                    set_pos = KEY_LEFT;
                    if(--OpMode < MODE_SYS) OpMode = MODE_SYS;
                }
            }
            else if(CursorMode == CUR_MODE_CURSOR)
            {
                if(phase > 0)
                    set_pos = KEY_RIGHT;
                else if(phase < 0)
                    set_pos = KEY_LEFT;
            }
            else if(CursorMode == CUR_MODE_VALUE)
            {
                if(phase > 0)
                    set_val = KEY_UP;
                else if(phase < 0)
                    set_val = KEY_DOWN;
            }

            if(OpMode > MODE_RUN_SET_END)
            {
                while(!Set_Mode(set_val, set_pos, &CursorMode, &OpMode, &OpModeSub))
                {
                    set_val = KEY_NON;
                    set_pos = KEY_NON;
                    Param = 0;
                }
            }
        }
    }
}

void DisplayTestContinue(void)
{
    int 	i;
    int 	seq_num;
    char 	run_on;
    char 	cRes[CUR_TEST_CHANNEL];

    if((OnTestProcess == 1) && (OnMotorPower == 1))
    {
        if(run_tick == 1) 	run_on = '*';
        else  				run_on = ' ';
    }
    else
    {
        run_on = ' ';
    }

    for(i = 0; i < CUR_TEST_CHANNEL; i++)
    {
        if(Res[i].TestCur == CUR_OK)	cRes[i] = ' ';
        else							cRes[i] = 'x';
    }

    if(DisplayPage == 0)
    {
#if defined BK2039 || defined BK2039A || defined BK2039_5CH
        if(Cont.Gen_Func == GEN_SWEEP)
        {
            if((Cont.Gen_Sweep_Mode == SWEEP_LIN_DN) || (Cont.Gen_Sweep_Mode == SWEEP_LOG_UP))
            {
                sprintf(VSTR1, "%c F2 - F2    VOLT", run_on);
                sprintf(VSTR2, "%5.0f-%5.0f  %4.1f", Cont.Gen_Sweep_F2, Cont.Gen_Sweep_F1, Cont.Gen_Amplitude);
            }
            else
            {
                sprintf(VSTR1, "%c F1 - F2    VOLT", run_on);
                sprintf(VSTR2, "%5.0f-%5.0f  %4.1f", Cont.Gen_Sweep_F1, Cont.Gen_Sweep_F2, Cont.Gen_Amplitude);
            }
        }
        else
        {
            sprintf(VSTR1, "%c  FREQ      VOLT", run_on);
            sprintf(VSTR2, "  %5.0f      %4.1f", Cont.Gen_Sweep_F1, Cont.Gen_Amplitude);
        }
    }
#elif BK2039X
        if(Cont[0].Gen_Func == GEN_SWEEP)
        {
            if((Cont[0].Gen_Sweep_Mode == SWEEP_LIN_DN) || (Cont[0].Gen_Sweep_Mode == SWEEP_LOG_UP))
            {
                sprintf(VSTR1, "%c F2 - F2    VOLT", run_on);
                sprintf(VSTR2, "%5.0f-%5.0f  %4.1f", Cont[0].Gen_Sweep_F2, Cont[0].Gen_Sweep_F1, Cont[0].Gen_Amplitude);
            }
            else
            {
                sprintf(VSTR1, "%c F1 - F2    VOLT", run_on);
                sprintf(VSTR2, "%5.0f-%5.0f  %4.1f", Cont[0].Gen_Sweep_F1, Cont[0].Gen_Sweep_F2, Cont[0].Gen_Amplitude);
            }
        }
        else
        {
            sprintf(VSTR1, "%c  FREQ      VOLT", run_on);
            sprintf(VSTR2, "  %5.0f      %4.1f", Cont[0].Gen_Sweep_F1, Cont[0].Gen_Amplitude);
        }
    }
#endif
    
#ifdef BK2039_5CH
    // BK2039, BX6244R10, 10 Ch.
    else if(DisplayPage == 2)
    {
        sprintf(VSTR1, "%cCH1%cCH2%cCH3%cCH4%cCH5",cRes[0],cRes[2],cRes[4],cRes[6],cRes[8]);
        sprintf(VSTR2, "%3d %3d %3d %3d %3d", (int)Res[0].Cur, (int)Res[2].Cur, (int)Res[4].Cur, (int)Res[6].Cur, (int)Res[8].Cur);
    }
#endif
#ifdef BK2039
    // BK2039, BX6244R10, 10 Ch.
    else if(DisplayPage == 2)
    {
        sprintf(VSTR1, "%cCH1%cCH2%cCH3%cCH4%cCH5",cRes[0],cRes[2],cRes[4],cRes[6],cRes[8]);
        sprintf(VSTR2, "%3d %3d %3d %3d %3d", (int)Res[0].Cur, (int)Res[2].Cur, (int)Res[4].Cur, (int)Res[6].Cur, (int)Res[8].Cur);
    }
    else if(DisplayPage == 3)
    {
        sprintf(VSTR1, "%cCH6%cCH7%cCH8%cCH9%cC10",cRes[10],cRes[12],cRes[14],cRes[16],cRes[18]);
        sprintf(VSTR2, "%3d %3d %3d %3d %3d", (int)Res[10].Cur, (int)Res[12].Cur, (int)Res[14].Cur, (int)Res[16].Cur, (int)Res[18].Cur);
    }
#endif
#if defined BK2039A || defined BK2039X
    // BK2039A, BX6244R10, 20 Ch.
    else if(DisplayPage == 1)
    {             
        sprintf(VSTR1,"%cCH1%cCH2%cCH3%cCH4%cCH5",cRes[0],cRes[1],cRes[2],cRes[3],cRes[4]); 
        sprintf(VSTR2," %3d %3d %3d %3d %3d", (int)Res[0].Cur, (int)Res[1].Cur, (int)Res[2].Cur, (int)Res[3].Cur, (int)Res[4].Cur);                
    }
    else if(DisplayPage == 2)
    {
        sprintf(VSTR1,"%cCH6%cCH7%cCH8%cCH9%cC10",cRes[5],cRes[6],cRes[7],cRes[8],cRes[9]); 
        sprintf(VSTR2," %3d %3d %3d %3d %3d", (int)Res[5].Cur, (int)Res[6].Cur, (int)Res[7].Cur, (int)Res[8].Cur, (int)Res[9].Cur);                
    }
    else if(DisplayPage == 3)
    {
        sprintf(VSTR1,"%cC11%cC12%cC13%cC14%cC15",cRes[10],cRes[11],cRes[12],cRes[13],cRes[14]); 
        sprintf(VSTR2," %3d %3d %3d %3d %3d", (int)Res[10].Cur, (int)Res[11].Cur, (int)Res[12].Cur, (int)Res[13].Cur, (int)Res[14].Cur);                     
    }
    else if(DisplayPage == 4)
    {
        sprintf(VSTR1,"%cC16%cC17%cC18%cC19%cC20",cRes[15],cRes[16],cRes[17],cRes[18],cRes[19]); 
        sprintf(VSTR2," %3d %3d %3d %3d %3d", (int)Res[15].Cur, (int)Res[16].Cur, (int)Res[17].Cur, (int)Res[18].Cur, (int)Res[19].Cur);                     
    }
#endif

    LCDUpdate(VSTR1, VSTR2);
}

void DisplayTestOnOff(void)
{
    int  i;
    int  seq_num;
    char str[10];
    char lstr1[10], lstr2[10];
    char run_on;
    char cRes[CUR_TEST_CHANNEL];

    if((OnTestProcess == 1) && (OnMotorPower == 1))
    {
        if(run_tick == 1) run_on = '*';
        else  run_on = ' ';
    }
    else
    {
        run_on = ' ';
    }

    for(i = 0; i < CUR_TEST_CHANNEL; i++)
    {
        if(Res[i].TestCur == CUR_OK)
            cRes[i] = ' ';
        else
            cRes[i] = 'x';
    }

    seq_num = Setup.TestSeq;

    if(DisplayPage == 0)
    {
        //12345678901234567890
        sprintf(VSTR1,"%c COUNT/  TOTAL SEQ", run_on);

        ltoa(Setup.TestCount, str);
        if     (Setup.TestCount > 999999)   sprintf(lstr1,"%s", str);
        else if(Setup.TestCount >  99999)   sprintf(lstr1," %s", str);
        else if(Setup.TestCount >   9999)   sprintf(lstr1,"  %s", str);
        else if(Setup.TestCount >    999)   sprintf(lstr1,"   %s", str);
        else if(Setup.TestCount >     99)   sprintf(lstr1,"    %s", str);
        else if(Setup.TestCount >      9)   sprintf(lstr1,"     %s", str);
        else                                sprintf(lstr1,"      %s", str);

        ltoa(Setup.TestNumber, str);
        if     (Setup.TestNumber > 999999)  sprintf(lstr2,"%s", str);
        else if(Setup.TestNumber >  99999)  sprintf(lstr2," %s", str);
        else if(Setup.TestNumber >   9999)  sprintf(lstr2,"  %s", str);
        else if(Setup.TestNumber >    999)  sprintf(lstr2,"   %s", str);
        else if(Setup.TestNumber >     99)  sprintf(lstr2,"    %s", str);
        else if(Setup.TestNumber >      9)  sprintf(lstr2,"     %s", str);
        else                                sprintf(lstr2,"      %s", str);

        sprintf(VSTR2,"%s/%s   %d", lstr1, lstr2, Setup.TestSeq + 1);
    }
    else if(DisplayPage == 1)
    {
        if(Seq[seq_num].Gen_Func == GEN_SWEEP)
        {
            if((Seq[seq_num].Gen_Sweep_Mode == SWEEP_LIN_DN) || (Seq[seq_num].Gen_Sweep_Mode == SWEEP_LOG_UP))
            {
                sprintf(VSTR1, "%c  F2-F1     VOLT", run_on);
                sprintf(VSTR2, "%5.0f-%5.0f  %4.1f", Seq[seq_num].Gen_Sweep_F2, Seq[seq_num].Gen_Sweep_F1, Seq[seq_num].Gen_Amplitude);
            }
            else
            {
                sprintf(VSTR1, "%c  F1-F2     VOLT",run_on);
                sprintf(VSTR2, "%5.0f-%5.0f  %4.1f", Seq[seq_num].Gen_Sweep_F1, Seq[seq_num].Gen_Sweep_F2, Seq[seq_num].Gen_Amplitude);
            }
        }
        else
        {
            sprintf(VSTR1, "%c  FREQ      VOLT",run_on);
            //sprintf(VSTR2, "  %5.0f      %4.1f", Seq[seq_num].Gen_Sine_Freq, Seq[seq_num].Gen_Amplitude);
            sprintf(VSTR2, "  %5.0f      %4.1f", Seq[seq_num].Gen_Sweep_F1, Seq[seq_num].Gen_Amplitude);
        }
    }
    // BK2039, BX6244R10, 10 Ch.
#ifdef BK2039_5CH
    else if(DisplayPage == 2)
    {
        sprintf(VSTR1, "%cCH1%cCH2%cCH3%cCH4%cCH5", cRes[0], cRes[2], cRes[4], cRes[6], cRes[8]);
        sprintf(VSTR2, "%3d %3d %3d %3d %3d", (int)Res[0].Cur, (int)Res[2].Cur, (int)Res[4].Cur, (int)Res[6].Cur, (int)Res[8].Cur);
    }
#endif
#ifdef BK2039
    else if(DisplayPage == 2)
    {
        sprintf(VSTR1, "%cCH1%cCH2%cCH3%cCH4%cCH5", cRes[0], cRes[2], cRes[4], cRes[6], cRes[8]);
        sprintf(VSTR2, "%3d %3d %3d %3d %3d", (int)Res[0].Cur, (int)Res[2].Cur, (int)Res[4].Cur, (int)Res[6].Cur, (int)Res[8].Cur);
    }
    else if(DisplayPage == 3)
    {
        sprintf(VSTR1, "%cCH6%cCH7%cCH8%cCH9%cC10", cRes[10], cRes[12], cRes[14], cRes[16], cRes[18]);
        sprintf(VSTR2, "%3d %3d %3d %3d %3d", (int)Res[10].Cur, (int)Res[12].Cur, (int)Res[14].Cur, (int)Res[16].Cur, (int)Res[18].Cur);
    }
#endif
#if defined BK2039A || defined BK2039X
    else if(DisplayPage == 2)
    {
        sprintf(VSTR1, "%cCH1%cCH2%cCH3%cCH4%cCH5", cRes[0], cRes[1], cRes[2], cRes[3], cRes[4]); 
        sprintf(VSTR2, " %3d %3d %3d %3d %3d", (int)Res[0].Cur, (int)Res[1].Cur, (int)Res[2].Cur, (int)Res[3].Cur, (int)Res[4].Cur);                
    }
    else if(DisplayPage == 3)
    {
        sprintf(VSTR1, "%cCH6%cCH7%cCH8%cCH9%cC10", cRes[5], cRes[6], cRes[7], cRes[8], cRes[9]);
        sprintf(VSTR2, " %3d %3d %3d %3d %3d", (int)Res[5].Cur, (int)Res[6].Cur, (int)Res[7].Cur, (int)Res[8].Cur, (int)Res[9].Cur);                
    }
    else if(DisplayPage == 4)
    {
        sprintf(VSTR1, "%cC11%cC12%cC13%cC14%cC15", cRes[10], cRes[11], cRes[12], cRes[13], cRes[14]); 
        sprintf(VSTR2, " %3d %3d %3d %3d %3d", (int)Res[10].Cur, (int)Res[11].Cur, (int)Res[12].Cur, (int)Res[13].Cur, (int)Res[14].Cur);                     
    }
    else if(DisplayPage == 5)
    {
        sprintf(VSTR1, "%cC16%cC17%cC18%cC19%cC20", cRes[15], cRes[16], cRes[17], cRes[18], cRes[19]);
        sprintf(VSTR2, " %3d %3d %3d %3d %3d", (int)Res[15].Cur, (int)Res[16].Cur, (int)Res[17].Cur, (int)Res[18].Cur, (int)Res[19].Cur);                     
    }
#endif

    LCDUpdate(VSTR1, VSTR2);
}  

void DisplayTestHaptic(void)
{
    char str[10];
    char lstr1[10], lstr2[10], lstr3[10];
    
    //12345678901234567890
    //HAP/N:   4/  10000
    //TEST:1000000/1000000
     
    ltoa(HapticCount.Test, str);
    if     (HapticCount.Test > 999999)  sprintf(lstr1,"%s", str);
    else if(HapticCount.Test >  99999)  sprintf(lstr1," %s", str);
    else if(HapticCount.Test >   9999)  sprintf(lstr1,"  %s", str);
    else if(HapticCount.Test >    999)  sprintf(lstr1,"   %s", str);
    else if(HapticCount.Test >     99)  sprintf(lstr1,"    %s", str);
    else if(HapticCount.Test >      9)  sprintf(lstr1,"     %s", str);
    else                                sprintf(lstr1,"      %s", str);

    ltoa(HapticSetup.TestNumber, str);
    if     (HapticSetup.TestNumber > 999999)  sprintf(lstr2,"%s", str);
    else if(HapticSetup.TestNumber >  99999)  sprintf(lstr2," %s", str);
    else if(HapticSetup.TestNumber >   9999)  sprintf(lstr2,"  %s", str);
    else if(HapticSetup.TestNumber >    999)  sprintf(lstr2,"   %s", str);
    else if(HapticSetup.TestNumber >     99)  sprintf(lstr2,"    %s", str);
    else if(HapticSetup.TestNumber >      9)  sprintf(lstr2,"     %s", str);
    else                                      sprintf(lstr2,"      %s", str);
    
    ltoa(HapticCount.SigRepeat+1, str);
    if     (HapticCount.SigRepeat > 9999)  sprintf(lstr3,"%s", str);
    else if(HapticCount.SigRepeat >  999)  sprintf(lstr3," %s", str);
    else if(HapticCount.SigRepeat >   99)  sprintf(lstr3,"  %s", str);
    else if(HapticCount.SigRepeat >    9)  sprintf(lstr3,"   %s", str);
    else                                   sprintf(lstr3,"    %s", str);
    
    if(OnTestProcess == 0)
    {
        if(HapticCount.Test >= HapticSetup.TestNumber)
            sprintf(VSTR1,"Haptic Test is Over");
        else                                    
            sprintf(VSTR1,"Push RUN To Start");
    }
    else
    {
        sprintf(VSTR1,"HAP/NUM:   %d/  %s", HapticCount.SigCurrent + 1, lstr3);
    }
    
    sprintf(VSTR2,"TEST:%s/%s", lstr1, lstr2);
    LCDUpdate(VSTR1, VSTR2);

}

void DisplayProcess(void)
{
    if(OnDisplayMain == 0) return;
    
    //if(OpMode != MODE_RUN) return;
    if(OpMode > MODE_RUN_SET_END) return;

    if(Setup.TestMode == TEST_CONTINUE)
    {
        DisplayTestContinue();
    }
    else if(Setup.TestMode == TEST_ON_OFF)
    {
        DisplayTestOnOff();
    }
    else if(Setup.TestMode == TEST_HAPTIC)
    {
        DisplayTestHaptic();
    }
}

void ShowCurrentResult(void)
{
    int i;
    //int value;

    if(OpMode > MODE_RUN_SET_END)
    {
        LED(LED_ALL, LED_OFF);
        return;
    }

    //if(OnTestProcess == 0)
    if((OnTestProcess == 0) || (OnMotorPower == 0))
    {
        LED(LED_ALL, LED_OFF);
        return;
    }

    for(i = 0; i < CUR_TEST_CHANNEL; i++)
    {
        LED(i, Res[i].TestCur);
    }

    // Test Buaaer /////////////////////////////////////
    if(CurTestCh == 0)
    {
        TestResAll = CUR_OK;
        for(i = 0; i < CUR_TEST_CHANNEL; i++)
        {
            if((Res[i].TestCur == CUR_SHT) || (Res[i].TestCur == CUR_NG))
            {
                TestResAll = CUR_NG;
            }
        }
        if(TestResAll == CUR_NG)
        {
            //BuzzerNumber(100,20,2);
            BeepNG();
        }
    }
}

#define ADC_FULL_SCALE  (5.0)
#define ADC_BIT_SCALE   (1024)
#define ADC_FULL_FACTOR (4.8828e-3)   // ADC_FULL_SCALE / ADC_BIT_SCALE
#define SIG_AMP_GAIN    (30)

/*
void DoTestVibration(int* data)
{
    int i;
    float tmpOffset;
    float adc_scale;
    float tmpV;
    float tmpAvg;
    float tmpPwr;
    float tmpRMS;
    float tmpVIB;

    adc_scale = 1.0 / RecordLength;
    tmpAvg = 0;
    for(i=0; i<RecordLength; i++)
    {
        tmpV = data[i];
        tmpV = tmpV * adc_scale;
        tmpAvg += tmpV;
    }
    tmpOffset = tmpAvg;

    adc_scale = ADC_FULL_FACTOR;
    tmpAvg = 0;
    for(i=0; i<RecordLength; i++)
    {
        tmpV = data[i] - tmpOffset;
        tmpV = (tmpV * adc_scale);
        tmpAvg += tmpV * tmpV;
    }

    tmpPwr = tmpAvg / RecordLength;
    tmpRMS = sqrt(tmpPwr);
    tmpRMS = tmpRMS / SIG_AMP_GAIN;


    tmpVIB = tmpRMS / (Setup.VibSensorSens * 0.001);
    if(tmpVIB > 9.9) tmpVIB = 9.9;
    if(tmpVIB < 0.1) tmpVIB = 0.0;
    Res.Vib = tmpVIB;

    //OnDisplayMain = 0;
    //sprintf(VSTR1,"%.3f",tmpRMS);
    //LCDUpdate(VSTR1, VSTR2);

}
*/

#define DEBUG_DoTestCurrent

#define SHUNT 			(0.1)

#if defined BK2039 || defined BK2039X    
    #define CUR_AMP_GAIN    (5.0)
#endif
#ifdef BK2039_5CH
    #define CUR_AMP_GAIN    (20.0)
#endif
#ifdef BK2039A
    #define CUR_AMP_GAIN    (1.16)  // (5.0)
#endif

void DoTestCurrent(int* data, int ch)
{
    int   i;
    float tmpOffset;
    float adc_scale;
    float tmpV;
    float tmpAvg;
    float tmpPwr;
    float tmpVin;
    //float tmpCtV;
    float tmpCUR;
    
    adc_scale = 1.0 / RecordLength;
    tmpAvg    = 0;    
    for(i = 0; i < RecordLength; i++)
    {
        tmpV   =  data[i];
        tmpV   =  tmpV * adc_scale;
        tmpAvg += tmpV;
    }
    tmpOffset = tmpAvg;

#ifdef DEBUG_DoTestCurrent
    printf("CH[%2d] tmpOffset = %f \n", ch + 1, tmpOffset);
#endif

    adc_scale = ADC_FULL_FACTOR;
    tmpAvg    = 0;
    for(i = 0; i < RecordLength; i++)
    {
        tmpV   =  data[i] - tmpOffset;
        tmpV   =  tmpV * adc_scale;
        tmpAvg += tmpV * tmpV;
    }
    tmpPwr = tmpAvg / RecordLength;
    tmpVin = sqrt(tmpPwr);

#ifdef DEBUG_DoTestCurrent
    printf("CH[%2d] tmpVin = %f \n", ch + 1, tmpVin);
#endif

    tmpCUR = tmpVin / CUR_AMP_GAIN;
    //tmpCUR = tmpCUR / SHUNT;
    tmpCUR = tmpCUR * 1000; //mA;

    // Calibration /////////////
    CurOffsetBefore[ch] = tmpCUR;
    tmpCUR = tmpCUR - Cal.Cur_Offset[ch];

    CurSensBefore[ch]   = tmpCUR;
    tmpCUR = tmpCUR * Cal.Cur_Sens[ch];
    
#ifdef DEBUG_DoTestCurrent
    printf("CH[%2d] tmpCUR = %f \n", ch + 1, tmpCUR);
#endif

    //if(tmpCUR > 250)              tmpCUR = 250;
    if(tmpCUR < CUR_OPEN_LIMIT)    tmpCUR = 0;

    Res[ch].Cur = tmpCUR;

    //sprintf(VSTR1,"CH:%2d  ",ch+1);
    //sprintf(VSTR2,"V:%5.3f CUR:%.3f", tmpVin, tmpCUR);
    //LCDUpdate(VSTR1, VSTR2);
    //OnDisplayMain = 0;
}

void DoTestCurrentResult(int ch)
{
    if(Res[ch].Cur < CUR_OPEN_LIMIT)
    {
        Res[ch].TestCur = CUR_OPN;
    }
    else if(Res[ch].Cur > CUR_SHORT_LIMIT)
    {
        Res[ch].TestCur = CUR_SHT;
    }
    else if((Res[ch].Cur < Setup.CurLimitLower) || (Res[ch].Cur > Setup.CurLimitUpper))
    {
        Res[ch].TestCur = CUR_NG;
    }
    else Res[ch].TestCur = CUR_OK;
}


//#define DEBUG_TimerProcess

void TimerProcess(void)
{
    int 	i;
    int 	snum;
    char 	key;

    for(i = 0; i < TIMER_EVENT_NUMBER; i++)
    {
        if(Timer[i].OnEvent == 1)
        {
            if(Timer[i].Event == TIMER_EVENT_MOTOR_ON)
            {
                //if(Setup.TestCount++ >= Setup.TestNumber)
                if(Setup.TestCount >= Setup.TestNumber)
                {
                    StopTest();
                    sprintf(VSTR1,"Test is Over_ ON_OFF");
                    (DISPLAY == VFD) ? sprintf(VSTR2,"Push %c", 0x17) : sprintf(VSTR2,"Push ENT");
                    LCDUpdate(VSTR1, VSTR2);
                    BuzzerNumber(300, 200, 3);
                    key = Get_KEY();
                    while(key != KEY_ENT)
                    {
                        key = Get_KEY();
                    }
                    //Reset_Count();
                }
                else
                {
                    Setup.TestCount++;
                    snum = Setup.TestSeq;

                    if(++Setup.SeqTestCount >= Seq[snum].RepeatNumber)
                    {
                        do
                        {
                            //if(++snum >= SEQUENCE_NUMBER)
                            if(++snum >= SEQ_ON_OFF_NUMBER)
                            {
                                snum = 0;
                            }
                        }
                        while((Seq[snum].Active == 0) && (snum != 0));

                        Setup.TestSeq = snum;
                        Setup.SeqTestCount = 0;
                    }

                    snum = Setup.TestSeq;
                    
                    Setup_GeneratorONOFF(Setup.TestSeq);
                    GEN_MUX_ON();
                    GEN_SignalMTrigger(GEN_CH, 0, GEN_CONTINUE);
                    
                    SetTimerInterval(TIMER_EVENT_MOTOR_ON , Seq[snum].OnTime + Seq[snum].OffTime);
                    SetTimerInterval(TIMER_EVENT_MOTOR_OFF, Seq[snum].OnTime);

                    SetTimerActive(TIMER_EVENT_MOTOR_ON , 1);
                    SetTimerActive(TIMER_EVENT_MOTOR_OFF, 1);

                    OnMotorPower = 1;

                    delay_ms(100);
                    RecordStart(CurTestCh);
                }
            }
            else if(Timer[i].Event == TIMER_EVENT_MOTOR_OFF)
            {
                GEN_SignalMTrigger(GEN_CH, 0, GEN_READY);
                OnMotorPower = 0;
                RecordStop();
            }
            Timer[i].OnEvent = 0;
        }
    }
    //sprintf(VSTR2,"%%3d/%3d, %3d/%3d", Timer[0].count,Timer[0].interval, Timer[1].count,Timer[1].interval);
    //LCDUpdate(VSTR1, VSTR2);
    //OnDisplayMain = 0;
}

void TestCurrentProcess(void)
{
    if((OnTestProcess == 1) && (OnMotorPower == 1))
    {
        if(OnRecordEnd == 1)
        {
            DoTestCurrent(TimeData, CurTestCh);
            DoTestCurrentResult(CurTestCh);
            if(++CurTestCh >= CUR_TEST_CHANNEL)
            {
                CurTestCh = 0;
            }
            RecordStart(CurTestCh);
        }
    }
}

void TimerProcessHapticLED(void)
{
    int i;
    
    if(OnTestProcess == 0)
    {  
        LED(LED_ALL, LED_OFF);
        return;
    }  
    
    for(i = 0; i < TIMER_EVENT_NUMBER; i++)
    {
        if(Timer[i].OnEvent == 1)
        {
            if(Timer[i].Event == TIMER_EVENT_FAST_ON)
            {
                LED(LED_ALL, LED_GREEN);              
                //SetTimerInterval(TIMER_EVENT_FAST_ON , 0.6);
                //SetTimerInterval(TIMER_EVENT_FAST_OFF, 0.3);
                SetTimerActive(TIMER_EVENT_FAST_ON , 1);
                SetTimerActive(TIMER_EVENT_FAST_OFF, 1);
            }
            else if(Timer[i].Event == TIMER_EVENT_FAST_OFF)
            {
                LED(LED_ALL, LED_OFF);
            }
            Timer[i].OnEvent = 0;
        }
    }
}

void TestHapticProcess(void)
{
    //char lstr1[10], lstr2[10];
    unsigned int nSig;
    PHAPTIC_SIGNAL pSig;    
    
    if(OnTestProcess == 0)
    { 
        return;
    }  
    
    if(HapticCount.Test >= HapticSetup.TestNumber)
    {
        StopTest();
        return;  
    }    
    
    if(HAP_CheckSignalRepeatCountEnd() == 1)
    {   
        StopTest_Haptic();
        
        do{            
            ++HapticCount.SigCurrent;
            nSig = HapticCount.SigCurrent;
            pSig = GetHapticSignal(nSig);
            if(HapticCount.SigCurrent >= HAPTIC_SIG_NUMBER) 
            {                              
                HapticCount.Test++;
                HapticCount.SigCurrent = 0;                
                if(HapticCount.Test >= HapticSetup.TestNumber)
                {
                    DisplayTestHaptic();
                    BuzzerNumber(300, 200, 3);
                    return;
                }
            }
        }while((pSig->Active != 1) && (HapticCount.SigCurrent > 0));
        
        HAP_ClearSignalRepeatCount();
        
        HAP_CheckSaveCount();        
        
        StartTest_Haptic();
    }            
                                 
/*    /////////////////////////////////
    nSig = HapticCount.SigCurrent;
    pSig = GetHapticSignal(nSig);        
    sprintf(VSTR1,"SIG/REP/AT: %d/%d/%d", HapticCount.SigCurrent, pSig->RepeatNumber, pSig->Active);
    sprintf(VSTR2,"ST/REP/: %d/%d",HapticCount.SigRepeatStart, HapticCount.SigRepeat);
    LCDUpdate(VSTR1, VSTR2);
    //BuzzerNumber(300, 200, 3);
    //delay_ms(1000);
*/
}

void TestProcess(void)
{
    if(OpMode != MODE_RUN) return;

    if(Setup.TestMode == TEST_CONTINUE)
    {
        TestCurrentProcess();
        ShowCurrentResult();
    }
    else if(Setup.TestMode == TEST_ON_OFF)
    {
        TimerProcess();
        TestCurrentProcess();
        ShowCurrentResult();
    }
    else if(Setup.TestMode == TEST_HAPTIC)
    {
        TestHapticProcess();
        TimerProcessHapticLED();
    }
        
    // Debug ///////////////////////////////
    /*if(OnRecordEnd == 1)
    {
        DoTestCurrent(TimeData,CurTestCh);
        RecordStart(CurTestCh);

        //sprintf(VSTR1,"CH:%2d",CurTestCh+1);
        //sprintf(VSTR2,"CUR:%.3f",Res[CurTestCh].Cur);
        //LCDUpdate(VSTR1, VSTR2);
        //OnDisplayMain = 0;
    }*/
}

void RUN_BK2019(void)
{
    RecordStart(CurTestCh);

    while(1)
    {
        //UART_ReceiveProcess(0);
        ModeProcess();
        TestProcess();
        DisplayProcess();
    }
}

int CUR_CalOffset(void)
{
    int   ch;
    int   phase_count;
    int   OnEnd;
    char  key;
    float CurOffset[CUR_TEST_CHANNEL];
    float GenVolt;

    // BX6244R10 & BX6101R22
    GenVolt = 3.0;// / GAIN_AMP_OUT;
        
    GEN_MUX_OFF();
    
    for(ch = 0; ch < GEN_CH_NUM; ch++)
    {
        GEN_SetRealSignalType(ch, WAVE_SINE);    
        GEN_SetRealSignalFrequency(ch, 175);
        GEN_SetRealSignalAmplitude(ch, GenVolt);
        GEN_RealSignalTrigger(ch, GEN_CONTINUE);
    }
    /*
    GEN_SetRealSignalType(0, WAVE_SINE);    
    GEN_SetRealSignalFrequency(0, 175);
    GEN_SetRealSignalAmplitude(0, GenVolt);
    GEN_RealSignalTrigger(0, GEN_CONTINUE);
    
    GEN_SetRealSignalType(1, WAVE_SINE);    
    GEN_SetRealSignalFrequency(1, 300);
    GEN_SetRealSignalAmplitude(1, GenVolt);
    GEN_RealSignalTrigger(1, GEN_CONTINUE);
    */    
    (DISPLAY == VFD) ? sprintf(VSTR1,"CUR Offset  [SAVE:%c]", 0x17) : sprintf(VSTR1,"CUR Offset  [SAVE:E]");

    sprintf(VSTR2,"CH01:  0.0  [   0.0]");
    LCDUpdate(VSTR1, VSTR2);
    BuzzerNumber(50, 50, 2);
    while(ReadRegKeyButton() != 0x00) {}

    for(ch = 0; ch < CUR_TEST_CHANNEL; ch++)
    {
        CurOffset[ch] = Cal.Cur_Offset[ch];
    }

    ch = 0;
    OnEnd = 0;
    //key = Get_KEY();
    RecordStart_Cal(ch);
    do
    {
        if(OnRecordEnd == 1)
        {
            DoTestCurrent(TimeData, ch);
            RecordStart_Cal(ch);

            (DISPLAY == VFD) ? sprintf(VSTR1,"CUR Offset  [SAVE:%c]", 0x17) : sprintf(VSTR1,"CUR Offset  [SAVE:E]");

            sprintf(VSTR2,"CH%02d:%5.1f  [ %5.1f]", ch+1, Cal.Cur_Offset[ch], CurSensBefore[ch]);
            LCDUpdate(VSTR1, VSTR2);
        }

        if(CheckPhaseCount() != 0)
        {
            phase_count = Get_PhaseCount();
            delay_ms(100);

            if      (phase_count > 0) ch++;
            else if (phase_count < 0) ch--;
            if(ch >= CUR_TEST_CHANNEL) ch = CUR_TEST_CHANNEL - 1;
            else if(ch < 0) ch = 0;
        }

        key = Get_KEY();
        if((key == KEY_SET) || (key == KEY_PUSH))
        {
            Cal.Cur_Offset[ch] = CurOffsetBefore[ch];
            delay_ms(100);
        }
        else if(key == KEY_ENT)
        {
            sprintf(VSTR1,"CUR Offset Save?");

            (DISPLAY == VFD) ? sprintf(VSTR2,"YES: %c   NO: RUN", 0x17) : sprintf(VSTR2,"YES: ENT   NO: RUN");

            LCDUpdate(VSTR1, VSTR2);
            BuzzerNumber(50, 50, 2);
            while(ReadRegKeyButton() != 0x00) {}

            key = Get_KEY();
            while((key != KEY_ENT) && (key != KEY_RUN))
            {
                key = Get_KEY();
            }

            if(key == KEY_ENT)      OnEnd = 1;
            else if(key == KEY_RUN) OnEnd = -1;
        }
    }
    while(OnEnd == 0);

    if(OnEnd == 1)
    {
        sprintf(VSTR1, "CUR Offset Save");
        sprintf(VSTR2, "Please Wait");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerOnOff(500,1000);

        sprintf(VSTR1, "CUR Offset is Saved");

        (DISPLAY == VFD) ? sprintf(VSTR2, "Push %c", 0x17) : sprintf(VSTR2, "Push ENT");

        LCDUpdate(VSTR1, VSTR2);

        key = Get_KEY();
        while(key != KEY_ENT)
        {
            key = Get_KEY();
        }
    }
    else if(OnEnd == -1)
    {
        for(ch = 0; ch < CUR_TEST_CHANNEL; ch++)
        {
            Cal.Cur_Offset[ch] = CurOffset[ch];
        }
    }
    
    return(OnEnd);
}

int CUR_CalSensitivity(void)
{
    int   ch;
    int   phase_count;
    int   OnEnd;
    char  key;
    char  cursor_mode;
    float CurSens[CUR_TEST_CHANNEL];
    float GenVolt;

    // BX6244R10 & BX6101R22
    GenVolt = 3.0;// / GAIN_AMP_OUT;

    GEN_MUX_ON();
    for(ch = 0; ch < GEN_CH_NUM; ch++)
    {
        GEN_SetRealSignalType(ch, WAVE_SINE);    
        GEN_SetRealSignalFrequency(ch, 175);
        GEN_SetRealSignalAmplitude(ch, GenVolt);
        GEN_RealSignalTrigger(ch, GEN_CONTINUE);
    }

    (DISPLAY == VFD) ? sprintf(VSTR1,"CUR Sens    [SAVE:%c]", 0x17) : sprintf(VSTR1,"CUR Sens    [SAVE:E]");

    sprintf(VSTR2,"CH01:1.000  [   0.0]");
    LCDUpdate(VSTR1, VSTR2);
    BuzzerNumber(50, 50, 2);
    while(ReadRegKeyButton() != 0x00) {}

    for(ch = 0; ch < CUR_TEST_CHANNEL; ch++)
    {
        CurSens[ch] = Cal.Cur_Sens[ch];
    }

    ch          = 0;
    OnEnd       = 0;
    cursor_mode = 0;
    key = Get_KEY();
    RecordStart_Cal(ch);
    do
    {
        if(OnRecordEnd == 1)
        {
            DoTestCurrent(TimeData, ch);
            RecordStart_Cal(ch);
            if(cursor_mode == 1)
                sprintf(VSTR1,"CUR Sens    [Cal...]",);
            else
                (DISPLAY == VFD) ? sprintf(VSTR1,"CUR Sens    [SAVE:%c]", 0x17) : sprintf(VSTR1,"CUR Sens    [SAVE:E]");

            sprintf(VSTR2,"CH%02d:%5.3f  [ %5.1f]", ch+1, Cal.Cur_Sens[ch], Res[ch].Cur);
            LCDUpdate(VSTR1, VSTR2);
        }

        if(CheckPhaseCount() != 0)
        {
            phase_count = Get_PhaseCount();
            delay_ms(100);

            if(cursor_mode == 1)
            {
                if      (phase_count > 0)           Cal.Cur_Sens[ch] += 0.01;
                else if (phase_count < 0)           Cal.Cur_Sens[ch] -= 0.01;

                if      (Cal.Cur_Sens[ch] > 9.99)   Cal.Cur_Sens[ch] =  9.99;
                else if (Cal.Cur_Sens[ch] < 0.1)    Cal.Cur_Sens[ch] =  0.1;
            }
            else
            {
                if      (phase_count > 0) ch++;
                else if (phase_count < 0) ch--;

                if      (ch >= CUR_TEST_CHANNEL)    ch = CUR_TEST_CHANNEL - 1;
                else if (ch < 0)                    ch = 0;
            }
        }

        key = Get_KEY();
        if(key == KEY_PUSH)
        {
            if(cursor_mode == 0) cursor_mode = 1;
            else cursor_mode = 0;
            delay_ms(100);
        }
        else if(key == KEY_SET)
        {
            cursor_mode = 0;
            delay_ms(100);
        }
        else if(key == KEY_ENT)
        {
            sprintf(VSTR1,"CUR Sens Save?");

            (DISPLAY == VFD) ? sprintf(VSTR2,"YES: %c   NO: RUN", 0x17) : sprintf(VSTR2,"YES: ENT   NO: RUN");

            LCDUpdate(VSTR1, VSTR2);
            BuzzerNumber(50, 50, 2);
            while(ReadRegKeyButton() != 0x00) {}

            key = Get_KEY();
            while((key != KEY_ENT) && (key != KEY_RUN))
            {
                key = Get_KEY();
            }
                    
            if(key == KEY_ENT) OnEnd = 1;
            else if(key == KEY_RUN) OnEnd = -1;
        }
    }
    while(OnEnd == 0);

    if(OnEnd == 1)
    {
        sprintf(VSTR1, "CUR Sens Save");
        sprintf(VSTR2, "Please Wait");
        LCDUpdate(VSTR1, VSTR2);
        BuzzerOnOff(500, 1000);

        sprintf(VSTR1, "CUR Sens is Saved");
        sprintf(VSTR2, "Push RUN");
        LCDUpdate(VSTR1, VSTR2);

        key = Get_KEY();
        while(key != KEY_RUN)
        {
            key = Get_KEY();
        }
    }
    if(OnEnd == -1)
    {
        for(ch = 0; ch < CUR_TEST_CHANNEL; ch++)
        {
            Cal.Cur_Sens[ch] = CurSens[ch];
        }
    }
    
    return(OnEnd);
}

void CUR_Calibration(void)
{
    char key,keyV,keyE;

    CurrentCalMode = 0;
    if((DAQStatus & DAQ_GENOK) != DAQ_GENOK) return;

    key  = ReadRegKeyButton();
    keyV = key & 0x0F;
    keyE = key & KEY_ACTIVE;

    //if((keyV == KEY_RUN) && (keyE == KEY_ACTIVE))
    if((keyV == KEY_CAL) && (keyE == KEY_ACTIVE))
    {
        sprintf(VSTR1,"Calibration Current?");

        (DISPLAY == VFD) ? sprintf(VSTR2,"YES: %c   NO: RUN", 0x17) : sprintf(VSTR2,"YES: ENT   NO: RUN");

        LCDUpdate(VSTR1, VSTR2);
        BuzzerNumber(50, 50, 2);
        while(ReadRegKeyButton() != 0x00) {}

        key = Get_KEY();
        while((key != KEY_ENT) && (key != KEY_RUN))
        {
            key = Get_KEY();
        }

        if(CUR_CalOffset() == 1)
        {
            CUR_CalSensitivity();
            CurrentCalMode = 1;
        }
    }
}

void Init_System(void)
{
    REG_BUZZER = 0;

    GEN_AMP_STANDBY();
    //REG_AMP_OUT_MUX = 0;

    LED(LED_ALL, LED_OFF);
    Init_LCD();
    UART_ClearRxBuffer();
}

void debug_key(void)
{
    char phase_count;
    int count;
    char key;//, pha,phb;
    char key1;
    count = 0;

    key1 = 0;

    while(1)
    {
        key = Get_KEY();

        if(key == KEY_ENT)
        {
            count = 0;
        }

        if(key != 0xff)
        {
            key1 = key;
        }

        phase_count = Get_PhaseCount();
        if(phase_count == 1) count++;
        else if(phase_count == -1) count--;

        sprintf(VSTR1,"KEY:%02x",key1);
        sprintf(VSTR2,"PH:%3d", count);
        LCDUpdate(VSTR1,VSTR2);
    }
}

void main(void)
{
    //unsigned char i;

    MCU_InitSetup();

    LED(LED_ALL, LED_OFF);

    Init_System();

    Intro_System();

    Check_SystemMemory();  

    Init_Var();

    CreateTimer();

    GEN_InitSetup();

    GEN_VerifySetup();

    //GEN_AMP_ON();

#asm("sei")

    CUR_Calibration();

    CheckSystemStatus();

    LED(LED_ALL, LED_OFF);

    RUN_BK2019();
}

//////////////////////////////////////////////////////////////////
// UART Message Process
/*void UART_MessageProcess(PUART_PACKET pMsg)
{
    char str[10];
    int index;
    int param1;
    int param2;
    int id;
    int iValue;
    float fValue;
    //int aCh;

    //int i;

    str[0] = pMsg->index[0];
    str[1] = pMsg->index[1];
    str[2] = pMsg->index[2];
    str[3] = 0;

    index = atoi(str);
    param1 = pMsg->parameter[0] - '0';
    param2 = pMsg->parameter[1] - '0';

    id = pMsg->data[UART_PACKET_DATA_LENGTH-1] - '0';
    pMsg->Ter = 0;

    ////////////////////////////////////////
            printf("Str:%c\n",pMsg->Str);
            printf("head:%c\n",pMsg->head);
            printf("index[0]:%c\n",str[0]);
            printf("index[1]:%c\n",str[1]);
            printf("index[2]:%c\n",str[2]);
            printf("parameter1:%d\n",param1);
            printf("parameter2:%d\n",param2);
            printf("data:");
            for(i=0; i<UART_PACKET_DATA_LENGTH; i++)
                printf("%c",pMsg->data[i]);
            printf("\n");
            printf("Ter:%c\n",pMsg->Ter);
    ///////////////////////////////

    if(pMsg->head == 'C')
    {
        switch(index)
        {
        default:

            break;

        }
    }
}*/


//UART_PACKET rData;
//unsigned char buff[20];
/*int UART_ReceiveProcess(char Str)
{
    int i;
    int len;
    int index;

    len = rx_counter0;
    if(len < UART_PACKET_LENGTH) return(0);

    do
    {
        for(i=0; i<UART_PACKET_LENGTH; i++)
        {
            buff[i] = getch();
            if((buff[i] == CR) && (i < UART_PACKET_LENGTH-1)) return(0);
        }

        /////////////////////////////////////////////////////////
        // Debug
        //buff[i] = 0;
        //printf(">%s",buff);
        //printf(">%s[%d:%d:%d]",buff,rx_wr_index0,rx_rd_index0,rx_counter0);

        index = 0;
        if((buff[15]=='>') && (buff[16]==CR))
        {
            if(buff[index]=='<')
            {
                rData.Str           = buff[index++];
                rData.head          = buff[index++];
                rData.index[0]      = buff[index++];
                rData.index[1]      = buff[index++];
                rData.index[2]      = buff[index++];
                rData.parameter[0]  = buff[index++];
                rData.parameter[1]  = buff[index++];

                for(i=0; i<UART_PACKET_DATA_LENGTH; i++)
                    rData.data[i] = buff[index++];

                rData.Ter = buff[index];

                //////////////////////////////////////////
                    printf("Str:%c\n",rData.Str);
                    printf("head:%c\n",rData.head);
                    printf("index[0]:%c\n",rData.index[0]);
                    printf("index[1]:%c\n",rData.index[1]);
                    printf("index[2]:%c\n",rData.index[2]);
                    printf("parameter1:%c\n",rData.parameter[0]);
                    printf("parameter2:%c\n",rData.parameter[1]);
                    printf("data:");
                    for(i=0; i<UART_PACKET_DATA_LENGTH; i++)
                        printf("%c",rData.data[i]);
                    printf("\n");
                    printf("Ter:%c\n",rData.Ter);
                ///////////////////////////////////////////

                UART_MessageProcess((PUART_PACKET)&rData);
            }

        }

        len = rx_counter0;
    }
    while(len >= UART_PACKET_LENGTH);

    return(1);
}*/

/*int UART_ReceiveData(void)
{
    int i;
    int len;
    int index;

    len = rx_counter0;
    if(len < UART_PACKET_LENGTH) return(0);

    do
    {
        for(i=0; i<UART_PACKET_LENGTH; i++)
        {
            buff[i] = getch();
            if((buff[i] == CR) && (i < UART_PACKET_LENGTH-1)) return(0);
        }

        /////////////////////////////////////////////////////////
        // Debug
        //buff[i] = 0;
        //printf(">%s",buff);
        //printf(">%s[%d:%d:%d]",buff,rx_wr_index0,rx_rd_index0,rx_counter0);

        index = 0;
        if((buff[15]=='>') && (buff[16]==CR))
        {
            if(buff[index]=='<')
            {
                rData.Str           = buff[index++];
                rData.head          = buff[index++];
                rData.index[0]      = buff[index++];
                rData.index[1]      = buff[index++];
                rData.index[2]      = buff[index++];
                rData.parameter[0]  = buff[index++];
                rData.parameter[1]  = buff[index++];

                for(i=0; i<UART_PACKET_DATA_LENGTH; i++)
                    rData.data[i] = buff[index++];

                rData.Ter = buff[index];

                //////////////////////////////////////////
                    printf("Str:%c\n",rData.Str);
                    printf("head:%c\n",rData.head);
                    printf("index[0]:%c\n",rData.index[0]);
                    printf("index[1]:%c\n",rData.index[1]);
                    printf("index[2]:%c\n",rData.index[2]);
                    printf("parameter1:%c\n",rData.parameter[0]);
                    printf("parameter2:%c\n",rData.parameter[1]);
                    printf("data:");
                    for(i=0; i<UART_PACKET_DATA_LENGTH; i++)
                        printf("%c",rData.data[i]);
                    printf("\n");
                    printf("Ter:%c\n",rData.Ter);
                ///////////////////////////////////////////

                UART_MessageProcess((PUART_PACKET)&rData);
            }

        }

        len = rx_counter0;
    }
    while(len >= UART_PACKET_LENGTH);

    return(1);
}*/

