#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "NSX/noise_suppression_x.h"
#include "mem.h"
#include "NSX/signal_processing_library.h"

struct WaveRecorderParam
{
    uint16_t    NumChannels;
    uint32_t    SampleRate;
    uint16_t    BitsPerSample;
} static wave_recorder_param;


/* Function prototype declaration */
void SYS_Init ( void );
void UART0_Init ( void );
void I2S_Init ( void );
void Timer0_Init ( void );
void demo_LineIn ( void );
void Sys_TickInit ( void );

volatile uint32_t nNopTime = 0;
volatile uint8_t nRxBuffer0 = 0;
volatile uint8_t nRxBuffer1 = 0;
volatile uint8_t nTxBuffer0 = 0;
volatile uint8_t nTxBuffer1 = 0;
//8000￡?NR_PART 12
uint16_t nSampleRate = 32000;
uint16_t nSamplePer10us = 160;
uint16_t nSampleLength = 320;

uint8_t *pPcmRxBuff = NULL;
uint8_t *pPcmTxBuff = NULL;
uint8_t *pPcmRxBuffHead = NULL;
uint8_t *pPcmRxBuffTail = NULL;
uint8_t *pPcmTxBuffHead = NULL;
uint8_t *pPcmTxBuffTail = NULL;

uint8_t *pLinRxBuff[2] = {NULL, NULL};
uint8_t *pOutTmpBuff[2] = {NULL, NULL};
uint8_t *pTmpTxBuff[2] = {NULL, NULL};
uint8_t *shBufferOut[2] = {NULL, NULL};
//first1
short shInL[160], shInH[160];
short shOutL[160], shOutH[160];
int  filter_state1[6], filter_state12[6];
int  Synthesis_state1[6], Synthesis_state12[6];

uint8_t bStart = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main ( void )
{
    int num11 = 0;
    int num22 = 0;
    /* Init System, IP clock and multi-function I/O */
    int i = 0;
    uint8_t err;
    NsxHandle *pNsxHandle = NULL;
    SYS_Init();
    Timer0_Init();
    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();
    /* FIXME: configure wave recorder parameters here. */
    wave_recorder_param.NumChannels = 2;
    wave_recorder_param.SampleRate = nSampleRate;
    wave_recorder_param.BitsPerSample = 16;
    nSamplePer10us = nSampleRate / 100;
    nSampleLength = nSamplePer10us * wave_recorder_param.BitsPerSample / 8  * wave_recorder_param.NumChannels;
    pPcmRxBuff = ( uint8_t * ) ARM_Alloc ( 2 * nSampleLength );
    pPcmTxBuff = ( uint8_t * ) ARM_Alloc ( 2 * nSampleLength );
    pPcmRxBuffHead = pPcmRxBuff;
    pPcmRxBuffTail = pPcmRxBuff + nSampleLength ;
    pPcmTxBuffHead = pPcmTxBuff;
    pPcmTxBuffTail = pPcmTxBuff + nSampleLength;
    pLinRxBuff[0] = ARM_Alloc ( nSampleLength / wave_recorder_param.NumChannels );
    pLinRxBuff[1] = ARM_Alloc ( nSampleLength / wave_recorder_param.NumChannels );
    pOutTmpBuff[0] = ARM_Alloc ( nSampleLength / wave_recorder_param.NumChannels );
    pOutTmpBuff[1] = ARM_Alloc ( nSampleLength / wave_recorder_param.NumChannels );
    pTmpTxBuff[0] = ARM_Alloc ( nSampleLength );
    pTmpTxBuff[1] = ARM_Alloc ( nSampleLength );
    //32K
    shBufferOut[0] = ARM_Alloc ( 2 * nSampleLength );
    shBufferOut[1] = ARM_Alloc ( 2 * nSampleLength );
    /* Init I2S, IP clock and multi-function I/O */
    I2S_Init();
    I2S_Open ( I2S,
               I2S_MODE_MASTER,
               wave_recorder_param.SampleRate,
               wave_recorder_param.BitsPerSample == 8 ? I2S_DATABIT_8 :
               wave_recorder_param.BitsPerSample == 16 ? I2S_DATABIT_16 :
               wave_recorder_param.BitsPerSample == 24 ? I2S_DATABIT_24 : I2S_DATABIT_32,
               wave_recorder_param.NumChannels == 1 ? I2S_MONO : I2S_STEREO,
               I2S_FORMAT_I2S,
               I2S_ENABLE_INTERNAL_CODEC );
    // Open MCLK
    I2S_EnableMCLK ( I2S, wave_recorder_param.SampleRate * 256 );
    I2S_SET_TX_TH_LEVEL ( I2S, I2S_FIFO_TX_LEVEL_WORD_15 );
    I2S_SET_RX_TH_LEVEL ( I2S, I2S_FIFO_RX_LEVEL_WORD_16 );
    I2S_SET_TXDMA_STADDR ( I2S, ( uint32_t ) pPcmTxBuffHead );               // Tx Start Address
    I2S_SET_TXDMA_THADDR ( I2S, ( uint32_t ) ( pPcmTxBuffHead + ( nSampleLength - 4 ) ) ); // Tx Threshold Address
    I2S_SET_TXDMA_EADDR ( I2S, ( uint32_t ) ( pPcmTxBuffTail + ( nSampleLength - 4 ) ) ); // Tx End Address
    I2S_SET_RXDMA_STADDR ( I2S, ( uint32_t ) pPcmRxBuffHead );               // Rx Start Address
    I2S_SET_RXDMA_THADDR ( I2S, ( uint32_t ) ( pPcmRxBuffHead + ( nSampleLength - 4 ) ) ); // Rx Threshold Address
    I2S_SET_RXDMA_EADDR ( I2S, ( uint32_t ) ( pPcmRxBuffTail + ( nSampleLength - 4 ) ) ); // Rx End Address
    demo_LineIn();
    I2S_ENABLE_RXDMA ( I2S );
    I2S_ENABLE_RX ( I2S );
    I2S_EnableInt ( I2S, ( I2S_IEN_RDMATIEN_Msk | I2S_IEN_RDMAEIEN_Msk ) );
    I2S_ENABLE_TXDMA ( I2S );
    I2S_ENABLE_TX ( I2S );
    I2S_EnableInt ( I2S, ( I2S_IEN_TDMATIEN_Msk | I2S_IEN_TDMAEIEN_Msk ) );
    NVIC_EnableIRQ ( I2S_IRQn );
    WebRtcNsx_Create ( &pNsxHandle );
    WebRtcNsx_Init ( pNsxHandle, nSampleRate );
    WebRtcNsx_set_policy ( pNsxHandle, 5 );

    while ( 1 )
    {
        while ( nRxBuffer0 || nRxBuffer1 )
        {
            if ( nRxBuffer0 )
            {
                for ( i = 0; i < nSamplePer10us; i++ )
                {
                    ( ( short * ) pLinRxBuff[0] ) [i] = ( ( short * ) pPcmRxBuffHead ) [2 * i];

                    if ( ( ( short * ) pLinRxBuff[0] ) [i] == '\0' )
                    {
                        num11++;
                    }
                }

                if ( num11 <= 5 )
                {
                    nRxBuffer0 = 0;
                    WebRtcSpl_AnalysisQMF ( ( const short * ) pLinRxBuff[0], 320, shInL, shInH, filter_state1, filter_state12 );

                    if ( 0 == WebRtcNsx_Process ( pNsxHandle , shInL, shInH , shOutL , shOutH ) )
                    {
                        WebRtcSpl_SynthesisQMF ( shOutL, shOutH, 160, ( short * ) shBufferOut[0], Synthesis_state1, Synthesis_state12 );

                        for ( i = 0; i < nSamplePer10us; i++ )
                        {
                            ( ( short * ) pTmpTxBuff[0] ) [2 * i] = ( ( short * ) shBufferOut[0] ) [i];
                        }

                        nTxBuffer0 = 1;
                    }
                }

                else
                {
                    memset ( pTmpTxBuff[0], 0, nSampleLength );
                    num11 = 0;
                    nTxBuffer0 = 1;
                }
            }

            if ( nRxBuffer1 )
            {
                for ( i = 0; i < nSamplePer10us; i++ )
                {
                    ( ( short * ) pLinRxBuff[1] ) [i] = ( ( short * ) pPcmRxBuffTail ) [2 * i];

                    if ( ( ( short * ) pLinRxBuff[1] ) [i] == '\0' )
                    {
                        num22++;
                    }
                }

                if ( num22 <= 5 )
                {
                    nRxBuffer1 = 0;
                    WebRtcSpl_AnalysisQMF ( ( const short * ) pLinRxBuff[1], 320, shInL, shInH, filter_state1, filter_state12 );

                    if ( 0 == WebRtcNsx_Process ( pNsxHandle , shInL  , shInH , shOutL , shOutH ) )
                    {
                        WebRtcSpl_SynthesisQMF ( shOutL, shOutH, 160, ( short * ) shBufferOut[1], Synthesis_state1, Synthesis_state12 );

                        for ( i = 0; i < nSamplePer10us; i++ )
                        {
                            ( ( short * ) pTmpTxBuff[1] ) [2 * i] = ( ( short * ) shBufferOut[1] ) [i];
                        }

                        nTxBuffer1 = 1;
                    }
                }

                else
                {
                    memset ( pTmpTxBuff[1], 0, nSampleLength );
                    num22 = 0;
                    nTxBuffer1 = 1;
                }
            }
        }

        nNopTime ++;
    };

    WebRtcNsx_Free ( pNsxHandle );
}

void I2S_IRQHandler ( void )
{
    uint32_t u32I2SIntFlag;
    u32I2SIntFlag = I2S_GET_INT_FLAG ( I2S, ( I2S_STATUS_RDMATIF_Msk | I2S_STATUS_RDMAEIF_Msk | I2S_IEN_TDMATIEN_Msk | I2S_IEN_TDMAEIEN_Msk ) );

    /* Copy RX data to TX buffer */
    if ( u32I2SIntFlag & I2S_STATUS_RDMATIF_Msk )
    {
        I2S_CLR_INT_FLAG ( I2S, I2S_STATUS_RDMATIF_Msk ); //I2S_STATUS_RDMATIF_Msk  5
        nRxBuffer0 = 1;
    }

    else if ( u32I2SIntFlag & I2S_STATUS_RDMAEIF_Msk )
    {
        I2S_CLR_INT_FLAG ( I2S, I2S_STATUS_RDMAEIF_Msk ); //I2S_STATUS_RDMAEIF_Msk 4
        nRxBuffer1 = 1;
    }

    else if ( u32I2SIntFlag & I2S_IEN_TDMATIEN_Msk )
    {
        I2S_CLR_INT_FLAG ( I2S, I2S_IEN_TDMATIEN_Msk ); //I2S_IEN_TDMATIEN_Msk 7
        memcpy ( pPcmTxBuffHead, pTmpTxBuff[0], nSampleLength );
        nTxBuffer0 = 0;
    }

    else if ( u32I2SIntFlag & I2S_IEN_TDMAEIEN_Msk )
    {
        I2S_CLR_INT_FLAG ( I2S, I2S_IEN_TDMAEIEN_Msk ); //I2S_IEN_TDMAEIEN_Msk 6
        memcpy ( pPcmTxBuffTail, pTmpTxBuff[1], nSampleLength );
        nTxBuffer1 = 0;
    }
}



void demo_LineIn()
{
    uint32_t i;
    // Setting Right Line In Channel
    SYS->GPD_MFPL  = ( SYS->GPD_MFPL & ( ~SYS_GPD_MFPL_PD4MFP_Msk ) ) | SYS_GPD_MFPL_PD4MFP_RLINEIN;
    SYS_SetSharedPinType ( SYS_PORT_D, 4, 0, 0 );
    /* IIC Configure Step without PLL: */
    /* Add MCLK(256*Fs) in. */
    I2S_SET_INTERNAL_CODEC ( I2S, 0x08, 0x1F );	// Mute headphone of Left channel
    I2S_SET_INTERNAL_CODEC ( I2S, 0x09, 0x1F );	// Mute headphone of Right channel
    I2S_SET_INTERNAL_CODEC ( I2S, 0x10, 0x0F );	//Mute the ADC Left channel volume
    I2S_SET_INTERNAL_CODEC ( I2S, 0x11, 0x0F );	//Mute the ADC Right channel volume
    I2S_SET_INTERNAL_CODEC ( I2S, 0x12, 0x0F );	//Mute the ADC Side tone volume
    I2S_SET_INTERNAL_CODEC ( I2S, 0x02, 0xC0 );	//Set CODEC slave
    I2S_SET_INTERNAL_CODEC ( I2S, 0x01, 0x80 );	//Digital Part Enable
    I2S_SET_INTERNAL_CODEC ( I2S, 0x0F, 0xF0 );	//Enable Analog Part
    I2S_SET_INTERNAL_CODEC ( I2S, 0x0E, 0x00 );	//ADC input select Line in
    I2S_SET_INTERNAL_CODEC ( I2S, 0x0B, 0xF3 );	//Analog Part Enable
    I2S_SET_INTERNAL_CODEC ( I2S, 0x0D, 0x31 );	//Biasing enable
    I2S_SET_INTERNAL_CODEC ( I2S, 0x0B, 0xE3 );

    for ( i = 0; i < 15; i++ )	//Delay 1.5s~2.5s
    {
        CLK_SysTickDelay ( 100000 );
    }

    I2S_SET_INTERNAL_CODEC ( I2S, 0x0A, 0x09 );
    I2S_SET_INTERNAL_CODEC ( I2S, 0x0B, 0xF0 );
    I2S_SET_INTERNAL_CODEC ( I2S, 0x00, 0xD0 );	//ADC digital enabled
    CLK_SysTickDelay ( 100000 );	//Delay 100mS
    I2S_SET_INTERNAL_CODEC ( I2S, 0x08, 0x06 );	//Un-mute Headphone and set volume
    I2S_SET_INTERNAL_CODEC ( I2S, 0x09, 0x06 );	//Un-mute Headphone and set volume
    I2S_SET_INTERNAL_CODEC ( I2S, 0x10, 0x00 );	//Un-Mute the ADC Left channel volume
    I2S_SET_INTERNAL_CODEC ( I2S, 0x11, 0x00 );	//Un-Mute the ADC Right channel volume
    //	I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume
    /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */
}


void SYS_Init ( void )
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    //SYS_UnlockReg();
    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;
    //CLK_SetCoreClock(FREQ_96MHZ);
    CLK_SetCoreClock ( 100000000 );
    /* PCLK divider */
    CLK_SetModuleClock ( PCLK_MODULE, NULL, 1 );
    SystemCoreClockUpdate();
}

void TMR0_IRQHandler ( void )
{
    uint32_t nUsageTime = 0;

    while ( nNopTime > 12500000 )
    {
        nNopTime -= 12500000;
    }

    nUsageTime = 12500000 - nNopTime;
    printf ( "CPU Usage:%d.%02d%%\n", nUsageTime / 125000, nUsageTime / 1250 % 100 );
    nNopTime = 0;
    // clear timer interrupt flag
    TIMER_ClearIntFlag ( TIMER0 );
    /* To avoid the synchronization issue between system and APB clock domain */
    TIMER_GetIntFlag ( TIMER0 );
    TIMER_Start ( TIMER0 );
}

void Timer0_Init ( void )
{
    /* Enable IP clock */
    CLK_EnableModuleClock ( TMR0_MODULE );
    /* Select IP clock source */
    CLK_SetModuleClock ( TMR0_MODULE, CLK_TMR0_SRC_EXT, 0 );
    TIMER_Open ( TIMER0, TIMER_PERIODIC_MODE, 1 );
    // Enable timer interrupt
    TIMER_EnableInt ( TIMER0 );
    NVIC_EnableIRQ ( TMR0_IRQn );
    // Start Timer 0
    TIMER_Start ( TIMER0 );
}
void UART0_Init ( void )
{
    /* Enable UART0 Module clock */
    CLK_EnableModuleClock ( UART0_MODULE );
    /* UART0 module clock from EXT */
    CLK_SetModuleClock ( UART0_MODULE, CLK_UART0_SRC_EXT, 0 );
    /* Reset IP */
    SYS_ResetModule ( UART0_RST );
    /* Configure UART0 and set UART0 Baud-rate */
    UART_Open ( UART0, 115200 );
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = ( SYS->GPB_MFPL & ( ~SYS_GPB_MFPL_PB0MFP_Msk ) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = ( SYS->GPB_MFPL & ( ~SYS_GPB_MFPL_PB1MFP_Msk ) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;
}


void I2S_Init ( void )
{
    /* Enable I2S Module clock */
    CLK_EnableModuleClock ( I2S_MODULE );
    /* I2S module clock from APLL */

    if ( wave_recorder_param.SampleRate % 11025 )
    {
        // APLL = 49152031Hz
        CLK_SET_APLL ( CLK_APLL_49152031 );
        //CLK_SysTickDelay(500);    //delay 5 seconds
        // I2S = 49152031Hz / (0+1) = 49152031Hz for 8k, 12k, 16k, 24k, 32k, 48k, and 96k sampling rate
        CLK_SetModuleClock ( I2S_MODULE, CLK_I2S_SRC_APLL, 0 );
    }

    else
    {
        // APLL = 45158425Hz
        CLK_SET_APLL ( CLK_APLL_45158425 );
        //CLK_SysTickDelay(500);    //delay 5 seconds
        // I2S = 45158425Hz / (0+1) = 45158425Hz for 11025, 22050, and 44100 sampling rate
        CLK_SetModuleClock ( I2S_MODULE, CLK_I2S_SRC_APLL, 0 );
    }

    /* Reset IP */
    SYS_ResetModule ( I2S_RST );
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for I2S */
    // GPC[8]  = MCLK
    // GPC[9]  = DIN
    // GPC[10] = DOUT
    // GPC[11] = LRCLK
    // GPC[12] = BCLK
    SYS->GPC_MFPH  = ( SYS->GPC_MFPH & ( ~SYS_GPC_MFPH_PC8MFP_Msk ) ) | SYS_GPC_MFPH_PC8MFP_I2S_MCLK;
    SYS->GPC_MFPH  = ( SYS->GPC_MFPH & ( ~SYS_GPC_MFPH_PC9MFP_Msk ) ) | SYS_GPC_MFPH_PC9MFP_I2S_DIN;
    SYS->GPC_MFPH  = ( SYS->GPC_MFPH & ( ~SYS_GPC_MFPH_PC10MFP_Msk ) ) | SYS_GPC_MFPH_PC10MFP_I2S_DOUT;
    SYS->GPC_MFPH  = ( SYS->GPC_MFPH & ( ~SYS_GPC_MFPH_PC11MFP_Msk ) ) | SYS_GPC_MFPH_PC11MFP_I2S_LRCLK;
    SYS->GPC_MFPH  = ( SYS->GPC_MFPH & ( ~SYS_GPC_MFPH_PC12MFP_Msk ) ) | SYS_GPC_MFPH_PC12MFP_I2S_BCLK;
}

