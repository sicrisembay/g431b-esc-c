/*
 * qpc_hooks.c
 *
 *  Created on: Oct 19, 2025
 *      Author: Sicris
 */

#include "stm32g4xx_hal.h"
#include "qpc.h"
#include "qs_pkg.h"
#include "appPubList.h"
#include "main.h"
#include "uart.h"
#include "string.h"
#include "AO_canopen.h"

static QSpyId const l_Tim3Tick_Handler = { 0U };
/*
 * small size pool
 */
static QF_MPOOL_EL(QEvt) smallPoolSto[32];

/*
 * medium size pool
 */
typedef struct {
    QEvt super;
    uint8_t data[16];
} mediumPool;
static QF_MPOOL_EL(mediumPool) mediumPoolSto[32];

/*
 * large size pool
 */
typedef struct {
    QEvt super;
    uint8_t data[32];
} largePool;
static QF_MPOOL_EL(largePool) largePoolSto[32];

/*
 * Storage for Publish-Subscribe
 */
static QSubscrList subscrSto[MAX_PUB_SIG];


static uint8_t qsTxBuf[1024];
static uint8_t qsRxBuf[512];
static QSTimeCtr tick_ms = 0;

void QPC_start(void)
{
    /*
     * Initialize QF framework
     */
    QF_init();

    /*
     * Initialize Event Pool
     * Note: QF can manage up to three event pools (e.g., small, medium, and large events).
     * An application may call this function up to three times to initialize up to three event
     * pools in QF.  The subsequent calls to QF_poolInit() function must be made with
     * progressively increasing values of the evtSize parameter.
     */
    QF_poolInit(smallPoolSto, sizeof(smallPoolSto), sizeof(smallPoolSto[0]));
    QF_poolInit(mediumPoolSto, sizeof(mediumPoolSto), sizeof(mediumPoolSto[0]));
    QF_poolInit(largePoolSto, sizeof(largePoolSto), sizeof(largePoolSto[0]));

    /*
     * Initialize Publish-Subscribe
     */
    QF_psInit(subscrSto, Q_DIM(subscrSto));

    QS_INIT((void *)0);

    extern UART_HandleTypeDef huart2;   // defined in main.c
    extern DMA_HandleTypeDef hdma_usart2_rx;
    extern DMA_HandleTypeDef hdma_usart2_tx;
    UART_ctor(&huart2, &hdma_usart2_tx, &hdma_usart2_rx);

    extern FDCAN_HandleTypeDef hfdcan1;
    CANOPEN_ctor(&hfdcan1, FDCAN1_IT0_IRQn);

    QF_run();
}


void QF_onStartup(void)
{
    tick_ms = 0U;

    /*
     * Start TIM3
     */
    extern TIM_HandleTypeDef htim3;
    HAL_TIM_Base_Start_IT(&htim3);
    __HAL_DBGMCU_FREEZE_TIM3();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        tick_ms++;
        QTIMEEVT_TICK_X(0U, &l_Tim3Tick_Handler);
    }
}

void QF_onCleanup(void)
{
}

void Q_onError(char const *module, int loc)
{
    uint8_t const * pBlock;
    uint8_t txBuffer[64];
    uint16_t txLen;

    /* Disable Global interrupt */
    __asm volatile ("cpsid i");

    /* Flush QSpy buffer with interrupt disabled! */
    while(1) {
        txLen = sizeof(txBuffer);
        pBlock = QS_getBlock(&txLen);
        if(txLen > 0) {
            extern UART_HandleTypeDef huart2;   // defined in main.c
            HAL_UART_Transmit(&huart2, pBlock, txLen, 1000U);
        } else {
            break;
        }
    }

    QS_ASSERTION(module, loc, (uint32_t)10000U); /* report assertion to QS */

    /* Flush QSpy buffer with interrupt disabled! */
    while(1) {
        txLen = sizeof(txBuffer);
        pBlock = QS_getBlock(&txLen);
        if(txLen > 0) {
            extern UART_HandleTypeDef huart2;   // defined in main.c
            HAL_UART_Transmit(&huart2, pBlock, txLen, 1000U);
        } else {
            break;
        }
    }

    do {
        volatile uint32_t* ARM_CM_DHCSR =  ((volatile uint32_t*) 0xE000EDF0UL); /* Cortex M CoreDebug->DHCSR */
        if ( (*ARM_CM_DHCSR) & 1UL ) __asm("BKPT #0\n"); /* Only halt mcu if debugger is attached */
    } while(0);

#ifndef NDEBUG
    /* for debugging, hang on in an endless loop... */
    for (;;) {
    }
#endif

    NVIC_SystemReset();
}


void QXK_onIdle(void)
{
    uint8_t const * pBlock;
    uint8_t txBuffer[64];
    uint16_t txLen = sizeof(txBuffer);

    QF_CRIT_STAT;
    QF_CRIT_ENTRY();
    pBlock = QS_getBlock(&txLen);
    if(txLen > 0) {
        memcpy(txBuffer, pBlock, txLen);
    }
    QF_CRIT_EXIT();

    if(txLen > 0) {
        UART_send((uint8_t *)pBlock, txLen);
    }
}


/************** QSPY Related section ***************************/
uint8_t QS_onStartup(void const *arg)
{
    (void)arg;  // unused parameter
    QS_initBuf(qsTxBuf, sizeof(qsTxBuf));
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

    QS_FILTER_ON(QS_QEP_STATE_ENTRY);
    QS_FILTER_ON(QS_UA_RECORDS);

    QS_LOC_FILTER(-QS_ALL_IDS);
    QS_LOC_FILTER(AO_CANOPEN_PRIORITY);


    return (uint8_t)1; /* return success */
}


void QS_onCleanup(void)
{
    /// TODO
}


QSTimeCtr QS_onGetTime(void)
{
    return tick_ms;
}


void QS_onFlush(void)
{
    /*
     * QS buffer flushing to HW peripheral
     * Note: This is a blocking function
     */
    uint8_t const * pBlock;
    uint8_t txBuffer[64];
    uint16_t txLen;

    while(1) {
        txLen = sizeof(txBuffer);
        pBlock = QS_getBlock(&txLen);
        if(txLen > 0) {
            extern UART_HandleTypeDef huart2; // defined in main.c
            HAL_UART_Transmit(&huart2, pBlock, txLen, 1000U);
        } else {
            break;
        }
    }
}


void QS_onReset(void)
{
    NVIC_SystemReset();
}


void QS_onCommand(uint8_t cmdId,
                  uint32_t param1, uint32_t param2, uint32_t param3)
{
    (void)cmdId;
    (void)param1;
    (void)param2;
    (void)param3;
}

