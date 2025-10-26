/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of <https://github.com/CANopenNode/CANopenNode>, a CANopen Stack.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this
 * file except in compliance with the License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */

#include "stm32g4xx_hal.h"
#include "qpc.h"
#include "301/CO_driver.h"

Q_DEFINE_THIS_FILE

static CO_CANmodule_t * pModule = NULL;

void
CO_CANsetConfigurationMode(void* CANptr) {
    if(CANptr != NULL) {
        /* Put CAN module in configuration mode */
        FDCAN_HandleTypeDef * hfdcan = (FDCAN_HandleTypeDef *)CANptr;
        if(hfdcan->State == HAL_FDCAN_STATE_BUSY) {
            const HAL_StatusTypeDef ret = HAL_FDCAN_Stop(hfdcan);
            Q_ASSERT(HAL_OK == ret);
        }
    }
}

void
CO_CANsetNormalMode(CO_CANmodule_t* CANmodule) {
    /* Put CAN module in normal mode */
    if((CANmodule == NULL) || (CANmodule->CANptr == NULL)) {
        return;
    }
    if(CANmodule->CANnormal == false) {
        FDCAN_HandleTypeDef * hfdcan = (FDCAN_HandleTypeDef *)CANmodule->CANptr;

        Q_ASSERT(HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, 32, 0) == HAL_OK);
        Q_ASSERT(HAL_FDCAN_EnableTxDelayCompensation(hfdcan) == HAL_OK);
        Q_ASSERT(HAL_FDCAN_Start(hfdcan) == HAL_OK);
        Q_ASSERT(HAL_FDCAN_ActivateNotification(hfdcan,
                (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY |
                FDCAN_IT_TX_ABORT_COMPLETE | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF),
                FDCAN_TX_BUFFER0) == HAL_OK);

        CANmodule->CANnormal = true;
    }
}

CO_ReturnError_t
CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
                  uint16_t txSize, uint16_t CANbitRate) {
    uint16_t i;

    /* verify arguments */
    Q_ASSERT((CANmodule != NULL) && (rxArray != NULL) &&
             (txArray != NULL) && (rxSize < 28U));

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = true;
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    pModule = CANmodule;

    for (i = 0U; i < rxSize; i++) {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (i = 0U; i < txSize; i++) {
        txArray[i].bufferFull = false;
    }

    /* Configure CAN module registers */
    FDCAN_HandleTypeDef * hfdcan = (FDCAN_HandleTypeDef *)CANptr;

    HAL_FDCAN_DeInit(hfdcan);
    hfdcan->Init.StdFiltersNbr = rxSize;  // New number of standard filters
    HAL_FDCAN_Init(hfdcan);
    for (i = 0U; i < rxSize; i++) {
        FDCAN_FilterTypeDef sFilterConfig;
        sFilterConfig.IdType = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex = i;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterID1 = 0;
        sFilterConfig.FilterID2 = 0;
        Q_ASSERT(HAL_OK == HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig));
    }
    /* Configure CAN timing */

    /* Configure CAN module hardware filters */
    Q_ASSERT(HAL_OK == HAL_FDCAN_ConfigGlobalFilter(
                                hfdcan,
                                FDCAN_REJECT,          /* unmatched standard IDs */
                                FDCAN_REJECT,          /* unmatched extended IDs  */
                                FDCAN_REJECT_REMOTE,   /* unmatched std RTR       */
                                FDCAN_REJECT_REMOTE));  /* unmatched ext RTR       */

    /* configure CAN interrupt registers */

    return CO_ERROR_NO;
}

void
CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
    if(CANmodule != NULL) {
        if(CANmodule->CANnormal) {
            CANmodule->CANnormal = false;
            if(CANmodule->CANptr != NULL) {
                /* turn off the module */
                FDCAN_HandleTypeDef * hfdcan = (FDCAN_HandleTypeDef *)CANmodule->CANptr;
                if(hfdcan->State == HAL_FDCAN_STATE_BUSY) {
                    Q_ASSERT(HAL_OK == HAL_FDCAN_Stop(hfdcan));
                }
            }
        }
    }
}

CO_ReturnError_t
CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object,
                   void (*CANrx_callback)(void* object, void* message)) {
    CO_ReturnError_t ret = CO_ERROR_NO;

    if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) && (index < CANmodule->rxSize)) {
        /* buffer, which will be configured */
        CO_CANrx_t* buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident & 0x07FFU;
        if (rtr) {
            // RTR not supported
        }
        buffer->mask = (mask & 0x07FFU);

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters) {
            FDCAN_FilterTypeDef sFilterConfig;
            sFilterConfig.IdType = FDCAN_STANDARD_ID;
            sFilterConfig.FilterIndex = index;
            sFilterConfig.FilterType = FDCAN_FILTER_MASK;
            sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
            sFilterConfig.FilterID1 = buffer->ident;
            sFilterConfig.FilterID2 = buffer->mask;

            FDCAN_HandleTypeDef * hfdcan = (FDCAN_HandleTypeDef *)CANmodule->CANptr;
            if(CANmodule->CANnormal) {
                Q_ASSERT(HAL_OK == HAL_FDCAN_Stop(hfdcan));
            }

            Q_ASSERT(HAL_OK == HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig));

            if(CANmodule->CANnormal) {
                Q_ASSERT(HAL_OK == HAL_FDCAN_Start(hfdcan));
            }
        }
    } else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

CO_CANtx_t*
CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                   bool_t syncFlag) {
    CO_CANtx_t* buffer = NULL;
    (void)rtr; // unused

    if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer, microcontroller specific. */
        buffer->ident = ((uint32_t)ident & 0x07FFU);
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

CO_ReturnError_t
CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    /* if CAN TX buffer is free, copy message to it */
    if (CANmodule->CANtxCount == 0) {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
        /* copy message and txRequest */
        FDCAN_TxHeaderTypeDef txHeader;
        txHeader.Identifier = buffer->ident;
        txHeader.IdType = FDCAN_STANDARD_ID;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0;
        txHeader.DataLength = buffer->DLC;

        FDCAN_HandleTypeDef * hfdcan = (FDCAN_HandleTypeDef *)CANmodule->CANptr;
        Q_ASSERT(HAL_OK == HAL_FDCAN_AddMessageToTxFifoQ(
                    hfdcan,
                    &txHeader,
                    buffer->data
                    ));
    }
    /* if no buffer is free, message will be sent by interrupt */
    else {
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}

void
CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/* messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag) {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount != 0U) {
        uint16_t i;
        CO_CANtx_t* buffer = &CANmodule->txArray[0];
        for (i = CANmodule->txSize; i > 0U; i--) {
            if (buffer->bufferFull) {
                if (buffer->syncFlag) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    if (tpdoDeleted != 0U) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

/* Get error counters from the module. If necessary, function may use different way to determine errors. */
static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void
CO_CANmodule_process(CO_CANmodule_t* CANmodule) {
    uint32_t err;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors >= 256U) {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        } else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF
                      ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
                         | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (txErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (overflow != 0) {
            /* CAN RX bus overflow */
            status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
    }
}

void
CO_CANinterrupt(CO_CANmodule_t* CANmodule) {

    /* receive interrupt */
    if (1) {

        /* Clear interrupt flag */
    }

    /* transmit interrupt */
    else if (0) {
        /* Clear interrupt flag */

    } else {
        /* some other interrupt reason */
    }
}


void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef * hfdcan,
    uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    CO_CANrxMsg_t rcvMsg;
    Q_ASSERT(pModule != NULL);

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rcvMsg.data) == HAL_OK) {
            rcvMsg.ident = rxHeader.Identifier;
            rcvMsg.DLC = rxHeader.DataLength;
            const uint16_t index = rxHeader.FilterIndex; /* index of received message */
            Q_ASSERT(index < pModule->rxSize);
            CO_CANrx_t* buffer = &pModule->rxArray[index]; /* receive message buffer from CO_CANmodule_t object. */
            Q_ASSERT(buffer != NULL);
            /* Call specific function, which will process the message */
            if(buffer->CANrx_callback != NULL) {
                buffer->CANrx_callback(buffer->object, (void*)&rcvMsg);
            }
        }
    }
}


void HAL_FDCAN_TxEventFifoCallback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t TxEventFifoITs)
{
    /* First CAN message (bootup) was sent successfully */
    pModule->firstCANtxMessage = false;
    /* clear flag from previous message */
    pModule->bufferInhibitFlag = false;
    /* Are there any new messages waiting to be send */
    if (pModule->CANtxCount > 0U) {
        uint16_t i; /* index of transmitting message */

        /* first buffer */
        CO_CANtx_t* buffer = &pModule->txArray[0];
        /* search through whole array of pointers to transmit message buffers. */
        for (i = pModule->txSize; i > 0U; i--) {
            /* if message buffer is full, send it. */
            if (buffer->bufferFull) {
                buffer->bufferFull = false;
                pModule->CANtxCount--;

                /* Copy message to CAN buffer */
                pModule->bufferInhibitFlag = buffer->syncFlag;

                FDCAN_TxHeaderTypeDef txHeader;
                txHeader.Identifier = buffer->ident;
                txHeader.IdType = FDCAN_STANDARD_ID;
                txHeader.TxFrameType = FDCAN_DATA_FRAME;
                txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                txHeader.BitRateSwitch = FDCAN_BRS_OFF;
                txHeader.FDFormat = FDCAN_CLASSIC_CAN;
                txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
                txHeader.MessageMarker = 0;
                txHeader.DataLength = buffer->DLC;

                FDCAN_HandleTypeDef * hfdcan = (FDCAN_HandleTypeDef *)pModule->CANptr;
                Q_ASSERT(HAL_OK == HAL_FDCAN_AddMessageToTxFifoQ(
                            hfdcan,
                            &txHeader,
                            buffer->data
                            ));
                break; /* exit for loop */
            }
            buffer++;
        } /* end of for loop */

        /* Clear counter if no more messages */
        if (i == 0U) {
            pModule->CANtxCount = 0U;
        }
    }
}
