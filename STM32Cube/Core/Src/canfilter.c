//CAN1 to CAR
//CAN2 to Sync
/*
 * canfilter.c
 *
 *  Created on: Jan 15, 2020
 *      Author: eko
 */
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint8_t gear;
uint8_t msg023b4;
uint8_t msg023b7;

// Block list for CAN1 -> CAN2 (do not include ids that need to be modified) From CAR TO SYNC
const uint16_t blocklist_can1_to_can2[] = {0x998, 0x999}; // Example IDs
const size_t blocklist_can1_to_can2_count = sizeof(blocklist_can1_to_can2)/sizeof(blocklist_can1_to_can2[0]);

int is_blocked_can1_to_can2(uint16_t id) {
    for (size_t i = 0; i < blocklist_can1_to_can2_count; ++i) {
        if (blocklist_can1_to_can2[i] == id) {
            return 1;
        }
    }
    return 0;
}

// Block list for CAN2 -> CAN1 (do not include ids that need to be modified) From SYNC TO CAR
const uint16_t blocklist_can2_to_can1[] = {0x998, 0x999}; // Example IDs
const size_t blocklist_can2_to_can1_count = sizeof(blocklist_can2_to_can1)/sizeof(blocklist_can2_to_can1[0]);

int is_blocked_can2_to_can1(uint16_t id) {
    for (size_t i = 0; i < blocklist_can2_to_can1_count; ++i) {
        if (blocklist_can2_to_can1[i] == id) {
            return 1;
        }
    }
    return 0;
}


/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void copyData(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2);
void filtercan(int airbid, uint8_t data[8], CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2);
void sendACCstate(CAN_HandleTypeDef *can2);
void sendIGNstate(CAN_HandleTypeDef *can2);
void sendGear(CAN_HandleTypeDef *can2);
/* USER CODE END PFP */

void canloop(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2) {
    while (1) {
        // Receive Message from Can1 & send to CAN2:
        if (HAL_CAN_GetRxFifoFillLevel(can1, CAN_RX_FIFO0) != 0) {
            if (HAL_CAN_GetRxMessage(can1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
                /* Reception Error */
                Error_Handler();
            }
	    // ID check goes here:
            if (!is_blocked_can1_to_can2(RxHeader.StdId)) {
            	copyData(can1, can2); // From CAN1 to CAN2
                if (HAL_CAN_GetTxMailboxesFreeLevel(can2) != 0) {
                    if (HAL_CAN_AddTxMessage(can2, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
                        // Transmission Error
                        HAL_CAN_ResetError(can2);
                        Error_Handler();
                    }
            }
        }
        // Do same on Can2:
        if (HAL_CAN_GetRxFifoFillLevel(can2, CAN_RX_FIFO1) != 0) {
            if (HAL_CAN_GetRxMessage(can2, CAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK) {
                /* Reception Error */
                Error_Handler();
            }
            // ID check goes here:
            if (!is_blocked_can2_to_can1(RxHeader.StdId)) {
            	copyData(can2, can1); // From CAN2 to CAN1
                if (HAL_CAN_GetTxMailboxesFreeLevel(can1) != 0) {
                    if (HAL_CAN_AddTxMessage(can1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
                        // Transmission Error
                        HAL_CAN_ResetError(can1);
                        Error_Handler();
                    }
            }
        }
        HAL_Delay(1); // Prevent 100% CPU usage
        }
     }
   }
}

void copyData(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2) {
    memcpy(TxData, RxData, 8);
    TxHeader.DLC = RxHeader.DLC;
    TxHeader.StdId = RxHeader.StdId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    filtercan(RxHeader.StdId, TxData, can1, can2);
}

void filtercan(int airbid, uint8_t data[8], CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2) {
    if (airbid == 0x3E9) {
	//Gear Position
        uint8_t d0 = data[0];
        gear = (d0 >> 4) & 0x0F; // Upper nibble
	if(gear > 3) gear = 0;
        switch (gear) {
            case 0:
                msg023b4 = 0x61;
                msg023b7 = 0x08;
                break;
            case 1:
                msg023b4 = 0x63;
                msg023b7 = 0x09;
                break;
            case 2:
                msg023b4 = 0x65;
                msg023b7 = 0x0a;
                break;
            case 3:
                msg023b4 = 0x67;
                msg023b7 = 0x0b;
                break;
            default:
                msg023b4 = 0x61;
                msg023b7 = 0x08;
                break;
        }
        sendGear(can2);
    }
    if (airbid == 0x353) {
	//Temperature Corection Code
        // Add 30 to data[4], handle overflow
        uint16_t val = (uint16_t)data[4] + 30;
        if (val > 0xFF) val = 0xFF; // Clamp to 255 if overflow
        data[4] = (uint8_t)val;
    }
    if (airbid == 0x545) {
        // Placeholder for future processing
    }
}

void sendACCstate(CAN_HandleTypeDef *can2) {
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0xa4, 0xc3, 0x83, 0x3a, 0x04, 0x81, 0x00, 0x00};
    uint32_t txMailbox;

    txHeader.StdId = 0x310;           // Set CAN ID to 0x310
    txHeader.ExtId = 0x00;            // Not used for standard ID
    txHeader.IDE = CAN_ID_STD;        // Use standard identifier
    txHeader.RTR = CAN_RTR_DATA;      // Data frame
    txHeader.DLC = 8;                 // 8 data bytes
    txHeader.TransmitGlobalTime = DISABLE; // Optional, depending on HAL version

    if (HAL_CAN_GetTxMailboxesFreeLevel(can2) != 0) {
        if (HAL_CAN_AddTxMessage(can2, &txHeader, txData, &txMailbox) != HAL_OK) {
            // Handle transmission error
            HAL_CAN_ResetError(can2);
            // Optionally call Error_Handler();
        }
    }
}

void sendIGNstate(CAN_HandleTypeDef *can2) {
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0xa6, 0xc3, 0x13, 0x40, 0x06, 0xc1, 0x00, 0x00};
    uint32_t txMailbox;

    txHeader.StdId = 0x310;           // Set CAN ID to 0x310
    txHeader.ExtId = 0x00;            // Not used for standard ID
    txHeader.IDE = CAN_ID_STD;        // Use standard identifier
    txHeader.RTR = CAN_RTR_DATA;      // Data frame
    txHeader.DLC = 8;                 // 8 data bytes
    txHeader.TransmitGlobalTime = DISABLE; // Optional, depending on HAL version

    if (HAL_CAN_GetTxMailboxesFreeLevel(can2) != 0) {
        if (HAL_CAN_AddTxMessage(can2, &txHeader, txData, &txMailbox) != HAL_OK) {
            // Handle transmission error
            HAL_CAN_ResetError(can2);
            // Optionally call Error_Handler();
        }
    }
}

void sendGear(CAN_HandleTypeDef *can2) {
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0x01, 0x00, 0x03, 0x30, msg023b4, 0x10, 0x4A, msg023b7};
    uint32_t txMailbox;

    txHeader.StdId = 0x023;            // Set CAN ID to 0x023
    txHeader.ExtId = 0x00;            // Not used for standard ID
    txHeader.IDE = CAN_ID_STD;        // Use standard identifier
    txHeader.RTR = CAN_RTR_DATA;      // Data frame
    txHeader.DLC = 8;                 // 8 data bytes
    txHeader.TransmitGlobalTime = DISABLE; // Optional, depending on HAL version

    if (HAL_CAN_GetTxMailboxesFreeLevel(can2) != 0) {
        if (HAL_CAN_AddTxMessage(can2, &txHeader, txData, &txMailbox) != HAL_OK) {
            // Handle transmission error
            HAL_CAN_ResetError(can2);
            // Optionally call Error_Handler();
        }
    }
}
