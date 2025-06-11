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
uint8_t key_state;

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
                copyData(can2);
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
                copyData(can1);
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

void copyData(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2) {
    memcpy(TxData, RxData, 8);
    TxHeader.DLC = RxHeader.DLC;
    TxHeader.StdId = RxHeader.StdId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    filtercan(RxHeader.StdId, TxData, can2);
}

void filtercan(int airbid, uint8_t data[8], CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2) {
    if (airbid == 0x128) {
	//screen brightness
        uint8_t d0 = data[1];
	if (d0 == 0xD6 || d0 == 0x1A) {
    		lights = 1;
		data[1] = 0x08;
	} else {
    		lights = 0;
		data[1] = 0xff;
	}
    }
    if (airbid == 0x406) {
	//Ign Status
        uint8_t d0 = data[1];
	if (d0 == 0x01) {
    		key_state = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	} else if (d0 == 0x02) {
    		key_state = 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	} else if (d0 == 0x04) {
    		key_state = 2;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	} else if (d0 == 0x05) {
    		key_state = 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	} else {
		key_state = 0;
	}
    }
    if (airbid == 0x545) {
        // Placeholder for future processing
    }
}
