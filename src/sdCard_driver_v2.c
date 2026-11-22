/*
 * sdCard_driver_v2.c
 *
 *  Created on: Nov 17, 2026
 *      Author: Marcellus Von Sacramento
 */

#include "sdCard_driver_v2.h"
#include <string.h>

#define SD_DUMMY_BYTE	(0xFFU)
#define CMD_TEMPLATE		(0x40U) /* Command template: 0 1 ... 1 */
#define R1_RETRY_COUNT 	(1000U)

/* The following are SD Card SPI command indices derived from SD Specification Part 1. */
#define CMD0 	(0x00U) /* Resets the SD Memory Card. */
#define CMD8 	(0x08U) /* See SD-card specification document. */
#define CMD9 	(0x09U) /* Asks selected card to send its card-specific data. */
#define CMD12	(0x0CU) /* Command to force card to stop transmission in multiple block operation. */
#define CMD13 	(0x0DU) /* Asks the selected card to send its status register. */
#define CMD17 	(0x11U) /* Reads a single block of the size selected by SET_BLOCKLEN command. */
#define CMD18	(0x12U) /* Continuously transfer data blocks from card to host until interrupted by CMD12. */
#define CMD24 	(0x18U) /* Writes a block of the size selected by the SET_BLOCKLEN command. */
#define CMD25	(0x19U) /* Continuously writes a block data until 'Stop Tran' token is sent (instead of Start Block). */
#define CMD55 	(0x37U) /* Defines to the card that the next command is an application specific command rather than a standard command. */
#define CMD58 	(0x3AU) /* Reads the OCR register of a card. CCS bit is assigned to OCR[30]. */
#define ACMD41 	(0x29U) /* Sends host capacity support information and activates card's initialization process. */

/* Start Block Tokens and 'Stop Tran' Token.*/
#define SINGLE_BLOCK_WRITE_START_TOKEN  	(0xF7U)  /* Send to card for single block write. */
#define SINGLE_BLOCK_READ_START_TOKEN   	(0xF7U)  /* Send to card for single block read. */
#define MULTIPLE_BLOCK_READ_START_TOKEN 	(0xF7U)  /* Send to card for multiple block read. CMD12 is used to stop multiple block read.*/
#define MULTIPLE_BLOCK_WRITE_START_TOKEN (0xFCU) /* Send to card for each block in multiple block write. */
#define MULTIPLE_BLOCK_WRITE_STOP_TOKEN  (0xFDU) /* Send to card to stop writing in multiple block write. */



/* Arbitrary loop timeout. */
#define RESPONSE_TIMEOUT (1000U)
#define DATA_TIMEOUT (50000U)

/* Command and Data width in Bytes.*/
#define CMD_TOKEN_BYTE_LEN 	(6U)
#define CMD_ARG_BYTE_LEN		(4U) /* 4-byte command argument. */
#define DATA_TOKEN_WIDTH 	(512U) /* For SDHC and SDXC only. For SDSC, block length is set by CMD16. */



/* Response width in Bytes. */
#define R1_RESPONSE_WIDTH (1U)
#define R2_RESPONSE_WIDTH (2U)
#define R3_RESPONSE_WIDTH (5U)
#define R7_RESPONSE_WIDTH (5U)

/* The following are generator polynomials for generating CRCs.*/
#define CRC7_POLYNOMIAL (0x09U)
#define CRC16_POLYNOMIAL (0x1021U)

/* 48-bit command format. */
typedef struct {
	uint8_t msb;			/* Most Significant Byte = 8-bit: 1-bit Start + 1-bit Direction + 4-bit Command Index. */
	uint8_t arg[4];		/* 32-bit: Argument. */
	uint8_t lsb;			/* Least Significant Byte = 8-bit: 7-bit CRC7 + 1-bit Stop. */
} command_token_t;

typedef enum {
	R1_RESPONSE = 0,
	R2_RESPONSE,
	R3_RESPONSE,
	R7_RESPONSE
} response_type_t;


//typedef struct {
//	uint8_t startToken;     /* Start of transmission from host to SD, or vice-versa. */
//	uint8_t dataBlock[512]; /* The actual data from host to SD, or vice-versa. */
//	uint8_t crc16[2];       /* 16-bit CRC. */
//} data_token_t;

/* Card response structures. */
typedef uint8_t r1_response_t;

typedef struct {
	r1_response_t r1;
	uint8_t r2;
} r2_response_t;

typedef struct {
	r1_response_t r1;
	uint8_t ocr[4]; /* Operating condition register of the card. */
} r3_response_t;

typedef struct {
	r1_response_t r1;
	uint8_t cic[4]; /* Card interface condition. */
} r7_response_t;

/* CMD12 must be sent in case of the following data errors during multiple block write.
 * CMD13 can be used to retrieve the cause of write problem.
 * CMD22 can be used to find the number of well-written blocks.
 * */
typedef enum {
	DATA_ACCEPTED = 0X05,
	DATA_REJECTED_CRC_ERROR = 0X0B, /* Data rejected due to a CRC error. */
	DATA_REJECTED_WRITE_ERROR = 0X0D /* Data rejected due to a Write error. */
} data_response_t;

/* Bit label for responses. */
/* R1 response. */
typedef enum {
	R1_CLEAR = 0x00,
	R1_IN_IDLE = 0x01,
	R1_ERASE_RESET = 0x02,
	R1_ILLEGAL_COMMAND = 0x04,
	R1_COM_CRC_ERROR = 0x08,
	R1_ERASE_SEQUENCE_ERROR = 0x10,
	R1_ADDRESS_ERROR = 0x20,
	R1_PARAMETER_ERROR = 0x30,
	R1_FATAL_ERROR = 0x7F
} r1_bit_label_t;

typedef enum {
	R2_CLEAR = 0X00,
	R2_CARD_LOCKED = 0X01, /* Set when the card is locked by the user. Reset when it is unlocked. */
	R2_WP_ERASE_SKIP_OR_LOCK_UNLOCK_CMD_FAILED = 0X02, /* It is set when the host attempts to erase a write-protected
										                 sector or makes a sequence or password errors during card lock/unlock operation.*/
	R2_UNKNOWN_ERROR = 0X04, /* A general or an unknown error occurred during the operation. */
	R2_CC_ERROR = 0X08, /* Internal card controller error. */
	R2_CARD_ECC_FAILED = 0X10, /* Card internal ECC was applied but failed to correct the data.*/
	R2_WP_VIOLATION = 0X20, /*The command tried to write a write-protected block. */
	R2_ERASE_PARAM = 0X40, /* An invalid selection for erase, sectors or groups. */
	R2_CSD_OVERWRITE = 0X80,
	R2_FATAL_ERROR = 0X7F
} r2_bit_label_t;

typedef enum {
	/* General command result. */
	CMD_ACCEPTED = 0,
	ILLEGAL_CMD,
	CRC_ERROR,
	ADDRESS_ERROR,
	PARAMETER_ERROR,
	FATAL_ERROR
} CMDRESULT;

/* Globals START. */

/* Globals END. */

/***** Helper method declaration START. *****/
static void selectDev(const sdCard_instance_t *dev);
static void deselectDev(const sdCard_instance_t *dev);
static uint8_t generateCRC7(const uint8_t* input, uint8_t size);
static uint16_t generateCRC16(const uint8_t* input, uint32_t size);
//static void print_cmdtoken(uint8_t* cmd_token);
//static void print_response(uint8_t* response, uint8_t size);
static void buildCommand(command_token_t *cmdToken, uint8_t cmd, uint8_t *arg, uint8_t needCRC);
static CMDRESULT sendCmd(const sdCard_instance_t *dev, command_token_t *cmdTok, uint8_t *r1, uint8_t *rem, uint8_t resLen);
//static void parseAddress(uint8_t* dest, uint32_t address);

/***** Helper method declaration END. *****/

/***** Helper functions definition START. *****/
static void selectDev(const sdCard_instance_t *dev) {
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0);
}

static void deselectDev(const sdCard_instance_t *dev) {
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1);
}

static uint8_t generateCRC7(const uint8_t* input, uint8_t size) {
	uint8_t crc = 0;

	for(int i = 0; i < size; ++i) {
		uint8_t currentByte = input[i];

		for(int j = 0; j < 8; ++j) {
			uint8_t crc_msb = (crc & 0x40) ? 1 : 0;
			uint8_t bit = (currentByte & 0x80) ? 1 : 0;

			crc <<= 1;

			if(crc_msb ^ bit) {
				crc ^= CRC7_POLYNOMIAL;
			}
			currentByte <<= 1;   // <-- move this to the end
		}
	}

	/* Only need 7-bits. */
	return (crc & 0x7F);
} /* End of generateCRC7(). */

static uint16_t generateCRC16(const uint8_t* input, uint32_t size) {
	uint16_t crc = 0;
	uint32_t crc_msb_pos = 15;

	for(int i = 0; i < size; ++i) {
		uint8_t currentByte = input[i];

		for(int j = 7; j >= 0; --j) {
			/* Extract one data bit. */
			uint8_t crc_msb = (crc >> crc_msb_pos) & 1;
			uint8_t bit = (currentByte >> j) & 1;

			/* Make space for new bit. */
			crc <<= 1;

			/* Check if CRC MSB is 1. */
			if(crc_msb ^ bit) {
				// XOR.
				crc ^= CRC16_POLYNOMIAL;
			}
		}
	}

	return crc;
} /* End of generateCRC16(). */

static void buildCommand(command_token_t *cmdToken, uint8_t cmd, uint8_t *arg, uint8_t needCRC) {
	cmdToken->msb = CMD_TEMPLATE | cmd;

	if(arg != NULL) {
		memcpy(cmdToken->arg, arg, CMD_ARG_BYTE_LEN);
	}
	else {
		memset(cmdToken->arg, 0, CMD_ARG_BYTE_LEN); /* Stuff or reserved bytes set to 0. */
	}

	/* Some command do not need CRC and stop bit such as ACMD41 in SPI mode. */
	if(needCRC) {
		cmdToken->lsb = ((generateCRC7((uint8_t*)cmdToken, CMD_TOKEN_BYTE_LEN - 1) << 1) | 0x01);
	}
	else {
		cmdToken->lsb = 0x01;
	}
}

/* Sends command from host to card. */
static CMDRESULT sendCmd(const sdCard_instance_t *dev, command_token_t *cmdTok, uint8_t *r1, uint8_t *rem, uint8_t resLen) {
	if(dev == NULL || r1 == NULL || resLen < 1 || (rem == NULL && resLen > 1)) {
		return PARAMETER_ERROR;
	}

	uint8_t dummyByte[resLen];

	if(resLen > 1) {
		memset(dummyByte, SD_DUMMY_BYTE, resLen);
	}
	else {
		dummyByte[0] = SD_DUMMY_BYTE;
	}

	/* Send command. */
	dev->spiTransmit(dev->spiHandle, (uint8_t*)cmdTok, CMD_TOKEN_BYTE_LEN);

	/* Attempt to get response from card. */
	for(int i = 0; i < R1_RETRY_COUNT; ++i) {
		/* Get R1 response. */
		dev->spiTransmitReceive(dev->spiHandle, dummyByte, r1, R1_RESPONSE_WIDTH);

		/* Wait until CRC flag bit clears. */
		if((*r1 & R1_COM_CRC_ERROR) == 0) {
			/* Then, check if command is valid. */
			if(*r1 & R1_ILLEGAL_COMMAND) { /* Invalid command. */
				return ILLEGAL_CMD;
			}

			/* If valid, check if more response is expected from card. */
			if(resLen > 1) {
				/* Get rest of response. */
				dev->spiTransmitReceive(dev->spiHandle, dummyByte, rem, resLen - 1); /* -1 for R1. */
			}

			return CMD_ACCEPTED;	 /* Depending on the command this might a be sufficient return value.
								  * Otherwise, validation of other R1 bits, aside from CRC and illegal
								  *	command flag bits which are checked here already, must be performed.
								  * */
		}
	}

	return CRC_ERROR; /* CRC flag never cleared even after multiple retries. */
}

/***** Helper functions definition END. *****/


uint8_t xSd_Init_v2(sdCard_instance_t *dev) {
	if(dev->spiHandle == NULL) {
			return 0;
	}
	/* Check SPI communication state. */
//	if(!(dev->getState(dev->spiHandle))) {
//		return 0; /* Failure. SPI not initialized or ready for use. */
//	}

	/* Baud rate for initialization: <= 400KHz. */
	dev->setBaudrate(dev->spiHandle, 1);

	/* SD Power Up Sequence START. */
	/* Per datasheet: 0.1ms to 35ms max ramp up time (time it takes for VDD to be stable)
	 * plus the the 1ms required VDD stable time.*/
	/* 40ms just to be conservative and safe. */
	dev->delay(40);

	deselectDev(dev);
	/* Send at least 74-clock (>=74 bits) cycle worth of dummy bytes to ensure power is stable before sending first command. */
	uint8_t dum[20];
	memset(dum, SD_DUMMY_BYTE, 20);
	dev->spiTransmit(dev->spiHandle, dum, sizeof(dum));
	/* SD Power Up Sequence END. */

	/* Start initialization. */
	command_token_t cmdTok;
	CMDRESULT res;
	r1_response_t r1;
	r3_response_t r3;
	r7_response_t r7;

	/* For CMD8. */
	uint8_t checkPattern = 0xAAU;
	uint8_t vhs = 0x01U; /* Voltage supply range. ~2.7v - 3.6v */

	/* For ACMD41. */
	uint8_t initSuccess = 0;
	uint32_t maxRetry = 10000;

	/* Prepare CMD0. */
	buildCommand(&cmdTok, CMD0, NULL, 1);
	/* Send CMD0. */
	selectDev(dev);
	res = sendCmd(dev, &cmdTok, &r1, NULL, R1_RESPONSE_WIDTH);
	deselectDev(dev);

	if((res != CMD_ACCEPTED) || ((r1 & R1_IN_IDLE) == 0)) {
		return 0; /* Error: CRC error or illegal command or card not in idle. */
	}

	/* Prepare CMD8. */
	buildCommand(&cmdTok, CMD8, (uint8_t[]) {0x00U, 0x00U, vhs, checkPattern}, 1);
	/* Send CMD8. */
	selectDev(dev);
	res = sendCmd(dev, &cmdTok, &r7.r1, r7.cic, R7_RESPONSE_WIDTH);
	deselectDev(dev);

	if((res == CMD_ACCEPTED) && (r7.r1 & R1_IN_IDLE)) { /* Accepted means SD card is ver2.00 or later. */

		/* Error: Idle bit not set or check pattern incorrect or host voltage supply not supported. */
		if((r7.cic[3] != checkPattern) || (r7.cic[2] != vhs)) {
			return 0;
		}

		/* Prepare initial CMD58. Initial cdm58 for VHS sanity check. */
		buildCommand(&cmdTok, CMD58, NULL, 1);
		/* Send CMD58. */
		selectDev(dev);
		res = sendCmd(dev, &cmdTok, &r3.r1, r3.ocr, R3_RESPONSE_WIDTH);
		deselectDev(dev);

		if(res != CMD_ACCEPTED || ((r1 & R1_IN_IDLE) != 1)) {
			return 0;
		}

		/* For VHS sanity check. */
		if((r3.ocr[1] | r3.ocr[2]) == 0) {
			return 0;
		}

		/* Wait until ACMD41 finishes initialization. */
		for(int i = 0; i < maxRetry; ++i) {
			/* Prepare CMD55. */
			buildCommand(&cmdTok, CMD55, NULL, 1);
			/* Send CMD55. */
			selectDev(dev);
			res = sendCmd(dev, &cmdTok, &r1, NULL, R1_RESPONSE_WIDTH);
			deselectDev(dev);

			if((res != CMD_ACCEPTED) || (r1 & R1_IN_IDLE) != 1) {
				return 0;
			}

			/* Prepare ACMD41. */
			buildCommand(&cmdTok, ACMD41, (uint8_t[]){0x40U, 0x00U, 0x00U, 0x00U}, 1);
			/* Send ACMD41. */
			selectDev(dev);
			res = sendCmd(dev, &cmdTok, &r1, NULL, R1_RESPONSE_WIDTH);
			deselectDev(dev);

			if(res != CMD_ACCEPTED) {
				return 0;
			}

			if((r1 & R1_IN_IDLE) == 0) {
				initSuccess = 1;
				break; /* Card finished initializing. */
			}
		}

		/* ACMD41 timeout. */
		if(!initSuccess) {
			return 0;
		}

		/* Prepare final CMD58. This is for identifying card capacity. */
		buildCommand(&cmdTok, CMD58, NULL, 1);

		/* Send CMD58. */
		selectDev(dev);
		res = sendCmd(dev, &cmdTok, &r3.r1, r3.ocr, R3_RESPONSE_WIDTH);
		deselectDev(dev);

		if((res != CMD_ACCEPTED) || (r1 & R1_IN_IDLE)) { /* Should not be in idle at this point. */
			return 0;
		}

		if((r3.ocr[0] & 0x80U) == 0) { /* Power up bit should be set at this point. */
			return 0;
		}

		/* Power up done and CCS is valid. */
		dev->ccs = (r3.ocr[0] & 0x40U) ? SDHC_SDXC : SDSC; /* 1 = High capacity card. 0 = Standard capacity card.*/
	}
	else if((res == ILLEGAL_CMD) && (r7.r1 & R1_IN_IDLE)) { /* Card is ver1.00 and does not support CMD8. */
		dev->ccs = SDSC; /* 0 = Standard capacity card. */

		/* Prepare initial CMD58. */
		buildCommand(&cmdTok, CMD58, NULL, 1);
		/* Send CMD58. */
		selectDev(dev);
		res = sendCmd(dev, &cmdTok, &r3.r1, r3.ocr, R3_RESPONSE_WIDTH);
		deselectDev(dev);

		if(res != CMD_ACCEPTED || ((r1 & R1_IN_IDLE) != 1)) {
			return 0;
		}

		/* Check if any voltage range bit is set. */
		if((r3.ocr[1] | r3.ocr[2]) == 0) {
			return 0;
		}

		/* Wait until ACMD41 finishes initialization. */
		for(int i = 0; i < maxRetry; ++i) {
			/* Prepare CMD55. */
			buildCommand(&cmdTok, CMD55, NULL, 1);
			/* Send CMD55. */
			selectDev(dev);
			res = sendCmd(dev, &cmdTok, &r1, NULL, R1_RESPONSE_WIDTH);
			deselectDev(dev);

			if((res != CMD_ACCEPTED) || (r1 & R1_IN_IDLE) != 1) {
				return 0;
			}

			/* Prepare ACMD41. */
			buildCommand(&cmdTok, ACMD41, NULL, 1); /* Argument can be 0 since HCS is ignored by SDSC. */
			/* Send ACMD41. */
			selectDev(dev);
			res = sendCmd(dev, &cmdTok, &r1, NULL, R1_RESPONSE_WIDTH);
			deselectDev(dev);

			if(res != CMD_ACCEPTED) {
				return 0;
			}

			if((r1 & R1_IN_IDLE) == 0) {
				initSuccess = 1;
				break; /* Card finished initializing. */
			}
		}
	}
	else {
		return 0; /* Failure due to CRC error or that card will is not in idle, or both. After CMD8. */
	}

	/* Add later: Checks for card lock and WP status. */


	/* Baud rate for other operation. fpclk = 16MHz/2 = 8 MHz. */
	dev->setBaudrate(dev->spiHandle, 0);


	dev->initialized = initSuccess;
	return initSuccess;
}


//uint8_t xSd_Init_v2(sdCard_instance_t *dev) {
//	if(dev->spiHandle == NULL) {
//			return 0;
//	}
//	/* Check SPI communication state. */
////	if(!(dev->getState(dev->spiHandle))) {
////		return 0; /* Failure. SPI not initialized or ready for use. */
////	}
//
//	/* Set byte-ordering mode. */
//	uint16_t temp = 0x4321U; /* AA is MSB and BB is LSB. */
//	uint8_t lowerAddressValue = *(uint8_t*)&temp; /* Store the first byte of the 2-byte check value. */
//	isLittleEndian = (lowerAddressValue == 0x21U);
//
//	/* Baud rate for initialization: <= 400KHz. */
//	dev->setBaudrate(dev->spiHandle, 1);
//
//	/* Send 74-clock cycle dummy to ensure power is stable before sending first command. */
//
//}
uint8_t xSd_Read_v2(const sdCard_instance_t *dev, uint32_t sector, uint8_t* buffer) {
	return 0;
}
uint8_t xSd_Write_v2(const sdCard_instance_t *dev, uint32_t sector, const uint8_t* buffer) {

	return 0;
}
uint8_t xSd_Busy_v2(const sdCard_instance_t *dev) {

	return 0;
}


