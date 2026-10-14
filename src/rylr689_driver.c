/*
 * rylr689.c
 *
 *  Created on: Sep 29, 2026
 *      Author: mvps1
 */


#include "rylr689_driver.h"

#define LORA_MODE__

/* Helper START. */
static void sendCommand(const rylr689_instance_t *dev, uint8_t opCode, uint8_t *args, uint32_t argCount) {
	uint8_t isBusy = dev->pinRead(dev->busyPin.port, dev->busyPin.pin);

	while(isBusy) {
		isBusy = dev->pinRead(dev->busyPin.port, dev->busyPin.pin);
	}

	/* Send OP code. */
	dev->spiTransmit(dev->spiHandle, &opCode, 1);

	/* Send command arguments. */
	if(args != NULL) {
		dev->spiTransmit(dev->spiHandle, args, argCount);
	}
} /* End of sendCommand() */

#ifdef LORA_MODE__
static void configCommonParams(const rylr689_instance_t *dev) {
	/* 1: Set to Stand-by. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	uint8_t cmd = LLCC68_CMD_SET_STANDBY;
	uint8_t args[] = {0x00};
	sendCommand(dev, cmd, args, sizeof(args));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 2: Set packet type. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PACKET_TYPE;
	uint8_t args2[] = {0x01}; /* LoRa Packet Type. */
	sendCommand(dev, cmd, args2, sizeof(args2));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 3: Set RF Frequency. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_RF_FREQUENCY;
	uint8_t args3[] = {0x39, 0x30, 0x00, 0x00}; /* 915000 MHz. */
	sendCommand(dev, cmd, args3, sizeof(args3));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 4: Set Buffer base address. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_BUFFER_BASE_ADDRESS;
	uint8_t args4[] = {0x00, 0x80};
	sendCommand(dev, cmd, args4, sizeof(args4));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 5: Set Modulation params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_MODULATION_PARAMS;
	/* SF9, 125 KHz BW, 4/5 CR, LDO Enabled.*/
	uint8_t args5[] = {0x09, 0x04, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args5, sizeof(args5));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */
} /* End of configureCommonParams() */
/* Helper END. */


void rylr689_radio_receive(const rylr689_instance_t *dev, uint32_t* packetSize) {
	uint8_t cmd;
	uint8_t receiveDone = 0;

	/* Set hardware switch to lock-in RX mode. */
	dev->pinWrite(dev->rfswV1Pin.port, dev->rfswV1Pin.pin, 1);
	dev->pinWrite(dev->rfswV2Pin.port, dev->rfswV2Pin.pin, 0);

	/* 6: Set Packet params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PACKET_PARAMS;
	/* Premable length: Byte #1 and #2, header type: variable, Payload length: 256 bytes, CRC enabled, Standard IQ, Byte 6 to 8: Not used. */
	uint8_t args6[] = {0x00, 0x0C, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args6, sizeof(args6));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 7: Set DioIrqParams. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_DIO_IRQ_PARAMS;
	uint8_t args7[] = {0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args7, sizeof(args7));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 9: Set chip to Rx mode. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_RX;
	uint8_t args9[] = {0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args9, sizeof(args9));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* Wait to receive packet. This is blocking version of RX. */
	while(!receiveDone) {
		receiveDone = dev->pinRead(dev->itPin.port, dev->itPin.pin);
	}

	/* Fetch buffer status. This includes size of packet received. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_GET_RX_BUFFER_STATUS;
	uint8_t status[3];
	sendCommand(dev, cmd, NULL,0);
	dev->spiReceive(dev->spiHandle, status, 3);
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* Protect from noise that triggers I/O interrupt in radio. */
	if(status[1] <= dev->rxBuffSize) {
		/* Fetch the buffer content. */
		dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
		cmd = LLCC68_CMD_READ_BUFFER;
		uint8_t args10[] = {status[2], 0x00};
		sendCommand(dev, cmd, args10, sizeof(args10));
		dev->spiReceive(dev->spiHandle,dev->rxBuff, status[1]);
		dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

		*packetSize = status[1];
	}

	/* 12: Clear IRQ RxDone flag. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_CLEAR_IRQ_STATUS;
	uint8_t args11[] = {0x00, 0x02};
	sendCommand(dev, cmd, args11, sizeof(args11));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */
}

void rylr689_radio_send(const rylr689_instance_t *dev, const uint32_t packetSize) {
	uint8_t cmd;
	uint8_t transmitDone = 0;

	/* Set hardware switch to lock-in TX mode. */
	dev->pinWrite(dev->rfswV1Pin.port, dev->rfswV1Pin.pin, 0);
	dev->pinWrite(dev->rfswV2Pin.port, dev->rfswV2Pin.pin, 1);

	/* 4: Set PA config. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PA_CONFIG;
	uint8_t args4[] = {0x04, 0x07, 0x00, 0x01};
	sendCommand(dev, cmd, args4, sizeof(args4));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 5: Set Tx params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_TX_PARAMS;
	uint8_t args5[] = {0x16, 0x02};
	sendCommand(dev, cmd, args5, sizeof(args5));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 7: Write to buffer. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_WRITE_BUFFER;
	uint8_t offset = 0x00U;
	sendCommand(dev, cmd, &offset, 1);
	dev->spiTransmit(dev->spiHandle, dev->txBuff, packetSize);
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 9: Set Packet params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PACKET_PARAMS;
	/* Premable length: Byte #1 and #2, header type: variable, Payload length: <packetSize> bytes, CRC enabled, Standard IQ, Byte 6 to 8: Not used. */
	uint8_t args9[] = {0x00, 0x0C, 0x00, packetSize, 0x01, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args9, sizeof(args9));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 10: Set DioIrqParams. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_DIO_IRQ_PARAMS;
	uint8_t args10[] = {0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args10, sizeof(args10));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 12: Set chip to Tx mode. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_TX;
	uint8_t args12[] = {0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args12, sizeof(args12));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 13: Wait for the transmission to finish. */
	while(!transmitDone) {
		transmitDone = dev->pinRead(dev->itPin.port, dev->itPin.pin);
	}

	/* 14: Clear IRQ TxDone flag. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_CLEAR_IRQ_STATUS;
	uint8_t args13[] = {0x00, 0x01};
	sendCommand(dev, cmd, args13, sizeof(args13));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */
}

void rylr689_radio_receive_it(const rylr689_instance_t *dev) {
	uint8_t cmd;
	uint8_t receiveDone = 0;

	/* Set hardware switch to lock-in RX mode. */
	dev->pinWrite(dev->rfswV1Pin.port, dev->rfswV1Pin.pin, 1);
	dev->pinWrite(dev->rfswV2Pin.port, dev->rfswV2Pin.pin, 0);

	/* 6: Set Packet params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PACKET_PARAMS;
	/* Premable length: Byte #1 and #2, header type: variable, Payload length: 256 bytes, CRC enabled, Standard IQ, Byte 6 to 8: Not used. */
	uint8_t args6[] = {0x00, 0x0C, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args6, sizeof(args6));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 7: Set DioIrqParams. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_DIO_IRQ_PARAMS;
	uint8_t args7[] = {0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args7, sizeof(args7));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 9: Set chip to Rx mode. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_RX;
	uint8_t args9[] = {0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args9, sizeof(args9));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */
}

void rylr689_radio_get_packet(const rylr689_instance_t *dev, uint32_t* packetSize) {
	uint8_t cmd;

	/* Fetch buffer status. This includes size of packet received. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_GET_RX_BUFFER_STATUS;
	uint8_t status[3];
	sendCommand(dev, cmd, NULL,0);
	dev->spiReceive(dev->spiHandle, status, 3);
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* Protect from noise that triggers I/O interrupt in radio. */
	if(status[1] <= dev->rxBuffSize) {
		/* Fetch the buffer content. */
		dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
		cmd = LLCC68_CMD_READ_BUFFER;
		uint8_t args10[] = {status[2], 0x00};
		sendCommand(dev, cmd, args10, sizeof(args10));
		dev->spiReceive(dev->spiHandle,dev->rxBuff, status[1]);
		dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

		*packetSize = status[1];
	}

	/* 12: Clear IRQ RxDone flag. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_CLEAR_IRQ_STATUS;
	uint8_t args11[] = {0x00, 0x02};
	sendCommand(dev, cmd, args11, sizeof(args11));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */
}

void rylr689_radio_send_it(const rylr689_instance_t *dev, const uint32_t packetSize) {
	uint8_t cmd;
	uint8_t transmitDone = 0;

	/* Set hardware switch to lock-in TX mode. */
	dev->pinWrite(dev->rfswV1Pin.port, dev->rfswV1Pin.pin, 0);
	dev->pinWrite(dev->rfswV2Pin.port, dev->rfswV2Pin.pin, 1);

	/* 4: Set PA config. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PA_CONFIG;
	uint8_t args4[] = {0x04, 0x07, 0x00, 0x01};
	sendCommand(dev, cmd, args4, sizeof(args4));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 5: Set Tx params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_TX_PARAMS;
	uint8_t args5[] = {0x16, 0x02};
	sendCommand(dev, cmd, args5, sizeof(args5));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 7: Write to buffer. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_WRITE_BUFFER;
	uint8_t offset = 0x00U;
	sendCommand(dev, cmd, &offset, 1);
	dev->spiTransmit(dev->spiHandle, dev->txBuff, packetSize);
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 9: Set Packet params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PACKET_PARAMS;
	/* Premable length: Byte #1 and #2, header type: variable, Payload length: <packetSize> bytes, CRC enabled, Standard IQ, Byte 6 to 8: Not used. */
	uint8_t args9[] = {0x00, 0x0C, 0x00, packetSize, 0x01, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args9, sizeof(args9));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 10: Set DioIrqParams. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_DIO_IRQ_PARAMS;
	uint8_t args10[] = {0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args10, sizeof(args10));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 12: Set chip to Tx mode. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_TX;
	uint8_t args12[] = {0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args12, sizeof(args12));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 13: Wait for the transmission to finish. */
	while(!transmitDone) {
		transmitDone = dev->pinRead(dev->itPin.port, dev->itPin.pin);
	}

	/* 14: Clear IRQ TxDone flag. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_CLEAR_IRQ_STATUS;
	uint8_t args13[] = {0x00, 0x01};
	sendCommand(dev, cmd, args13, sizeof(args13));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */
}



#else /* (G)FSK Mode. */
static void configCommonParams(const rylr689_instance_t *dev) {
	/* 1: Set to Stand-by. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	uint8_t cmd = LLCC68_CMD_SET_STANDBY;
	uint8_t args[] = {0x00};
	sendCommand(dev, cmd, args, sizeof(args));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 2: Set packet type. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PACKET_TYPE;
	uint8_t args2[] = {0x00};
	sendCommand(dev, cmd, args2, sizeof(args2));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 3: Set RF Frequency. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_RF_FREQUENCY;
	uint8_t args3[] = {0x39, 0x30, 0x00, 0x00};
	sendCommand(dev, cmd, args3, sizeof(args3));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 4: Set Buffer base address. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_BUFFER_BASE_ADDRESS;
	uint8_t args4[] = {0x00, 0x80};
	sendCommand(dev, cmd, args4, sizeof(args4));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 5: Set Modulation params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_MODULATION_PARAMS;
	uint8_t args5[] = {0x03, 0x41, 0x55, 0x09, 0x0C, 0x00, 0x14, 0x7B};
	sendCommand(dev, cmd, args5, sizeof(args5));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 6: Set Sync word value. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_WRITE_REGISTER;
	uint8_t args8[] = {0x06, 0xC0, 0x01, 0x02, 0x03};
	sendCommand(dev, cmd, args8, sizeof(args8));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */
} /* End of configureCommonParams() */
/* Helper END. */



void rylr689_radio_receive(const rylr689_instance_t *dev, uint32_t* packetSize) {
	uint8_t cmd;
	uint8_t receiveDone = 0;

	/* Set hardware switch to lock-in RX mode. */
	dev->pinWrite(dev->rfswV1Pin.port, dev->rfswV1Pin.pin, 1);
	dev->pinWrite(dev->rfswV2Pin.port, dev->rfswV2Pin.pin, 0);

	/* 6: Set Packet params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PACKET_PARAMS;
	uint8_t args6[] = {0x00, 0x20, 0x04, 0x18, 0x00, 0x01, 0xFF, 0x02, 0x01};
	sendCommand(dev, cmd, args6, sizeof(args6));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 7: Set DioIrqParams. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_DIO_IRQ_PARAMS;
	uint8_t args7[] = {0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args7, sizeof(args7));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 9: Set chip to Rx mode. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_RX;
	uint8_t args9[] = {0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args9, sizeof(args9));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* Wait to receive packet. This is blocking version of RX. */
	while(!receiveDone) {
		receiveDone = dev->pinRead(dev->itPin.port, dev->itPin.pin);
	}

	/* Fetch buffer status. This includes size of packet received. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_GET_RX_BUFFER_STATUS;
	uint8_t status[3];
	sendCommand(dev, cmd, NULL,0);
	dev->spiReceive(dev->spiHandle, status, 3);
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* Protect from noise that triggers I/O interrupt in radio. */
	if(status[1] <= dev->rxBuffSize) {
		/* Fetch the buffer content. */
		dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
		cmd = LLCC68_CMD_READ_BUFFER;
		uint8_t args10[] = {status[2], 0x00};
		sendCommand(dev, cmd, args10, sizeof(args10));
		dev->spiReceive(dev->spiHandle,dev->rxBuff, status[1]);
		dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

		*packetSize = status[1];
	}

	/* 12: Clear IRQ RxDone flag. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_CLEAR_IRQ_STATUS;
	uint8_t args11[] = {0x00, 0x02};
	sendCommand(dev, cmd, args11, sizeof(args11));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */
}

void rylr689_radio_send(const rylr689_instance_t *dev, const uint32_t packetSize) {
	uint8_t cmd;
	uint8_t transmitDone = 0;

	/* Set hardware switch to lock-in TX mode. */
	dev->pinWrite(dev->rfswV1Pin.port, dev->rfswV1Pin.pin, 0);
	dev->pinWrite(dev->rfswV2Pin.port, dev->rfswV2Pin.pin, 1);

	/* 4: Set PA config. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PA_CONFIG;
	uint8_t args4[] = {0x04, 0x07, 0x00, 0x01};
	sendCommand(dev, cmd, args4, sizeof(args4));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 5: Set Tx params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_TX_PARAMS;
	uint8_t args5[] = {0x16, 0x02};
	sendCommand(dev, cmd, args5, sizeof(args5));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 7: Write to buffer. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_WRITE_BUFFER;
	uint8_t offset = 0x00U;

	sendCommand(dev, cmd, &offset, 1);
	dev->spiTransmit(dev->spiHandle, dev->txBuff, packetSize);
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 9: Set Packet params. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_PACKET_PARAMS;
	uint8_t args9[] = {0x00, 0x20, 0x04, 0x18, 0x00, 0x01, packetSize, 0x02, 0x01};
	sendCommand(dev, cmd, args9, sizeof(args9));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 10: Set DioIrqParams. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_DIO_IRQ_PARAMS;
	uint8_t args10[] = {0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args10, sizeof(args10));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 12: Set chip to Tx mode. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_SET_TX;
	uint8_t args12[] = {0x00, 0x00, 0x00};
	sendCommand(dev, cmd, args12, sizeof(args12));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */

	/* 13: Wait for the transmission to finish. */
	while(!transmitDone) {
		transmitDone = dev->pinRead(dev->itPin.port, dev->itPin.pin);
	}

	/* 14: Clear IRQ TxDone flag. */
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 0); /* Chip select enable. */
	cmd = LLCC68_CMD_CLEAR_IRQ_STATUS;
	uint8_t args13[] = {0x00, 0x01};
	sendCommand(dev, cmd, args13, sizeof(args13));
	dev->pinWrite(dev->csPin.port, dev->csPin.pin, 1); /* Chip select disable. */
}
#endif







void rylr689_radio_init(rylr689_instance_t *dev) {

	configCommonParams(dev);

	dev->initialized = 1;
}
