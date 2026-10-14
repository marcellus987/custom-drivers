/*
 * oled_SSH1106_driver.h
 *
 *  Created on: Sep 28, 2026
 *      Author: Marcellus Von Sacramento
 */

#include <oled_sh1106_driver.h>
#include "sh1106_charset.h"


#define OLED_START_COLUMN 3 /* Column start at 3rd pixel so that 5x7 font character will have even padding of 1px each side. */
#define OLED_PAGE_COUNT   8   /* 64 rows is divided into 8 pages. */
#define OLED_COLUMN_COUNT 128
#define CHARACTERS_PER_LINE 21 /* 21 characters per line for 5x7 font. In reality, this is 6x7 because of the 1px spacing for each characters. */

/* To be used for clearing screen or a portion of the screen. */
static const uint8_t clearPixels[OLED_COLUMN_COUNT] = {0};


/* Enumerated IDs to easily locate and reference registers in the map/ */
typedef enum {
    CMD_DISPLAY_OFF,
    CMD_COLUMN_LOWER,
    CMD_COLUMN_UPPER,
    CMD_START_LINE,
    CMD_PAGE_ADDRESS,
    CMD_CONTRAST_REG,
    CMD_CONTRAST_VAL,
    CMD_SEG_REMAP,
    CMD_SCAN_DIR,
    CMD_NORMAL_MODE,
    CMD_MULTIPLEX_REG,
    CMD_MULTIPLEX_VAL,
    CMD_OFFSET_REG,
    CMD_OFFSET_VAL,
    CMD_CLK_DIV_REG,
    CMD_CLK_DIV_VAL,
    CMD_PRECHARGE_REG,
    CMD_PRECHARGE_VAL,
    CMD_COMPINS_REG,
    CMD_COMPINS_VAL,
    CMD_VCOMH_REG,
    CMD_VCOMH_VAL,
    CMD_DISPLAY_ON,
    CMD_TOTAL_COUNT /* Automatically keeps track of the map size. */
} OLED_Command_ID;


/* Structure defining a single entry in our master database. */
typedef struct {
    OLED_Command_ID id;
    uint8_t value;
    const char* description;
} OLED_CommandMap_t;



/* --- THE MASTER DISPLAY INITIALIZATION MAP FROM DATASHEET. ---
 *  Hard-coded since reset sequence would not change. */
static OLED_CommandMap_t SH1106_InitMap[] = {
    { CMD_DISPLAY_OFF,    0xAE, "Display Sleep Mode (OFF)" },
    { CMD_COLUMN_LOWER,   0x02, "Set Lower Column Address Offset (SH1106 2-pixel offset fix)" },
    { CMD_COLUMN_UPPER,   0x10, "Set Higher Column Address Offset" },
    { CMD_START_LINE,     0x40, "Set Display Start Line to 0" },
    { CMD_PAGE_ADDRESS,    0xB0, "Set Page Address to 0" },
    { CMD_CONTRAST_REG,   0x81, "Contrast Register Select" },
    { CMD_CONTRAST_VAL,   0xCF, "Contrast Value (0x00 - 0xFF)" },
    { CMD_SEG_REMAP,      0xA1, "Segment Re-map (Horizontal Flip for normal layout)" },
    { CMD_SCAN_DIR,       0xC8, "COM Output Scan Direction (Vertical Flip for normal layout)" },
    { CMD_NORMAL_MODE,    0xA6, "Normal Display Mode (Non-Inverted)" },
    { CMD_MULTIPLEX_REG,  0xA8, "Multiplex Ratio Register Select" },
    { CMD_MULTIPLEX_VAL,  0x3F, "Multiplex Value (1/64 Duty)" },
    { CMD_OFFSET_REG,     0xD3, "Display Offset Register Select" },
    { CMD_OFFSET_VAL,     0x00, "Display Offset Value (0)" },
    { CMD_CLK_DIV_REG,    0xD5, "Display Clock Divide Register Select" },
    { CMD_CLK_DIV_VAL,    0x80, "Display Clock Divide Value (Reset Default)" },
    { CMD_PRECHARGE_REG,  0xD9, "Pre-charge Period Register Select" },
    { CMD_PRECHARGE_VAL,  0xF1, "Pre-charge Period Value" },
    { CMD_COMPINS_REG,    0xDA, "COM Pins Hardware Config Register Select" },
    { CMD_COMPINS_VAL,    0x12, "COM Pins Hardware Config Value" },
    { CMD_VCOMH_REG,      0xDB, "VCOMH Deselect Level Register Select" },
    { CMD_VCOMH_VAL,      0x40, "VCOMH Deselect Level Value" },
    { CMD_DISPLAY_ON,     0xAF, "Display Normal Mode (ON)" }
};


/* ----- Helper function prototypes START. ----- */

static void setCursor(const oled_sh1106_instance_t *dev, const uint8_t page, const uint8_t column);
static void oled_send_cmd(const oled_sh1106_instance_t *dev, uint8_t* cmd, uint32_t len);
static void oled_send_data(const oled_sh1106_instance_t *dev, const uint8_t* data, uint32_t len);

/* ----- Helper functions prototypes END. ----- */

/* ----- Helper functions START. ----- */


/**
  * @brief: Helper for setting cursor. It takes care of formatting the set address command
  * 	    before sending it to device.
  * @param: dev: Pointer to target device.
  * @param: page: Page address.
  * @param: column: Column address.
  * @retval: None
  */
static void setCursor(const oled_sh1106_instance_t *dev, const uint8_t page, const uint8_t column) {
	uint8_t cmd[3] = {
			(0xB0 | page), /* Page address. */
			column & 0x0F, /* Lower column address. */
			(0x10 | ((column >> 4) & 0x0F)) /* Upper column adddress. */
	};
	oled_send_cmd(dev, cmd, 3); /* Send starting address to clear. */
}


/**
  * @brief: Sends commands to device.
  * @param: dev: Pointer to target device.
  * @param: data: Pointer to command buffer.
  * @param: len: Length of command to send.
  * @retval: None
  */
static void oled_send_cmd(const oled_sh1106_instance_t *dev, uint8_t* cmd, uint32_t len) {
	/* Pull DC pin LOW to send command. */
	dev->gpio_write(dev->dc.port, dev->dc.pin, 0);

	/* Pull CS pin LOW to select device. */
	dev->gpio_write(dev->cs.port, dev->cs.pin, 0);

	dev->spiTransmit(dev->spiHandle, cmd, len);

	/* Pull CS pin HIGH to deselect device. */
	dev->gpio_write(dev->cs.port, dev->cs.pin, 1);
}


/**
  * @brief: Sends data to device.
  * @note: Device automatically increments column address but not page address.
  * 	   To go to next page, you must state both the page and column address.
  * @param: dev: Pointer to target device.
  * @param: data: Pointer to data buffer.
  * @param: len: Length of pixels to write.
  * @retval: None
  */
static void oled_send_data(const oled_sh1106_instance_t *dev, const uint8_t* data, uint32_t len) {
	/* Pull DC pin HIGH to send data. */
	dev->gpio_write(dev->dc.port, dev->dc.pin, 1);

	/* Pull CS pin LOW to select device. */
	dev->gpio_write(dev->cs.port, dev->cs.pin, 0);

	dev->spiTransmit(dev->spiHandle, data, len);

	/* Pull CS pin HIGH to deselect device. */
	dev->gpio_write(dev->cs.port, dev->cs.pin, 1);
}



/* ----- Helper functions END. ----- */


/* ----- APIs START. ----- */
/**
  * @brief: Sets cursor to beginning column of a character at specific row.
  * @param: dev: Pointer to target device.
  * @param: linePos: Page# (row) from 0 to 7
  * @param: charPos: Character position from 0 to n-1. n= 21 characters for 5x7 font size.
  * @retval: None
  */
void sh1106_setAtCharacter(oled_sh1106_instance_t *dev, uint8_t linePos, uint8_t charPos) {
	/* Negative wraps to big number beyond allowable for 5x7 font. */
	if(linePos > 7 || charPos > 20) {
		return;
	}

	dev->cursor.page = linePos;

	/* Width of each character is 6px and offset of 3px from left to get even padding
	 * on both left and right. */
	dev->cursor.column = (charPos * 6) + OLED_START_COLUMN;

	setCursor(dev, dev->cursor.page, dev->cursor.column);
}

/**
  * @brief: Clears the portion of the screen at current line ONLY!
  * @note: This does not change the instance's cursor position, but it will change the device's internal
  * 	   memory pointer. So before this function ends, it must send the instance's current cursor position
  * 	   to the device to sync the hardware and software.
  * @param: dev: Pointer to target device.
  * @param: start:Character position starting from position 0 to n-1. n = 21 characters for 5x7 font size.
  * @param: charCount: Number of characters from 1 <= charCount <= n - start. n = 21 for 21 characters for 5x7 font size.
  * @retval: None
  */
void sh1106_clearPortion(const oled_sh1106_instance_t *dev, uint8_t startChar, uint8_t charCount) {
	if(startChar > 20) {
		return;
	}

	/* Find remaining character for the line. */
	uint8_t remCharCount = 21 - startChar;
	if(charCount < 1 || charCount > remCharCount) {
		return;
	}

	/* Convert start to column #, taking into account the 3px offset. */
	startChar = (startChar * 6) + OLED_START_COLUMN;

	/* Same page but may be different column from current cursor position. */
	setCursor(dev, dev->cursor.page, startChar);

	/* Clear the portion. No offset needed since this is just the width of each character. */
	uint8_t clearColumnSize = charCount * 6;

	oled_send_data(dev, clearPixels, clearColumnSize);

	/* Set device's internal pointer back to original position. */
	setCursor(dev, dev->cursor.page, dev->cursor.column);
}

/**
  * @brief: Clears each pages (rows) from @start to @end.
  * @note: This does not change the instance's cursor position, but it will change the device's internal
  * 	   memory pointer. So before this function ends, it must send the instance's current cursor position
  * 	   to the device to sync the hardware and software.
  * 	   Also, if end < start, start and end will be swapped.
  * @param: dev: Pointer to target device.
  * @param: start: starting page # from 0 to 7. This will wrap to a large number if negative.
  * @param: end: ending page # from 0 to 7. This will wrap to a large number if negative.
  * @retval: None
  */
void sh1106_clearPages(oled_sh1106_instance_t *dev, uint8_t start, uint8_t end) {

	if(start > 7 || end > 7) {
		return;
	}

	/* Swap if necessary. */
	if(start > end) {
		uint8_t temp = start;
		start = end;
		end = temp;
	}

	uint8_t pageCount = (end - start) + 1; /* +1 because this is the count not index. */
	for(int i = 0; i < pageCount; ++i) {
		/* When clearing always start at Column 2 to remove any unwanted write to 2px in case of bugs. */
		setCursor(dev, start + i, OLED_START_COLUMN);
		oled_send_data(dev, clearPixels, 128);
	}

	/* Set device's internal pointer back to original position. */
	setCursor(dev, dev->cursor.page, dev->cursor.column);
}

/**
  * @brief: Sets cursor to beginning column of next page (row/line).
  * @param: dev: Pointer to target device.
  * @retval: None
  */
void sh1106_nextLine(oled_sh1106_instance_t *dev) {
	dev->cursor.page = (dev->cursor.page + 1) % 7;
	dev->cursor.column = OLED_START_COLUMN;

	setCursor(dev, dev->cursor.page, dev->cursor.column);
}

/**
  * @brief: Sets cursor to beginning column of previous page (row/line).
  * @param: dev: Pointer to target device.
  * @retval: None
  */
void sh1106_previousLine(oled_sh1106_instance_t *dev) {
	dev->cursor.page = (dev->cursor.page == 0) ? 7 : dev->cursor.page - 1;
	dev->cursor.column = OLED_START_COLUMN;

	setCursor(dev, dev->cursor.page, dev->cursor.column);
}

/**
  * @brief: Set each pixel to clearPixels' value.
  * @param: dev: Pointer to target device.
  * @retval: None
  */
void sh1106_clearScreen(oled_sh1106_instance_t *dev) {
	/* Command to set starting address to write. */
	dev->cursor.page = 0;
	/* Start at column #3 so that each line has equal padding: <-1px-> <-126px-> <-1px-> == 128px. */
	dev->cursor.column = OLED_START_COLUMN;


	/* When clearing screen the offset used is 2. When writing, this will be 3. */
	for(int i = 0; i < OLED_PAGE_COUNT; ++i) {
		setCursor(dev, dev->cursor.page + i, 2); /* Send starting address to clear. */
		oled_send_data(dev, clearPixels, 128);
	}


	setCursor(dev, dev->cursor.page, dev->cursor.column); /* Reset cursor position to beginning. */
}


/**
  * @brief: Initializes the hardware and clears the screen.
  * @param: dev: Pointer to target device.
  * @retval: 1 on success. Otherwise, 0 on failure.
  */
uint8_t sh1106_init(oled_sh1106_instance_t *dev) {
	if(dev == NULL || dev->spiHandle == NULL) {
		return 0; /* Failed to initialize. */
	}

    /* 1. Hardware reset cycle to clear display internal registers */
	/* Pull NRESET pin LOW to start resetting device. */
	dev->gpio_write(dev->nreset.port, dev->nreset.pin, 0);
	dev->delay_ms(10);

	/* Pull NRESET pin HIGH to END resetting device. */
	dev->gpio_write(dev->nreset.port, dev->nreset.pin, 1);
	dev->delay_ms(10);

    /* 2. Fetch the total number of map entries dynamically */
    uint16_t map_entries = sizeof(SH1106_InitMap) / sizeof(OLED_CommandMap_t);

    /* Create a local array scratchpad to aggregate just the raw bytes for transmission */
    uint8_t raw_bytes[CMD_TOTAL_COUNT];

    /* 3. Extract the byte payloads sequentially from the structured command database */
    for (uint16_t i = 0; i < map_entries; i++) {
        raw_bytes[i] = SH1106_InitMap[i].value;
    }

    /* 4. Send command stream. */
    oled_send_cmd(dev, raw_bytes, map_entries);

    /* Clears screen and sets cursor to beginning. */
    sh1106_clearScreen(dev);

    dev->initialized = 1;

    return 1; /* Success.*/
}


/**
  * @brief: Sets cursor to beginning column of next page (row/line).
  * @note: This function automatically go to next line. It will keep wrapping around the line.
  * 	   Be aware that having more than 21 characters to write will lead to unexpected behavior.
  * @param: dev: Pointer to target device.
  * @param: msg: Pointer buffer holding the text to write.
  * @param: len: Number of characters to write. 1 to 21 for 5x7 font size.
  * @retval: None
  */
void sh1106_write_line(oled_sh1106_instance_t *dev, char *msg, uint8_t len) {
	uint8_t blank = 0;
	uint32_t curIndex;
	for(int i = 0; i < len; ++i) {
		curIndex = msg[i] * 5; /* ASCII decimal of 'A'. */
		oled_send_data(dev, (font + curIndex), 5);
		oled_send_data(dev, &blank, 1);
		dev->cursor.column = dev->cursor.column + SH1106_FONT_WIDTH + 1;
		if(dev->cursor.column > 128) { /* 3 to 128 = 126px, and 126px / 6 = 21 characters per page. */
			dev->cursor.page = (dev->cursor.page + 1) % 8; /* Wraps to 1st page (row #1). */
			dev->cursor.column = 3;
			setCursor(dev, dev->cursor.page, dev->cursor.column);
		}
	}
}


/* ----- APIs END. ----- */




