/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include <stdio.h>
#include <xc.h>
#include<math.h>       // I like to do math
#include"ILI9163C.h"
#include"I2C2_Commands.h"

#define messageLen 100
#define CS LATBbits.LATB7       // chip select pin

// Global Variables
char writeSerialDataFlag = 0;
char freqUpdate = 100;
char maxNum = 100+1;

char message[messageLen];


// Definitions to make life easier
#define CS LATBbits.LATB7       // chip select pin
#define IMU_Address 0b1101011 // Address for the IMU
#define reg_Accel 0x10
#define reg_Gyro 0x11
#define reg_Contr 0x12
#define reg_OUT_TEMP_L 0x20


// Color Definitions
// Color: RRRRR GGGGGG BBBBB
#define colorRED  0b1111100000000000
#define colorGREEN 0b0000001111100000
#define colorBLUE 0b0000000000011111
#define colorPURPLE 0b1011100000011111
#define colorCYAN 0b0000011100011100
#define colorYELLOW 0b1111111111100000
#define colorORANGE 0b1111101011100000
#define colorBLACK 0b0000000000000000
#define colorWHITE 0b1111111111111111

// Character Dimensions
#define charWidth 5
#define charHeight 8
#define screenWidth 130
#define screenHeight 130

void SPI1_init() {
    SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
    RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
    TRISBbits.TRISB7 = 0; // SS is B7
    LATBbits.LATB7 = 1; // SS starts high

    // A0 / DAT pin
    ANSELBbits.ANSB15 = 0;
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;

    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi1
}

unsigned char spi_io(unsigned char o) {
    SPI1BUF = o;
    while (!SPI1STATbits.SPIRBF) { // wait to receive the byte
        ;
    }
    return SPI1BUF;
}

void LCD_command(unsigned char com) {
    LATBbits.LATB15 = 0; // DAT
    CS = 0; // CS
    spi_io(com);
    CS = 1; // CS
}

void LCD_data(unsigned char dat) {
    LATBbits.LATB15 = 1; // DAT
    CS = 0; // CS
    spi_io(dat);
    CS = 1; // CS
}

void LCD_data16(unsigned short dat) {
    LATBbits.LATB15 = 1; // DAT
    CS = 0; // CS
    spi_io(dat >> 8);
    spi_io(dat);
    CS = 1; // CS
}

void LCD_init() {
    int time = 0;
    LCD_command(CMD_SWRESET); //software reset
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 2) {
    } //delay(500);

    LCD_command(CMD_SLPOUT); //exit sleep
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 200) {
    } //delay(5);

    LCD_command(CMD_PIXFMT); //Set Color Format 16bit
    LCD_data(0x05);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 200) {
    } //delay(5);

    LCD_command(CMD_GAMMASET); //default gamma curve 3
    LCD_data(0x04); //0x04
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_GAMRSEL); //Enable Gamma adj
    LCD_data(0x01);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_NORML);

    LCD_command(CMD_DFUNCTR);
    LCD_data(0b11111111);
    LCD_data(0b00000110);

    int i = 0;
    LCD_command(CMD_PGAMMAC); //Positive Gamma Correction Setting
    for (i = 0; i < 15; i++) {
        LCD_data(pGammaSet[i]);
    }

    LCD_command(CMD_NGAMMAC); //Negative Gamma Correction Setting
    for (i = 0; i < 15; i++) {
        LCD_data(nGammaSet[i]);
    }

    LCD_command(CMD_FRMCTR1); //Frame Rate Control (In normal mode/Full colors)
    LCD_data(0x08); //0x0C//0x08
    LCD_data(0x02); //0x14//0x08
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_DINVCTR); //display inversion
    LCD_data(0x07);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_PWCTR1); //Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
    LCD_data(0x0A); //4.30 - 0x0A
    LCD_data(0x02); //0x05
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_PWCTR2); //Set BT[2:0] for AVDD & VCL & VGH & VGL
    LCD_data(0x02);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_VCOMCTR1); //Set VMH[6:0] & VML[6:0] for VOMH & VCOML
    LCD_data(0x50); //0x50
    LCD_data(99); //0x5b
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_VCOMOFFS);
    LCD_data(0); //0x40
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_CLMADRS); //Set Column Address
    LCD_data16(0x00);
    LCD_data16(_GRAMWIDTH);

    LCD_command(CMD_PGEADRS); //Set Page Address
    LCD_data16(0x00);
    LCD_data16(_GRAMHEIGH);

    LCD_command(CMD_VSCLLDEF);
    LCD_data16(0); // __OFFSET
    LCD_data16(_GRAMHEIGH); // _GRAMHEIGH - __OFFSET
    LCD_data16(0);

    LCD_command(CMD_MADCTL); // rotation
    LCD_data(0b00001000); // bit 3 0 for RGB, 1 for GBR, rotation: 0b00001000, 0b01101000, 0b11001000, 0b10101000

    LCD_command(CMD_DISPON); //display ON
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_RAMWR); //Memory Write
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
    // check boundary
    LCD_setAddr(x, y, x + 1, y + 1);
    LCD_data16(color);
}

void LCD_setAddr(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1) {
    LCD_command(CMD_CLMADRS); // Column
    LCD_data16(x0);
    LCD_data16(x1);

    LCD_command(CMD_PGEADRS); // Page
    LCD_data16(y0);
    LCD_data16(y1);

    LCD_command(CMD_RAMWR); //Into RAM
}

void LCD_clearScreen(unsigned short color) {
    int i;
    LCD_setAddr(0, 0, _GRAMWIDTH, _GRAMHEIGH);
    for (i = 0; i < _GRAMSIZE; i++) {
        LCD_data16(color);
    }
}

void draw_Character(unsigned short x, unsigned short y, unsigned short character, unsigned short frontColor, unsigned short backColor, float fontSize) {
    // Don't draw off the screen
    if (((x + charWidth) <= screenWidth)&&((y + charHeight) <= screenHeight)) {
        // Initialize counter Variables
        int i = 0;
        int j = 0;
        // Loop through columns
        for (i = 0; i < charWidth * fontSize; i++) {
            // Loop through rows
            for (j = 0; j < charHeight * fontSize; j++) {
                // Check if bit at row,column of selected character is 1 or 0 then assign color
                if (((ASCII[character - 0x20][(int) (i / fontSize)])&(0b00000001 << ((int) (j / fontSize)))) != 0) {
                    LCD_drawPixel(x + i, y + j, frontColor);
                } else {
                    LCD_drawPixel(x + i, y + j, backColor);
                }
            }
        }
    }
}

void draw_Message(unsigned short x, unsigned short y, char* message, unsigned short frontColor, unsigned short backColor, float fontSize) {
    // Loop through all characters in string
    int i = 0;
    while (message[i] != 0) {
        draw_Character(x + i * charWidth*fontSize, y, message[i], frontColor, backColor, fontSize);
        i++;
    }
}

void draw_Rectangle(unsigned short x1, unsigned short x2, unsigned short y1, unsigned short y2, unsigned short frontColor) {
    int i = 0;
    int j = 0;
    // Loop through rows
    for (i = 0; i < (x2 - x1); i++) {
        // Loop through columns
        for (j = 0; j < y2 - y1; j++) {
            LCD_drawPixel(x1 + i, y1 + j, frontColor);
        }
    }
}

void draw_HLine(unsigned short x1, unsigned short y1, int length, int thickness, unsigned short frontColor) {
    int i = 0;
    int j = 0;

    if (length > 0) {
        for (i = 0; i < length; i++) {
            for (j = -thickness / 2; j < thickness / 2; j++) {
                LCD_drawPixel(x1 + i, y1 + j, frontColor);
            }
        }
    } else {
        for (i = 0; i < fabs(length); i++) {
            for (j = -thickness / 2; j < thickness / 2; j++) {
                LCD_drawPixel(x1 - i, y1 + j, frontColor);
            }
        }
    }


}

void draw_VLine(unsigned short x1, unsigned short y1, int length, int thickness, unsigned short frontColor) {
    int i = 0;
    int j = 0;

    if (length > 0) {
        for (i = 0; i < length; i++) {
            for (j = -thickness / 2; j < thickness / 2; j++) {
                LCD_drawPixel(x1 + j, y1 + i, frontColor);
            }
        }
    } else {
        for (i = 0; i < fabs(length); i++) {
            for (j = -thickness / 2; j < thickness / 2; j++) {
                LCD_drawPixel(x1 + j, y1 - i, frontColor);
            }
        }
    }

}

void IMU_init(void) {
    // Initializes the IMU
    unsigned char sendbyte;

    i2c_master_start();
    sendbyte = (IMU_Address << 1) | (0b00000000); // Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(reg_Accel); // CTRL1_XL Register
    i2c_master_send(0b10000010); //1.66 kHz [1000], 2g [00], 100 Hz Filter [10]
    i2c_master_stop();

    i2c_master_start();
    sendbyte = (IMU_Address << 1) | (0b00000000); // Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(reg_Gyro); // CTRL2_G Register
    i2c_master_send(0b10001000); // 1.66kHz [1000], 1000 dps sense [10], [00]
    i2c_master_stop();

    i2c_master_start();
    sendbyte = (IMU_Address << 1) | (0b00000000); // Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(reg_Contr); // CTRL3_C Register
    i2c_master_send(0b00000100); // [00000100]
    i2c_master_stop();
}

void I2C_read_multiple(unsigned char address, unsigned char regRead, unsigned char * data, int length) {
    unsigned char sendbyte;

    // Read from registers
    i2c_master_start();
    sendbyte = (IMU_Address << 1) | (0b00000000); // Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(regRead); // The Register to read
    i2c_master_restart();
    sendbyte = (IMU_Address << 1) | (0b00000001); // Reading    
    i2c_master_send(sendbyte); // Send Device ID with Read Byte

    int i = 0;
    // Read all data
    for (i = 0; i < length; i++) {
        data[i] = i2c_master_recv(); // Receive current register
        if (i == length - 1) {
            i2c_master_ack(1);
        } else {
            i2c_master_ack(0);
        }
    }

    i2c_master_stop(); // End Comms

}

void debugLED(void) {
    // Configure AN4 as output
    ANSELA = 0;
    TRISAbits.TRISA4 = 0;
    // Turn on the LED
    LATAbits.LATA4 = 1;
}

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
int startTime = 0;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    startTime = _CP0_GET_COUNT();
    
    
    // Configure Bits
    ANSELA = 0;
    ANSELB = 0;
    
    // Initialize communications
    SPI1_init();
    i2c_master_setup();
    // Initialize the LCD
    LCD_init();
    LCD_clearScreen(colorWHITE);
    IMU_init();
    
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */


void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */

    switch (appData.state) {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {
                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);

                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

            if (appData.isReadComplete || _CP0_GET_COUNT() - startTime > (48000000 / 2 / freqUpdate)) {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }

            break;


        case APP_STATE_SCHEDULE_WRITE:

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            // Increment Counter
            i++;
            // If haven't met limit, change the report string
            if (i < maxNum) {
                // Read the data
                unsigned char data[14];
                I2C_read_multiple(IMU_Address, reg_OUT_TEMP_L, data, 14);
                // Shift Data into variables to get raw data
                short temperature   = (data[1] << 8)    | data[0];
                short gyroX         = (data[3] << 8)    | data[2];
                short gyroY         = (data[5] << 8)    | data[4];
                short gyroZ         = (data[7] << 8)    | data[6];
                short accelX        = (data[9] << 8)    | data[8];
                short accelY        = (data[11] << 8)   | data[10];
                short accelZ        = (data[13] << 8)   | data[12];
                     
                len = sprintf(dataOut, "%d %d  \r\n", i,accelZ);
            }

            if (appData.isReadComplete) {
                // Check if an 'r' was pushed
                if (appData.readBuffer[0] == 'r') {
                    i = 0;
                }
                // Report observed letter
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle,
                        appData.readBuffer, 1,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);

            } else {
                // Report string if within limit
                if (i < maxNum) {
                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &appData.writeTransferHandle, dataOut, len,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                    startTime = _CP0_GET_COUNT();
                }                    // Otherwise report nothing
                else {
                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &appData.writeTransferHandle, "", 1,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                    startTime = _CP0_GET_COUNT();
                }
            }
            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}



/*******************************************************************************
 End of File
 */
