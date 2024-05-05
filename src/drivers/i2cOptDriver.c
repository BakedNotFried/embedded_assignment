/**************************************************************************************************
*  Filename:       i2cOptDriver.c
*  By:             Jesse Haviland
*  Created:        1 February 2019
*  Revised:        23 March 2019
*  Revision:       2.0
*
*  Description:    i2c Driver for use with opt3001.c and the TI OP3001 Optical Sensor
*************************************************************************************************/

// ----------------------- Includes -----------------------
#include "i2cOptDriver.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Semaphore for i2c non-blocking functionality
extern SemaphoreHandle_t xI2C0Semaphore;

/*
 * Sets slave address to ui8Addr
 * Puts ui8Reg followed by two data bytes in *data and transfers
 * over i2c
 */
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Load device slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    // while(I2CMasterBusy(I2C0_BASE)) { }
    if (xSemaphoreTake(xI2C0Semaphore, portMAX_DELAY) == pdPASS)
    {
        // Send Data
        I2CMasterDataPut(I2C0_BASE, data[0]);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        
    }
    // while(I2CMasterBusy(I2C0_BASE)) { }
    if (xSemaphoreTake(xI2C0Semaphore, portMAX_DELAY) == pdPASS)
    {
        I2CMasterDataPut(I2C0_BASE, data[1]);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    }

    // Delay until transmission completes
    // while(I2CMasterBusBusy(I2C0_BASE)) { }
    if (xSemaphoreTake(xI2C0Semaphore, portMAX_DELAY) == pdPASS)
    {
        return true;
    }

    // return true;
}


/*
 * Sets slave address to ui8Addr
 * Writes ui8Reg over i2c to specify register being read from
 * Reads three bytes from i2c slave. The third is redundant but
 * helps to flush the i2c register
 * Stores first two received bytes into *data
 */
bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    uint8_t byteA, byteB;

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    // Wait on semaphore
    if( xSemaphoreTake(xI2C0Semaphore, portMAX_DELAY) == pdPASS)
    {
        // Load device slave address
        I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, true);

        // Read two bytes from I2C
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    }

    // Wait on semaphore
    if( xSemaphoreTake(xI2C0Semaphore, portMAX_DELAY) == pdPASS)
    {
        byteA = I2CMasterDataGet(I2C0_BASE);

        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    }

    // Wait on semaphore
    if( xSemaphoreTake(xI2C0Semaphore, portMAX_DELAY) == pdPASS)
    {
        byteB = I2CMasterDataGet(I2C0_BASE);

        data[0] = byteA;
        data[1] = byteB;

        return true;
    }
}

// /*
//  * Sets slave address to ui8Addr
//  * Writes ui8Reg over i2c to specify register being read from
//  * Reads three bytes from i2c slave. The third is redundant but
//  * helps to flush the i2c register
//  * Stores first two received bytes into *data
//  */
// bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
// {
//     uint16_t delay = 1000;
//     uint8_t byteA, byteB;

//     // Load device slave address
//     I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

//     // Place the character to be sent in the data register
//     I2CMasterDataPut(I2C0_BASE, ui8Reg);
//     I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
//     while(I2CMasterBusy(I2C0_BASE)) { }

//     // Load device slave address
//     I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, true);

//     // Read two bytes from I2C
//     I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
//     while(I2CMasterBusy(I2C0_BASE)) { }
//     byteA = I2CMasterDataGet(I2C0_BASE);


//     I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
//     SysCtlDelay(delay);
//     byteB = I2CMasterDataGet(I2C0_BASE);

//     data[0] = byteA;
//     data[1] = byteB;

//     return true;
// }


