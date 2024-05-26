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
#include "semphr.h"

// Semaphore for i2c non-blocking functionality
extern SemaphoreHandle_t xI2C0OPTSemaphore;

// enum for checking which sensor is using the I2C bus
enum sensorType {NONE, OPT3001, IMU};
extern enum sensorType g_I2C_flag;

/*
 * Sets slave address to ui8Addr
 * Puts ui8Reg followed by two data bytes in *data and transfers
 * over i2c
 */
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Set Flag
    g_I2C_flag = OPT3001;

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xI2C0OPTSemaphore, pdMS_TO_TICKS(1000)) == pdPASS)
    {
        // Send Data
        I2CMasterDataPut(I2C2_BASE, data[0]);
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

        if (xSemaphoreTake(xI2C0OPTSemaphore, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            I2CMasterDataPut(I2C2_BASE, data[1]);
            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
            // Delay until transmission completes
            if (xSemaphoreTake(xI2C0OPTSemaphore, pdMS_TO_TICKS(1000)) == pdPASS)
            {
                // Set Flag
                g_I2C_flag = NONE;
                
                return true;
            }
        }
    }

    // Set Flag
    g_I2C_flag = NONE;

    return false;
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
    // Set Flag
    g_I2C_flag = OPT3001;

    uint8_t byteA, byteB;

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    // Wait on semaphore
    if( xSemaphoreTake(xI2C0OPTSemaphore, pdMS_TO_TICKS(1000)) == pdPASS)
    {
        // Load device slave address
        I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);

        // Read two bytes from I2C
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

        if( xSemaphoreTake(xI2C0OPTSemaphore, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            byteA = I2CMasterDataGet(I2C2_BASE);

            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

            if( xSemaphoreTake(xI2C0OPTSemaphore, pdMS_TO_TICKS(1000)) == pdPASS)
            {
                byteB = I2CMasterDataGet(I2C2_BASE);

                data[0] = byteA;
                data[1] = byteB;

                // Set flag
                g_I2C_flag = NONE;

                return true;
            }
        }
    }

    // Set flag
    g_I2C_flag = NONE;
    return false;
}