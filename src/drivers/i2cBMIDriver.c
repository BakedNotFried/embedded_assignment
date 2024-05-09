// ----------------------- Includes -----------------------
#include "i2cBMIDriver.h"
#include "bmi160.h"
#include "bmi160_defs.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"

// Semaphore for i2c non-blocking functionality
extern SemaphoreHandle_t xI2C0BMISemaphore;

// enum for checking which sensor is using the I2C bus
enum sensorType {NONE, OPT3001, BMI160};
extern enum sensorType g_I2C_flag;

int8_t readI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Set flag
    g_I2C_flag = BMI160;

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    // Wait on semaphore
    if( xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
    {
        // Load device slave address
        I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, true);

        // Read two bytes from I2C
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    }
    // Wait on semaphore
    if( xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
    {
        data[0] = I2CMasterDataGet(I2C0_BASE);
        // Set flag
        g_I2C_flag = NONE;
        return true;
    }
    // Set flag
    g_I2C_flag = NONE;
    return false;    
}

int8_t writeI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Set flag
    g_I2C_flag = BMI160;

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait on semaphore
    if( xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
    {
        // Send Data
        I2CMasterDataPut(I2C0_BASE, data[0]);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    }

    // Delay until transmission completes
    if( xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
    {
        // Set flag
        g_I2C_flag = NONE;
        return true;
    }
    // Set flag
    g_I2C_flag = NONE;
    return false;
}

// int8_t readI2CBMI(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
// {
//     // Set Flag
//     g_I2C_flag = BMI160;

//     // Load device slave address
//     I2CMasterSlaveAddrSet(I2C0_BASE, id, false);

//     // Place the register address to be sent in the data register
//     I2CMasterDataPut(I2C0_BASE, reg_addr);
//     I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

//     // Wait on semaphore
//     if (xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
//     {
//         // Load device slave address for reading
//         I2CMasterSlaveAddrSet(I2C0_BASE, id, true);

//         // Read the specified number of bytes from I2C
//         for (uint16_t i = 0; i < len; i++)
//         {
//             if (i == (len - 1))
//             {
//                 // Last byte to read
//                 I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
//             }
//             else
//             {
//                 // More bytes to read
//                 I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
//             }

//             // Wait on semaphore
//             if (xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
//             {
//                 data[i] = I2CMasterDataGet(I2C0_BASE);
//             }
//             else
//             {
//                 // Semaphore error, set flag and return error code
//                 g_I2C_flag = NONE;
//                 return false;
//             }
//         }
//     }
//     else
//     {
//         // Semaphore error, set flag and return error code
//         g_I2C_flag = NONE;
//         return false;
//     }

//     // Set flag
//     g_I2C_flag = NONE;

//     // Return success
//     return BMI160_OK;
// }