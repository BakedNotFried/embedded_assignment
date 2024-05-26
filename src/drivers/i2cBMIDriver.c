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

bool readI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    bool success = false;

    // Set flag
    g_I2C_flag = BMI160;

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    // Wait on semaphore
    if (xSemaphoreTake(xI2C0BMISemaphore, pdMS_TO_TICKS(1000)) == pdPASS)
    {
        // Load device slave address
        I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);

        // Read two bytes from I2C
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

        // Wait on semaphore
        if (xSemaphoreTake(xI2C0BMISemaphore, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            data[0] = I2CMasterDataGet(I2C2_BASE);
            success = true;
        }
    }

    // Set flag
    g_I2C_flag = NONE;

    return success;
}

bool writeI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    bool success = false;

    // Set flag
    g_I2C_flag = BMI160;

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait on semaphore
    if (xSemaphoreTake(xI2C0BMISemaphore, pdMS_TO_TICKS(1000)) == pdPASS)
    {
        // Send Data
        I2CMasterDataPut(I2C2_BASE, data[0]);
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

        // Delay until transmission completes
        if (xSemaphoreTake(xI2C0BMISemaphore, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            success = true;
        }
    }

    // Set flag
    g_I2C_flag = NONE;

    return success;
}