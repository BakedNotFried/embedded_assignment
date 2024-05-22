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
extern SemaphoreHandle_t xI2C2BusMutex;
extern TaskHandle_t xTaskToNotify;

// enum for checking which sensor is using the I2C bus
enum sensorType {NONE, OPT3001, BMI160};
extern enum sensorType g_I2C_flag;

// void resetI2CPeripheral(uint32_t ui32Base)
// {
//     // Disable the I2C master module
//     I2CMasterDisable(ui32Base);

//     // Flush the FIFO if supported
//     I2CMasterControl(ui32Base, I2C_MASTER_CMD_FIFO_FLUSH);

//     // Re-enable the I2C master module
//     I2CMasterEnable(ui32Base);

//     // Reinitialize the I2C master module
//     I2CMasterInitExpClk(ui32Base, SysCtlClockGet(), false);
// }

int8_t readI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    uint32_t ulNotificationValue;

    if (xSemaphoreTake(xI2C2BusMutex, portMAX_DELAY) == pdPASS)
    {
        xTaskToNotify = xTaskGetCurrentTaskHandle();

        // Set flag
        g_I2C_flag = BMI160;

        // Load device slave address
        I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

        // Place the character to be sent in the data register
        I2CMasterDataPut(I2C2_BASE, ui8Reg);
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
        
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while(I2CMasterBusBusy(I2C2_BASE)) {}
        if ( (ulNotificationValue == 0) || (I2CMasterErr(I2C2_BASE) != I2C_MASTER_ERR_NONE) )
        {
            g_I2C_flag = NONE;
            xSemaphoreGive(xI2C2BusMutex);
            UARTprintf("Error1\n");
            return false;
        }

        I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while(I2CMasterBusBusy(I2C2_BASE)) {}
        if ( (ulNotificationValue == 0) || (I2CMasterErr(I2C2_BASE) != I2C_MASTER_ERR_NONE) )
        {
            g_I2C_flag = NONE;
            xSemaphoreGive(xI2C2BusMutex);
            UARTprintf("Error1\n");
            return false;
        }

        data[0] = I2CMasterDataGet(I2C2_BASE);
        
        g_I2C_flag = NONE;
        xSemaphoreGive(xI2C2BusMutex);
        return true;
    }
    // Set flag
    g_I2C_flag = NONE;
    return false;
    // UARTprintf("R2d\n");    
}

// int8_t readI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
// {
//     // Set flag
//     g_I2C_flag = BMI160;

//     // Load device slave address
//     I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

//     // Place the character to be sent in the data register
//     I2CMasterDataPut(I2C2_BASE, ui8Reg);
//     I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    
//     // UARTprintf("R1\n");
//     // Wait on semaphore
//     if( xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
//     {
//         // UARTprintf("");
//         // Load device slave address
//         while(I2CMasterBusy(I2C2_BASE)) {}
//         if (I2CMasterErr(I2C2_BASE) != I2C_MASTER_ERR_NONE) {
//             UARTprintf("Error1\n");
//             xSemaphoreGive(xI2C0BMISemaphore);  // Release the mutex before returning
//             return false;
//         }
//         I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);
//         while(I2CMasterBusy(I2C2_BASE)) {}
//         if (I2CMasterErr(I2C2_BASE) != I2C_MASTER_ERR_NONE) {
//             UARTprintf("Error1\n");
//             xSemaphoreGive(xI2C0BMISemaphore);  // Release the mutex before returning
//             return false;
//         }
//         // Read two bytes from I2C
//         I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
//     }
//     else
//     {
//         UARTprintf("Error taking semaphore1\n");
//     }
//     // UARTprintf("R1d\n");
//     // UARTprintf("Read1 \n");
//     // Wait on semaphore
//     if( xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
//     {
//         // UARTprintf("s11\n");
//         data[0] = I2CMasterDataGet(I2C2_BASE);
//         // UARTprintf("s22\n");
//         // Set flag
//         g_I2C_flag = NONE;
//         return true;
//     }
//     else
//     {
//         UARTprintf("Error taking semaphore2\n");
//     }
//     // Set flag
//     g_I2C_flag = NONE;
//     return false;
//     // UARTprintf("R2d\n");    
// }

int8_t writeI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    uint32_t ulNotificationValue;

    if (xSemaphoreTake(xI2C2BusMutex, portMAX_DELAY) == pdPASS)
    {
        xTaskToNotify = xTaskGetCurrentTaskHandle();

        // Set flag
        g_I2C_flag = BMI160;

        UARTprintf("W1\n");
        // Load device slave address
        I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);
        UARTprintf("W2\n");
        // Place the character to be sent in the data register
        I2CMasterDataPut(I2C2_BASE, ui8Reg);
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotificationValue == 0) {
            g_I2C_flag = NONE;
            xSemaphoreGive(xI2C2BusMutex);
            return false;
        }

        // Send Data
        I2CMasterDataPut(I2C2_BASE, data[0]);
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotificationValue == 0) {
            g_I2C_flag = NONE;
            xSemaphoreGive(xI2C2BusMutex);
            return false;
        }

        // Set flag
        g_I2C_flag = NONE;
        xSemaphoreGive(xI2C2BusMutex);
        return true;
    }

    // Set flag
    g_I2C_flag = NONE;
    return false;
}

// int8_t writeI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
// {
//     // Set flag
//     g_I2C_flag = BMI160;

//     // Load device slave address
//     I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

//     // Place the character to be sent in the data register
//     I2CMasterDataPut(I2C2_BASE, ui8Reg);
//     I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

//     // Wait on semaphore
//     if( xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
//     {
//         // Send Data
//         I2CMasterDataPut(I2C2_BASE, data[0]);
//         I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
//     }

//     // Delay until transmission completes
//     if( xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
//     {
//         // Set flag
//         g_I2C_flag = NONE;
//         return true;
//     }
//     // Set flag
//     g_I2C_flag = NONE;
//     return false;
// }

// int8_t readI2CBMI(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
// {
//     // Set Flag
//     g_I2C_flag = BMI160;

//     // Load device slave address
//     I2CMasterSlaveAddrSet(I2C2_BASE, id, false);

//     // Place the register address to be sent in the data register
//     I2CMasterDataPut(I2C2_BASE, reg_addr);
//     I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);

//     // Wait on semaphore
//     if (xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
//     {
//         // Load device slave address for reading
//         I2CMasterSlaveAddrSet(I2C2_BASE, id, true);

//         // Read the specified number of bytes from I2C
//         for (uint16_t i = 0; i < len; i++)
//         {
//             if (i == (len - 1))
//             {
//                 // Last byte to read
//                 I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
//             }
//             else
//             {
//                 // More bytes to read
//                 I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
//             }

//             // Wait on semaphore
//             if (xSemaphoreTake(xI2C0BMISemaphore, portMAX_DELAY) == pdPASS)
//             {
//                 data[i] = I2CMasterDataGet(I2C2_BASE);
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