#ifndef _I2CBMIDRIVER_H_
#define _I2CBMIDRIVER_H_


// ----------------------- Includes -----------------------
#include <stdbool.h>
#include <stdint.h>


// ----------------------- Exported prototypes -----------------------
extern bool readI2CBMI(uint8_t id, uint8_t reg_addr, uint8_t *data);
extern bool writeI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data);


#endif /* _I2CBMIDRIVER_H_ */