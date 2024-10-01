#include <stdint.h>

// Register Addresses
#define CTRL_REG_ADDR         0x80003200
#define STATUS_REG_ADDR       0x80003204
#define TX_REG_ADDR           0x80003208
#define RX_REG_ADDR           0x8000320C
#define CLK_DIV_LO_REG_ADDR   0x80003210
#define CLK_DIV_HI_REG_ADDR   0x80003214

// Control regisetr bit masks
#define CTRL_REG_EN_MASK            0x01
#define CTRL_REG_MODE_MASK          0x02
#define CTRL_REG_START_MASK         0x04
#define CTRL_REG_STOP_MASK          0x08
#define CTRL_REG_RW_MASK            0x10
#define CTRL_REG_LD_SLAVE_ADDR_MASK 0x20
#define CTRL_REG_LD_REG_ADDR_MASK   0x40

// Status regsiter bit masks
#define STATUS_REG_BUSY_MASK    0x01
#define STATUS_REG_DONE_MASK    0x02
#define STATUS_REG_NACK_MASK    0x04

#define WRITE_ADDR(dir, value) { (*(volatile unsigned *)dir) = (value); }
#define READ_ADDR(dir) (*(volatile unsigned *)dir)

uint8_t ctrl, status;

int I2C_HAL_init(uint16_t);
int I2C_HAL_write(uint8_t, uint8_t, uint8_t);
uint8_t I2C_HAL_read(uint8_t, uint8_t);
