#include "simple_i2c_hal.h"


int I2C_HAL_init(uint16_t clk_div)
{
    // Set enable and master mode bits 
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl | (CTRL_REG_EN_MASK | CTRL_REG_MODE_MASK)));
    
    // Set clock divider
    WRITE_ADDR(CLK_DIV_HI_REG_ADDR, ((clk_div >> 8) & 0xFF));
    WRITE_ADDR(CLK_DIV_LO_REG_ADDR, (clk_div & 0xFF));

    return 0;
}

int I2C_HAL_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{

    // Check if I2C core is already busy by checking busy bit
    status = READ_ADDR(STATUS_REG_ADDR);
    if (status & STATUS_REG_BUSY_MASK)
    {
        return -1;
    }

    // Reset read bit (enables write)
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl & ~CTRL_REG_RW_MASK))

    // Set slave address
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl | CTRL_REG_LD_SLAVE_ADDR_MASK));
    WRITE_ADDR(TX_REG_ADDR, slave_addr);
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl & ~CTRL_REG_LD_SLAVE_ADDR_MASK));

    // Set regsiter address
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl | CTRL_REG_LD_REG_ADDR_MASK));
    WRITE_ADDR(TX_REG_ADDR, reg_addr);
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl & ~CTRL_REG_LD_REG_ADDR_MASK));

    // Set write data
    WRITE_ADDR(TX_REG_ADDR, data);

    // Start I2C transaction
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl | CTRL_REG_START_MASK));

    // Wait for transaction to start, check busy bit
    do 
    {
        status = READ_ADDR(STATUS_REG_ADDR);
    }
    while (!(status & STATUS_REG_BUSY_MASK));

    // Reset start bit once transaction has started
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl & ~CTRL_REG_START_MASK));

    return 0;
}

uint8_t I2C_HAL_read(uint8_t slave_addr, uint8_t reg_addr)
{
    // Check if I2C core is already busy by checking busy bit
    status = READ_ADDR(STATUS_REG_ADDR);
    if (status & STATUS_REG_BUSY_MASK)
    {
        return -1;
    }

    // Reset read bit (enables write)
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl | CTRL_REG_RW_MASK))

    // Set slave address
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl | CTRL_REG_LD_SLAVE_ADDR_MASK));
    WRITE_ADDR(TX_REG_ADDR, slave_addr);
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl & ~CTRL_REG_LD_SLAVE_ADDR_MASK));

    // Set regsiter address
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl | CTRL_REG_LD_REG_ADDR_MASK));
    WRITE_ADDR(TX_REG_ADDR, reg_addr);
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl & ~CTRL_REG_LD_REG_ADDR_MASK));

    // Start I2C transaction
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl | CTRL_REG_START_MASK));

    // Wait for transaction to start, check busy bit
    do 
    {
        status = READ_ADDR(STATUS_REG_ADDR);
    }
    while (!(status & STATUS_REG_BUSY_MASK));

    // Reset start bit once transaction has started
    ctrl = READ_ADDR(CTRL_REG_ADDR);
    WRITE_ADDR(CTRL_REG_ADDR, (ctrl & ~CTRL_REG_START_MASK));

    // Wait until transaction has complete
    do
    {
        status = READ_ADDR(STATUS_REG_ADDR);
    }
    while (!(status & STATUS_REG_DONE_MASK));

    // check nack to see if transaction completed successfully

    // Read rx register
    uint8_t data = READ_ADDR(RX_REG_ADDR);

    return data;
}