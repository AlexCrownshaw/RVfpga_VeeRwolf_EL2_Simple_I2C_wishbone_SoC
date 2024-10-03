#include "simple_i2c_hal.h"

uint16_t clk_div = 0x3D;    // 400kHz
uint8_t slave_addr = 0x2A;
uint8_t reg_addr = 0x68;

int main(void)
{
    I2C_HAL_init(clk_div);
    I2C_HAL_write(slave_addr, reg_addr, 0xFF);
    volatile uint8_t read_data = I2C_HAL_read(slave_addr, reg_addr);
}