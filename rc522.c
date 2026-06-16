#include "hardware.h"
#include "rc522.h"
#include <INTRINS.H>

/* MFRC522 register addresses */
#define COMMAND_REG         0x01
#define TX_CONTROL_REG      0x14
#define TX_ASK_REG          0x15
#define MODE_REG            0x11
#define T_MODE_REG          0x2A
#define T_PRESCALER_REG     0x2B
#define T_RELOAD_H_REG      0x2C
#define T_RELOAD_L_REG      0x2D
#define VERSION_REG         0x37

#define PCD_SOFT_RESET      0x0F

static void rc522_delay(void)
{
    unsigned int i;

    for (i = 0; i < 1000; ++i) {
        _nop_();
    }
}

static unsigned char rc522_spi_transfer(unsigned char value)
{
    unsigned char i;
    unsigned char received;

    received = 0;

    for (i = 0; i < 8; ++i) {
        RC522_MOSI = (value & 0x80) ? 1 : 0;
        value <<= 1;

        RC522_SCK = 1;
        received <<= 1;
        if (RC522_MISO) {
            received |= 0x01;
        }
        RC522_SCK = 0;
    }

    return received;
}

void rc522_spi_init(void)
{
    RC522_NSS = 1;
    RC522_SCK = 0;
    RC522_MOSI = 0;
    RC522_MISO = 1;
}

void rc522_write_register(unsigned char address, unsigned char value)
{
    RC522_NSS = 0;
    rc522_spi_transfer((address << 1) & 0x7E);
    rc522_spi_transfer(value);
    RC522_NSS = 1;
}

unsigned char rc522_read_register(unsigned char address)
{
    unsigned char value;

    RC522_NSS = 0;
    rc522_spi_transfer(((address << 1) & 0x7E) | 0x80);
    value = rc522_spi_transfer(0x00);
    RC522_NSS = 1;

    return value;
}

static void rc522_set_bits(unsigned char address, unsigned char mask)
{
    rc522_write_register(address, rc522_read_register(address) | mask);
}

static void rc522_antenna_on(void)
{
    if ((rc522_read_register(TX_CONTROL_REG) & 0x03) != 0x03) {
        rc522_set_bits(TX_CONTROL_REG, 0x03);
    }
}

unsigned char rc522_init(void)
{
    unsigned char timeout;
    unsigned char version;

    rc522_spi_init();

    RC522_RST = 0;
    rc522_delay();
    RC522_RST = 1;
    rc522_delay();

    rc522_write_register(COMMAND_REG, PCD_SOFT_RESET);
    timeout = 100;
    while (timeout > 0) {
        if ((rc522_read_register(COMMAND_REG) & 0x10) == 0) {
            break;
        }
        --timeout;
    }

    rc522_write_register(T_MODE_REG, 0x8D);
    rc522_write_register(T_PRESCALER_REG, 0x3E);
    rc522_write_register(T_RELOAD_L_REG, 30);
    rc522_write_register(T_RELOAD_H_REG, 0);
    rc522_write_register(TX_ASK_REG, 0x40);
    rc522_write_register(MODE_REG, 0x3D);
    rc522_antenna_on();

    version = rc522_read_register(VERSION_REG);
    return version;
}
