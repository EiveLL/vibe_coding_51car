#include "hardware.h"
#include "rc522.h"
#include <INTRINS.H>

/* MFRC522 register addresses */
#define COMMAND_REG         0x01
#define COMM_IEN_REG        0x02
#define COMM_IRQ_REG        0x04
#define DIV_IRQ_REG         0x05
#define ERROR_REG           0x06
#define STATUS2_REG         0x08
#define FIFO_DATA_REG       0x09
#define FIFO_LEVEL_REG      0x0A
#define CONTROL_REG         0x0C
#define BIT_FRAMING_REG     0x0D
#define COLL_REG            0x0E
#define TX_CONTROL_REG      0x14
#define TX_ASK_REG          0x15
#define MODE_REG            0x11
#define T_MODE_REG          0x2A
#define T_PRESCALER_REG     0x2B
#define T_RELOAD_H_REG      0x2C
#define T_RELOAD_L_REG      0x2D
#define VERSION_REG         0x37

#define PCD_IDLE            0x00
#define PCD_TRANSCEIVE      0x0C
#define PCD_SOFT_RESET      0x0F

#define PICC_REQIDL         0x26
#define PICC_ANTICOLL       0x93

static void rc522_delay(void)
{
    unsigned int i;

    for (i = 0; i < 1000; ++i) {
        _nop_();
    }
}

static void rc522_spi_delay(void)
{
    _nop_();
    _nop_();
    _nop_();
}

static unsigned char rc522_spi_transfer(unsigned char value)
{
    unsigned char i;
    unsigned char received;

    received = 0;

    for (i = 0; i < 8; ++i) {
        RC522_SCK = 0;
        RC522_MOSI = (value & 0x80) ? 1 : 0;
        rc522_spi_delay();
        value <<= 1;

        RC522_SCK = 1;
        rc522_spi_delay();
        received <<= 1;
        if (RC522_MISO) {
            received |= 0x01;
        }
        rc522_spi_delay();
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

static void rc522_clear_bits(unsigned char address, unsigned char mask)
{
    rc522_write_register(address, rc522_read_register(address) & ~mask);
}

static void rc522_antenna_on(void)
{
    if ((rc522_read_register(TX_CONTROL_REG) & 0x03) != 0x03) {
        rc522_set_bits(TX_CONTROL_REG, 0x03);
    }
}

static unsigned char rc522_to_card(unsigned char command,
                                   unsigned char *send_data,
                                   unsigned char send_len,
                                   unsigned char *back_data,
                                   unsigned int *back_bits,
                                   unsigned char max_back_len)
{
    unsigned char status;
    unsigned char irq_en;
    unsigned char wait_irq;
    unsigned char last_bits;
    unsigned char n;
    unsigned char i;
    unsigned int timeout;

    status = RC522_ERR;
    irq_en = 0x00;
    wait_irq = 0x00;

    if (command == PCD_TRANSCEIVE) {
        irq_en = 0x77;
        wait_irq = 0x30;
    }

    rc522_write_register(COMM_IEN_REG, irq_en | 0x80);
    rc522_clear_bits(COMM_IRQ_REG, 0x80);
    rc522_set_bits(FIFO_LEVEL_REG, 0x80);
    rc522_write_register(COMMAND_REG, PCD_IDLE);

    for (i = 0; i < send_len; ++i) {
        rc522_write_register(FIFO_DATA_REG, send_data[i]);
    }

    rc522_write_register(COMMAND_REG, command);
    if (command == PCD_TRANSCEIVE) {
        rc522_set_bits(BIT_FRAMING_REG, 0x80);
    }

    timeout = 2000;
    do {
        n = rc522_read_register(COMM_IRQ_REG);
        --timeout;
    } while ((timeout != 0) && ((n & 0x01) == 0) && ((n & wait_irq) == 0));

    rc522_clear_bits(BIT_FRAMING_REG, 0x80);

    if (timeout != 0) {
        if ((rc522_read_register(ERROR_REG) & 0x1B) == 0) {
            status = RC522_OK;

            if (n & irq_en & 0x01) {
                status = RC522_NOTAG;
            }

            if (command == PCD_TRANSCEIVE) {
                n = rc522_read_register(FIFO_LEVEL_REG);
                last_bits = rc522_read_register(CONTROL_REG) & 0x07;

                if (last_bits) {
                    *back_bits = ((unsigned int)(n - 1) * 8) + last_bits;
                } else {
                    *back_bits = (unsigned int)n * 8;
                }

                if (n == 0) {
                    n = 1;
                }
                if (n > max_back_len) {
                    n = max_back_len;
                }

                for (i = 0; i < n; ++i) {
                    back_data[i] = rc522_read_register(FIFO_DATA_REG);
                }
            }
        }
    }

    return status;
}

static unsigned char rc522_request(unsigned char req_mode, unsigned char *tag_type)
{
    unsigned int back_bits;
    unsigned char status;

    rc522_write_register(BIT_FRAMING_REG, 0x07);
    tag_type[0] = req_mode;

    status = rc522_to_card(PCD_TRANSCEIVE, tag_type, 1, tag_type, &back_bits, 2);
    if ((status != RC522_OK) || (back_bits != 0x10)) {
        status = RC522_ERR;
    }

    return status;
}

static unsigned char rc522_anticoll(unsigned char *uid)
{
    unsigned char status;
    unsigned char i;
    unsigned char checksum;
    unsigned int back_bits;
    unsigned char buffer[5];

    rc522_write_register(BIT_FRAMING_REG, 0x00);
    rc522_write_register(COLL_REG, 0x80);

    buffer[0] = PICC_ANTICOLL;
    buffer[1] = 0x20;

    status = rc522_to_card(PCD_TRANSCEIVE, buffer, 2, buffer, &back_bits, 5);
    if (status == RC522_OK) {
        checksum = 0;
        for (i = 0; i < 4; ++i) {
            uid[i] = buffer[i];
            checksum ^= buffer[i];
        }

        if (checksum != buffer[4]) {
            status = RC522_ERR;
        }
    }

    return status;
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

unsigned char rc522_read_uid(unsigned char *uid)
{
    unsigned char tag_type[2];

    if (rc522_request(PICC_REQIDL, tag_type) != RC522_OK) {
        return RC522_NOTAG;
    }

    return rc522_anticoll(uid);
}
