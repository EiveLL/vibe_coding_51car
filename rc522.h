#ifndef __RC522_H__
#define __RC522_H__

#define RC522_VERSION_V1       0x91
#define RC522_VERSION_V2       0x92
#define RC522_VERSION_CLONE    0x88

void rc522_spi_init(void);
unsigned char rc522_init(void);
unsigned char rc522_read_register(unsigned char address);
void rc522_write_register(unsigned char address, unsigned char value);

#endif
