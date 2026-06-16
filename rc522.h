#ifndef __RC522_H__
#define __RC522_H__

#define RC522_VERSION_V1       0x91
#define RC522_VERSION_V2       0x92
#define RC522_VERSION_CLONE    0x88
#define RC522_VERSION_FM17522  0x12

#define RC522_OK               0
#define RC522_NOTAG            1
#define RC522_ERR              2

void rc522_spi_init(void);
unsigned char rc522_init(void);
unsigned char rc522_read_register(unsigned char address);
void rc522_write_register(unsigned char address, unsigned char value);
unsigned char rc522_read_uid(unsigned char *uid);

#endif
