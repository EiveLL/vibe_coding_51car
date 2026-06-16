#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include <REG52.H>

/* Motor driver: Port 1 */
sbit PWMA = P1^0;
sbit IN1  = P1^1;
sbit IN2  = P1^2;
sbit IN3  = P1^3;
sbit IN4  = P1^4;
sbit PWMB = P1^5;

/*
 * Two digital line sensors: Port 0
 * P0 has no internal pull-up resistors. Add external pull-ups when the
 * sensor module does not provide push-pull high-level outputs.
 */
sbit TRACK_LEFT  = P0^0;
sbit TRACK_RIGHT = P0^1;

/*
 * RC522 software SPI: Port 2
 * The pin numbers in the supplied PDF are an MSP430 example. RC522 only
 * requires the signal relationship, so an 8051 software SPI can remap them.
 */
sbit RC522_NSS  = P2^0;
sbit RC522_SCK  = P2^1;
sbit RC522_MOSI = P2^2;
sbit RC522_MISO = P2^3;
sbit RC522_RST  = P2^4;

/* Station indication. P1.6, P2.5, P2.6 and P2.7 remain available. */
sbit BUZZER = P1^7;

/* Buzzer module is active low. */
#define BUZZER_ON_LEVEL   0
#define BUZZER_OFF_LEVEL  1

/* Two-bit result returned by track_read(). */
#define TRACK_LEFT_MASK   0x01
#define TRACK_RIGHT_MASK  0x02

/*
 * Current line-following configuration assumes:
 *   left=0, right=0: go straight
 *   left=1, right=1: stop / possible station marker
 */
#define TRACK_NORMAL_LEVEL  0
#define TRACK_MARK_LEVEL    1

void station_buzzer_start(unsigned char station_number);

#endif
