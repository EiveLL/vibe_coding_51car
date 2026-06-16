#include "hardware.h"
#include "rc522.h"

/*
 * Dual motor driver test for a generic 8052-compatible board.
 *
 * Pin assignments are collected in hardware.h.
 */

volatile unsigned char pwm_a = 0;
volatile unsigned char pwm_b = 0;
volatile unsigned char pwm_count = 0;
volatile unsigned char tick_100us = 0;
volatile bit flag_10ms = 0;

unsigned char track_value = 0x00;
unsigned char track_last_sample = 0x00;
unsigned char track_stable_count = 0;
unsigned char last_drive_action = 0;
unsigned int startup_ignore_stop_ticks = 0;
unsigned char buzzer_beeps_left = 0;
unsigned char buzzer_phase_ticks = 0;
unsigned int buzzer_hold_ticks = 0;
bit buzzer_is_on = 0;
bit station_line_detected = 0;
bit rc522_ready = 0;
unsigned char rc522_version = 0;

#define BUZZER_ON_TICKS   15
#define BUZZER_OFF_TICKS  15

#define TRACK_DEBOUNCE_COUNT  3
#define SPEED_BASE            24
#define SPEED_TURN_FORWARD    40
#define SPEED_TURN_REVERSE    16

#define DRIVE_STRAIGHT        0
#define DRIVE_LEFT            1
#define DRIVE_RIGHT           2
#define STARTUP_IGNORE_TICKS  500

/*
 * Timer 0 interrupts every 100 us with a classic 12 MHz, 12-clock 8051.
 * A 100-step cycle therefore produces PWM at approximately 100 Hz.
 */
void timer0_isr(void) interrupt 1
{
    ++pwm_count;
    if (pwm_count >= 100) {
        pwm_count = 0;
    }

    PWMA = (pwm_count < pwm_a) ? 1 : 0;
    PWMB = (pwm_count < pwm_b) ? 1 : 0;

    ++tick_100us;
    if (tick_100us >= 100) {
        tick_100us = 0;
        flag_10ms = 1;
    }
}

static void timer0_init(void)
{
    /* Timer 0 mode 2: 8-bit automatic reload, 100 us at 12 MHz. */
    TMOD = (TMOD & 0xF0) | 0x02;
    TH0 = 0x9C;
    TL0 = 0x9C;
    ET0 = 1;
    EA = 1;
    TR0 = 1;
}

static void gpio_init(void)
{
    /* Release Port 0 pins so external line sensor outputs can drive them. */
    P0 = 0xFF;

    RC522_NSS = 1;
    RC522_SCK = 0;
    RC522_MOSI = 0;
    RC522_MISO = 1;
    RC522_RST = 1;

    BUZZER = BUZZER_OFF_LEVEL;
}

static void motor_init(void)
{
    pwm_a = 0;
    pwm_b = 0;
    pwm_count = 0;
    PWMA = 0;
    PWMB = 0;
    IN1 = 0;
    IN2 = 0;
    IN3 = 0;
    IN4 = 0;
}

/*
 * Returns P0.1..P0.0 as RIGHT, LEFT.
 * Whether black is 0 or 1 depends on the line sensor module.
 */
static unsigned char track_read(void)
{
    return P0 & (TRACK_LEFT_MASK | TRACK_RIGHT_MASK);
}

/*
 * Announces a station number as repeated short beeps:
 * station 1 = one beep, station 2 = two beeps, and so on.
 * This rhythm method works with common active buzzer modules.
 * station_number 0 is reserved for one 0.5 s initialization beep.
 */
void station_buzzer_start(unsigned char station_number)
{
    if (station_number == 0) {
        buzzer_beeps_left = 0;
        buzzer_phase_ticks = 0;
        buzzer_hold_ticks = 50;
        buzzer_is_on = 1;
        BUZZER = BUZZER_ON_LEVEL;
        return;
    }

    buzzer_beeps_left = station_number;
    buzzer_phase_ticks = BUZZER_ON_TICKS;
    buzzer_is_on = 1;
    BUZZER = BUZZER_ON_LEVEL;
}

static void buzzer_task_10ms(void)
{
    if (buzzer_hold_ticks > 0) {
        --buzzer_hold_ticks;
        if (buzzer_hold_ticks == 0) {
            BUZZER = BUZZER_OFF_LEVEL;
            buzzer_is_on = 0;
        }
        return;
    }

    if (buzzer_phase_ticks == 0) {
        return;
    }

    --buzzer_phase_ticks;
    if (buzzer_phase_ticks == 0) {
        if (buzzer_is_on) {
            BUZZER = BUZZER_OFF_LEVEL;
            buzzer_is_on = 0;
            --buzzer_beeps_left;

            if (buzzer_beeps_left > 0) {
                buzzer_phase_ticks = BUZZER_OFF_TICKS;
            }
        } else {
            BUZZER = BUZZER_ON_LEVEL;
            buzzer_is_on = 1;
            buzzer_phase_ticks = BUZZER_ON_TICKS;
        }
    }
}

static void motor_stop(void)
{
    pwm_a = 0;
    pwm_b = 0;
    IN1 = 0;
    IN2 = 0;
    IN3 = 0;
    IN4 = 0;
}

static void system_init(void)
{
    gpio_init();
    motor_init();

    rc522_version = rc522_init();
    if ((rc522_version == RC522_VERSION_V1) ||
        (rc522_version == RC522_VERSION_V2) ||
        (rc522_version == RC522_VERSION_CLONE)) {
        rc522_ready = 1;
    } else {
        rc522_ready = 0;
    }

    timer0_init();
    startup_ignore_stop_ticks = STARTUP_IGNORE_TICKS;
    station_buzzer_start(0);
}

static void motor_forward(unsigned char speed_a, unsigned char speed_b)
{
    IN1 = 1;
    IN2 = 0;
    IN3 = 1;
    IN4 = 0;
    pwm_a = speed_a;
    pwm_b = speed_b;
}

static void motor_turn_left(unsigned char forward_speed, unsigned char reverse_speed)
{
    IN1 = 0;
    IN2 = 1;
    IN3 = 1;
    IN4 = 0;
    pwm_a = reverse_speed;
    pwm_b = forward_speed;
}

static void motor_turn_right(unsigned char forward_speed, unsigned char reverse_speed)
{
    IN1 = 1;
    IN2 = 0;
    IN3 = 0;
    IN4 = 1;
    pwm_a = forward_speed;
    pwm_b = reverse_speed;
}

/*
 * Called every 10 ms.
 *
 * Assumptions:
 *   - Sensor output 0 means normal line-following state.
 *   - Sensor output 1 means the corresponding side needs correction.
 *   - Motor A is the left motor and motor B is the right motor.
 */
static void line_follow_10ms(void)
{
    unsigned char sample;

    sample = track_read();

    if (sample != track_last_sample) {
        track_last_sample = sample;
        track_stable_count = 1;
        return;
    }

    if (track_stable_count < TRACK_DEBOUNCE_COUNT) {
        ++track_stable_count;
        if (track_stable_count < TRACK_DEBOUNCE_COUNT) {
            return;
        }
    }

    track_value = sample;

    switch (track_value) {
    case 0x00:
        /* Both sensors are 0: go straight. */
        station_line_detected = 0;
        last_drive_action = DRIVE_STRAIGHT;
        motor_forward(SPEED_BASE, SPEED_BASE);
        break;

    case 0x02:
        /* Right sensor is 1: steer left. */
        station_line_detected = 0;
        last_drive_action = DRIVE_LEFT;
        motor_turn_left(SPEED_TURN_FORWARD, SPEED_TURN_REVERSE);
        break;

    case 0x01:
        /* Left sensor is 1: steer right. */
        station_line_detected = 0;
        last_drive_action = DRIVE_RIGHT;
        motor_turn_right(SPEED_TURN_FORWARD, SPEED_TURN_REVERSE);
        break;

    case 0x03:
    default:
        /* Both sensors are 1: stop and let RC522 confirm a station. */
        if (startup_ignore_stop_ticks > 0) {
            station_line_detected = 0;
            last_drive_action = DRIVE_STRAIGHT;
            motor_forward(SPEED_BASE, SPEED_BASE);
            break;
        }

        station_line_detected = 1;
        motor_stop();
        break;
    }
}

void main(void)
{
    system_init();

    while (1) {
        if (flag_10ms) {
            flag_10ms = 0;
            buzzer_task_10ms();
            if (startup_ignore_stop_ticks > 0) {
                --startup_ignore_stop_ticks;
            }
            line_follow_10ms();
        }
    }
}
