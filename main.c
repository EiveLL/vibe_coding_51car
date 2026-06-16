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
unsigned int startup_force_straight_ticks = 0;
unsigned int startup_ignore_stop_ticks = 0;
unsigned char buzzer_beeps_left = 0;
unsigned char buzzer_phase_ticks = 0;
unsigned int buzzer_hold_ticks = 0;
bit buzzer_is_on = 0;
bit station_line_detected = 0;
bit rc522_ready = 0;
unsigned char rc522_version = 0;
unsigned char car_state = 0;
unsigned char station_current = 0;
unsigned char station_lock = 0;
unsigned int station_stop_ticks = 0;
bit stop_prompt_played = 0;
unsigned char rc522_poll_ticks = 0;
unsigned int station_leave_ticks = 0;
unsigned char motor_boost_ticks = 0;
unsigned char motor_boost_pwm = 0;
bit motor_was_stopped = 1;

#define STATION_COUNT  4

unsigned char code station_uid[STATION_COUNT][4] = {
    {0x83, 0xB1, 0x00, 0x07},
    {0xD3, 0x97, 0xBA, 0xE2},
    {0x2C, 0xC0, 0x31, 0x6F},
    {0x83, 0xF0, 0xA1, 0x05}
};

#define BUZZER_ON_TICKS   15
#define BUZZER_OFF_TICKS  15

#define TRACK_DEBOUNCE_COUNT  3
#define SPEED_BASE            22
#define SPEED_TURN_FORWARD    40
#define SPEED_TURN_REVERSE    16
#define SPEED_START_BOOST     45
#define SPEED_LEAVE_BOOST     34
#define START_BOOST_TICKS     20
#define LEAVE_BOOST_TICKS     8

#define DRIVE_STRAIGHT        0
#define DRIVE_LEFT            1
#define DRIVE_RIGHT           2
#define STARTUP_STRAIGHT_TICKS 50
#define STARTUP_IGNORE_TICKS  500

#define CAR_RUNNING           0
#define CAR_STOPPED_AT_STATION 1
#define STATION_STOP_TICKS    200
#define RC522_POLL_TICKS      10
#define STATION_LEAVE_TICKS   300

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

static unsigned char find_station(unsigned char *uid)
{
    unsigned char station;
    unsigned char i;

    for (station = 0; station < STATION_COUNT; ++station) {
        for (i = 0; i < 4; ++i) {
            if (uid[i] != station_uid[station][i]) {
                break;
            }
        }

        if (i == 4) {
            return station + 1;
        }
    }

    return 0;
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

static void buzzer_hold_start(unsigned int duration_10ms)
{
    buzzer_beeps_left = 0;
    buzzer_phase_ticks = 0;
    buzzer_hold_ticks = duration_10ms;
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
    motor_boost_ticks = 0;
    motor_boost_pwm = 0;
    motor_was_stopped = 1;
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
        (rc522_version == RC522_VERSION_CLONE) ||
        (rc522_version == RC522_VERSION_FM17522)) {
        rc522_ready = 1;
    } else {
        rc522_ready = 0;
    }

    timer0_init();
    startup_force_straight_ticks = STARTUP_STRAIGHT_TICKS;
    startup_ignore_stop_ticks = STARTUP_IGNORE_TICKS;
    station_buzzer_start(0);
}

static void motor_forward(unsigned char speed_a, unsigned char speed_b)
{
    if (motor_was_stopped) {
        if (motor_boost_ticks == 0) {
            motor_boost_ticks = START_BOOST_TICKS;
            motor_boost_pwm = SPEED_START_BOOST;
        }
        motor_was_stopped = 0;
    }

    IN1 = 1;
    IN2 = 0;
    IN3 = 1;
    IN4 = 0;
    if (motor_boost_ticks > 0) {
        pwm_a = motor_boost_pwm;
        pwm_b = motor_boost_pwm;
    } else {
        pwm_a = speed_a;
        pwm_b = speed_b;
    }
}

static void motor_turn_left(unsigned char forward_speed, unsigned char reverse_speed)
{
    if (motor_was_stopped) {
        if (motor_boost_ticks == 0) {
            motor_boost_ticks = START_BOOST_TICKS;
            motor_boost_pwm = SPEED_START_BOOST;
        }
        motor_was_stopped = 0;
    }

    IN1 = 0;
    IN2 = 1;
    IN3 = 1;
    IN4 = 0;
    if (motor_boost_ticks > 0) {
        pwm_a = motor_boost_pwm;
        pwm_b = motor_boost_pwm;
    } else {
        pwm_a = reverse_speed;
        pwm_b = forward_speed;
    }
}

static void motor_turn_right(unsigned char forward_speed, unsigned char reverse_speed)
{
    if (motor_was_stopped) {
        if (motor_boost_ticks == 0) {
            motor_boost_ticks = START_BOOST_TICKS;
            motor_boost_pwm = SPEED_START_BOOST;
        }
        motor_was_stopped = 0;
    }

    IN1 = 1;
    IN2 = 0;
    IN3 = 0;
    IN4 = 1;
    if (motor_boost_ticks > 0) {
        pwm_a = motor_boost_pwm;
        pwm_b = motor_boost_pwm;
    } else {
        pwm_a = forward_speed;
        pwm_b = reverse_speed;
    }
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
        stop_prompt_played = 0;
        last_drive_action = DRIVE_STRAIGHT;
        motor_forward(SPEED_BASE, SPEED_BASE);
        break;

    case 0x02:
        /* Right sensor is 1: steer left. */
        station_line_detected = 0;
        stop_prompt_played = 0;
        last_drive_action = DRIVE_LEFT;
        motor_turn_left(SPEED_TURN_FORWARD, SPEED_TURN_REVERSE);
        break;

    case 0x01:
        /* Left sensor is 1: steer right. */
        station_line_detected = 0;
        stop_prompt_played = 0;
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

static void stop_at_station(unsigned char station_number)
{
    motor_stop();
    car_state = CAR_STOPPED_AT_STATION;
    station_current = station_number;
    station_stop_ticks = STATION_STOP_TICKS;
    stop_prompt_played = 1;
    station_buzzer_start(station_number);
}

static unsigned char rc522_get_station(void)
{
    unsigned char uid[4];
    unsigned char station_number;

    if (!rc522_ready) {
        return 0;
    }

    if (rc522_read_uid(uid) != RC522_OK) {
        station_lock = 0;
        return 0;
    }

    station_number = find_station(uid);
    if ((station_number != 0) && (station_number != station_lock)) {
        station_lock = station_number;
        return station_number;
    }

    return 0;
}

static void rc522_station_task(void)
{
    unsigned char station_number;

    ++rc522_poll_ticks;
    if (rc522_poll_ticks < RC522_POLL_TICKS) {
        return;
    }
    rc522_poll_ticks = 0;

    station_number = rc522_get_station();
    if (station_number != 0) {
        stop_at_station(station_number);
    }
}

static void handle_stop_condition(void)
{
    unsigned char station_number;

    motor_stop();
    station_line_detected = 1;

    station_number = rc522_get_station();
    if (station_number != 0) {
        stop_at_station(station_number);
        return;
    }

    if (!stop_prompt_played) {
        buzzer_hold_start(100);
        stop_prompt_played = 1;
    }
}

void main(void)
{
    system_init();

    while (1) {
        if (flag_10ms) {
            flag_10ms = 0;
            buzzer_task_10ms();

            if (car_state == CAR_STOPPED_AT_STATION) {
                motor_stop();
                if (station_stop_ticks > 0) {
                    --station_stop_ticks;
                }
                if (station_stop_ticks == 0) {
                    car_state = CAR_RUNNING;
                    station_current = 0;
                    station_leave_ticks = STATION_LEAVE_TICKS;
                    startup_ignore_stop_ticks = STARTUP_IGNORE_TICKS;
                    startup_force_straight_ticks = 0;
                    motor_boost_ticks = LEAVE_BOOST_TICKS;
                    motor_boost_pwm = SPEED_LEAVE_BOOST;
                }
                continue;
            }

            if (startup_force_straight_ticks > 0) {
                --startup_force_straight_ticks;
                if (motor_boost_ticks > 0) {
                    --motor_boost_ticks;
                }
                if (startup_ignore_stop_ticks > 0) {
                    --startup_ignore_stop_ticks;
                }
                if (station_leave_ticks > 0) {
                    --station_leave_ticks;
                    if (station_leave_ticks == 0) {
                        station_lock = 0;
                    }
                }
                station_line_detected = 0;
                last_drive_action = DRIVE_STRAIGHT;
                motor_forward(SPEED_BASE, SPEED_BASE);
                continue;
            }

            if (startup_ignore_stop_ticks > 0) {
                --startup_ignore_stop_ticks;
            }
            if (motor_boost_ticks > 0) {
                --motor_boost_ticks;
            }
            if (station_leave_ticks > 0) {
                --station_leave_ticks;
                if (station_leave_ticks == 0) {
                    station_lock = 0;
                }
            }
            line_follow_10ms();
            if (station_line_detected) {
                handle_stop_condition();
            } else {
                rc522_station_task();
            }
        }
    }
}
