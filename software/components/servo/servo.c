/*
 * Copyright 2026 Rob Meades
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 * @brief 180 degree servo motor driver, using the LEDC peripheral.
 */

// Ensure we are compiling with maximum debug, can then be trimmed
// at run-time by fgr_log
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "sys/queue.h"

#include "fgr_util.h"

#include "servo.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Logging prefix
#define TAG "servo"

// All servos run at 50 Hz: this is the standard refresh rate for
// hobby servos, i.e. a 20 ms period.
#define SERVO_LEDC_FREQ_HZ 50

// Use 13-bit resolution (range 0-8191)
#define SERVO_LEDC_RESOLUTION LEDC_TIMER_13_BIT

// The period of the PWM signal in microseconds (1/50Hz = 20ms)
#define SERVO_PERIOD_US 20000

// The maximum duty value for 13-bit resolution
#define SERVO_DUTY_MAX 8191

// The number of LEDC timers we track.  LEDC_TIMER_MAX is the
// count defined by the driver; timers are numbered 0..(MAX-1).
#define SERVO_LEDC_TIMER_MAX LEDC_TIMER_MAX

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// Structure to keep track of all the elements of a servo driver,
// designed to be used as part of a linked list.  The public
// typedef in servo.h names this struct but does not define it,
// so the contents are private to this file.
struct motor_servo_t {
    int32_t pwm_pin;
    ledc_mode_t speed_mode;
    ledc_timer_t timer;
    ledc_channel_t channel;
    int32_t min_pulse_us;
    int32_t max_pulse_us;
    int32_t angle;
    SLIST_ENTRY(motor_servo_t) next;
};

// Servo list head.
SLIST_HEAD(motor_servo_list_t, motor_servo_t);

// Context.
typedef struct {
    SemaphoreHandle_t lock;
    struct motor_servo_list_t servo_list;
    // Whether each timer has been configured by us.  Indexed by
    // the timer number.  We only track configuration, not use, so
    // this is set on first use and never cleared until servo_deinit().
    bool timer_configured[SERVO_LEDC_TIMER_MAX];
} context_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// Context.
static context_t g_context = {0};

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Convert an angle in degrees to a LEDC duty value.
static uint32_t angle_to_duty(motor_servo_t *servo, int32_t angle)
{
    if (angle < 0) {
        angle = 0;
    }
    if (angle > 180) {
        angle = 180;
    }

    int32_t pulse_range = servo->max_pulse_us - servo->min_pulse_us;

    // pulse_us = min_pulse_us + (angle * pulse_range) / 180
    // Round to nearest by adding half the divisor before dividing
    int32_t pulse_us = servo->min_pulse_us + ((angle * pulse_range) + 90) / 180;

    // duty = (pulse_us * SERVO_DUTY_MAX) / SERVO_PERIOD_US
    // Round to nearest
    return (uint32_t) (((pulse_us * SERVO_DUTY_MAX) + (SERVO_PERIOD_US / 2)) /
                       SERVO_PERIOD_US);
}

// Clean up a servo.
// IMPORTANT: the context must be locked before this is called.
static void clean_up(motor_servo_t *servo)
{
    if (servo) {
        // Stop the PWM output and release the GPIO
        ledc_stop(servo->speed_mode, servo->channel, 0);
        gpio_reset_pin(servo->pwm_pin);

        // Remove the servo from the list, if present
        motor_servo_t *iter;
        motor_servo_t *prev = NULL;
        SLIST_FOREACH(iter, &g_context.servo_list, next) {
            if (iter == servo) {
                if (prev == NULL) {
                    // Removing the first element
                    SLIST_REMOVE_HEAD(&g_context.servo_list, next);
                } else {
                    // Removing a middle element
                    SLIST_REMOVE_AFTER(prev, next);
                }
                // Done; MUST break after an insertion or removal as
                // otherwise SLIST_FOREACH will go bang as it
                // relies on pointers still being valid.
                break;
            }
            prev = iter;
        }
        free(servo);
    }
}

// Common code to create a servo, given pulse width range.
static motor_servo_t *create_servo(int32_t pwm_pin,
                                   ledc_mode_t speed_mode,
                                   ledc_timer_t timer,
                                   ledc_channel_t channel,
                                   int32_t min_pulse_us,
                                   int32_t max_pulse_us)
{
    motor_servo_t *servo = NULL;
    esp_err_t err = ESP_OK;

    if ((timer < 0) || (timer >= SERVO_LEDC_TIMER_MAX) ||
        (channel < 0) || (channel >= LEDC_CHANNEL_MAX)) {
        err = ESP_ERR_INVALID_ARG;
    }

    if ((err == ESP_OK) &&
        ((min_pulse_us <= 0) || (max_pulse_us <= min_pulse_us) ||
         (max_pulse_us >= SERVO_PERIOD_US))) {
        err = ESP_ERR_INVALID_ARG;
    }

    if (err == ESP_OK) {
        servo = malloc(sizeof(*servo));
        if (servo == NULL) {
            err = ESP_ERR_NO_MEM;
        }
    }

    if (err == ESP_OK) {
        CONTEXT_LOCK(g_context.lock, "create_servo()");

        memset(servo, 0, sizeof(*servo));
        servo->pwm_pin = pwm_pin;
        servo->speed_mode = speed_mode;
        servo->timer = timer;
        servo->channel = channel;
        servo->min_pulse_us = min_pulse_us;
        servo->max_pulse_us = max_pulse_us;
        servo->angle = 0;

        // Configure the timer if we haven't already done so.
        // Note: the same timer number could in principle be
        // configured independently in high and low speed modes
        // on some chips, but on the C3 only low speed mode is
        // available, and treating the timer number as globally
        // unique is a reasonable simplification.
        if (!g_context.timer_configured[timer]) {
            ledc_timer_config_t cfg = {
                .speed_mode      = speed_mode,
                .timer_num       = timer,
                .duty_resolution = SERVO_LEDC_RESOLUTION,
                .freq_hz         = SERVO_LEDC_FREQ_HZ,
                .clk_cfg         = LEDC_AUTO_CLK,
            };
            err = ledc_timer_config(&cfg);
            if (err == ESP_OK) {
                g_context.timer_configured[timer] = true;
            }
        }

        // Configure the channel
        if (err == ESP_OK) {
            ledc_channel_config_t cfg = {
                .gpio_num   = pwm_pin,
                .speed_mode = speed_mode,
                .channel    = channel,
                .intr_type  = LEDC_INTR_DISABLE,
                .timer_sel  = timer,
                .duty       = angle_to_duty(servo, 0),
                .hpoint     = 0,
            };
            err = ledc_channel_config(&cfg);
        }

        if (err == ESP_OK) {
            SLIST_INSERT_HEAD(&g_context.servo_list, servo, next);
        } else {
            clean_up(servo);
            servo = NULL;
        }

        CONTEXT_UNLOCK(g_context.lock, "create_servo()");
    }

    return servo;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Initialise the servo module.
int32_t servo_init()
{
    int32_t err = ESP_OK;

    if (!g_context.lock) {
        // Create mutex
        err = -ESP_ERR_NO_MEM;
        g_context.lock = xSemaphoreCreateMutex();
        if (g_context.lock) {
            err = ESP_OK;
            SLIST_INIT(&g_context.servo_list);
            memset(g_context.timer_configured, 0,
                   sizeof(g_context.timer_configured));
        }
    }

    return err;
}

// Deinitialise the servo module.
void servo_deinit()
{
    CONTEXT_LOCK(g_context.lock, "servo_deinit()");

    while (!SLIST_EMPTY(&g_context.servo_list)) {
        motor_servo_t *p = SLIST_FIRST(&g_context.servo_list);
        clean_up(p);
    }

    // Pause any timers we configured.  We don't reset the flags,
    // so that a subsequent servo_init()/servo_create() sequence
    // would try to configure the timer again; use servo_deinit()
    // followed by servo_init() to fully reset.
    for (int32_t i = 0; i < SERVO_LEDC_TIMER_MAX; i++) {
        if (g_context.timer_configured[i]) {
            ledc_timer_pause(LEDC_LOW_SPEED_MODE, (ledc_timer_t) i);
            g_context.timer_configured[i] = false;
        }
    }

    CONTEXT_UNLOCK(g_context.lock, "servo_deinit()");
    // The semaphore will be re-used
}

// Create a servo motor with default pulse widths.
motor_servo_t *servo_create(int32_t pwm_pin,
                            ledc_mode_t speed_mode,
                            ledc_timer_t timer,
                            ledc_channel_t channel)
{
    return create_servo(pwm_pin, speed_mode, timer, channel,
                        SERVO_DEFAULT_MIN_PULSE_US,
                        SERVO_DEFAULT_MAX_PULSE_US);
}

// Create a servo motor with custom pulse widths.
motor_servo_t *servo_create_custom(int32_t pwm_pin,
                                   ledc_mode_t speed_mode,
                                   ledc_timer_t timer,
                                   ledc_channel_t channel,
                                   int32_t min_pulse_us,
                                   int32_t max_pulse_us)
{
    return create_servo(pwm_pin, speed_mode, timer, channel,
                        min_pulse_us, max_pulse_us);
}

// Free a servo motor.
void servo_destroy(motor_servo_t *servo)
{
    CONTEXT_LOCK(g_context.lock, "servo_destroy()");
    clean_up(servo);
    CONTEXT_UNLOCK(g_context.lock, "servo_destroy()");
}

// Set the angle of the servo.
int32_t servo_set_angle(motor_servo_t *servo, int32_t angle)
{
    int32_t err = ESP_ERR_INVALID_ARG;

    if (servo) {
        if (angle < 0) {
            angle = 0;
        }
        if (angle > 180) {
            angle = 180;
        }

        err = ledc_set_duty(servo->speed_mode, servo->channel,
                            angle_to_duty(servo, angle));
        if (err == ESP_OK) {
            err = ledc_update_duty(servo->speed_mode, servo->channel);
        }
        if (err == ESP_OK) {
            servo->angle = angle;
        }
    }

    // Return negative value from esp_err_t.
    return -err;
}

// Get the last angle set on the servo.
int32_t servo_get_angle(motor_servo_t *servo)
{
    int32_t angle = -1;

    if (servo) {
        angle = servo->angle;
    }

    return angle;
}

// End of file