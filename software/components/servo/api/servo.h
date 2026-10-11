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

#ifndef _SERVO_H_
#define _SERVO_H_

/** @file
 * @brief Servo API.
 */

#ifdef __cplusplus
extern "C" {
#endif

/** @file
 * @brief 180 degree servo motor API.
 */

#ifdef __cplusplus
extern "C" {
#endif

// Needed for ledc_mode_t, ledc_timer_t and ledc_channel_t
#include "driver/ledc.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/** Default minimum pulse width in microseconds, corresponding to
 * 0 degrees.
 */
#define SERVO_DEFAULT_MIN_PULSE_US 500

/** Default maximum pulse width in microseconds, corresponding to
 * 180 degrees.
 */
#define SERVO_DEFAULT_MAX_PULSE_US 2500

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/** A servo motor.  The contents of this structure are private to
 * servo.c; only pointers to it are ever passed around.
 */
typedef struct motor_servo_t motor_servo_t;

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

/** Initialise the servo module.  This function may be called at
 * any time; if it has already been called it will do nothing and
 * return success.  This creates a semaphore that will never be
 * destroyed.
 *
 * @return ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t servo_init();

/** Deinitialise the servo module.  When called all servos will be
 * released by calling servo_destroy() and any LEDC timers that
 * were configured by this module will be paused.
 */
void servo_deinit();

/** Create a servo motor with default pulse widths (500us to 2500us
 * for 0 to 180 degrees), which is the most common configuration
 * for a standard hobby servo.
 *
 * @param pwm_pin    the GPIO output pin to which the PWM signal
 *                   should be supplied.
 * @param speed_mode the LEDC speed mode to use, e.g.
 *                   LEDC_LOW_SPEED_MODE.
 * @param timer      the LEDC timer to use.  The timer will be
 *                   configured for 50Hz operation at 16-bit
 *                   resolution by this component; do not also
 *                   configure it yourself.
 * @param channel    the LEDC channel to use.  Must be unique
 *                   across all servos and all other users of the
 *                   same timer.
 * @return           a handle for the servo, else NULL.
 */
motor_servo_t *servo_create(int32_t pwm_pin,
                            ledc_mode_t speed_mode,
                            ledc_timer_t timer,
                            ledc_channel_t channel);

/** Create a servo motor with custom pulse widths.  Use this if
 * your servo does not conform to the standard 500us to 2500us
 * range; check the datasheet for your servo.
 *
 * @param pwm_pin      the GPIO output pin to which the PWM signal
 *                     should be supplied.
 * @param speed_mode   the LEDC speed mode to use.
 * @param timer        the LEDC timer to use, as for servo_create().
 * @param channel      the LEDC channel to use, as for servo_create().
 * @param min_pulse_us the pulse width in microseconds that
 *                     corresponds to 0 degrees.
 * @param max_pulse_us the pulse width in microseconds that
 *                     corresponds to 180 degrees.
 * @return             a handle for the servo, else NULL.
 */
motor_servo_t *servo_create_custom(int32_t pwm_pin,
                                   ledc_mode_t speed_mode,
                                   ledc_timer_t timer,
                                   ledc_channel_t channel,
                                   int32_t min_pulse_us,
                                   int32_t max_pulse_us);

/** Delete a servo motor.  It is always safe to call this function.
 * The PWM output on the servo's channel is stopped and all
 * resources held by this servo are released.  The underlying LEDC
 * timer is left configured, since other servos or the application
 * may still be using it; call servo_deinit() to release timers.
 *
 * @param servo the handle that was returned by servo_create().
 */
void servo_destroy(motor_servo_t *servo);

/** Set the angle of the servo.
 *
 * @param servo the handle that was returned by servo_create().
 * @param angle the desired angle in degrees, 0 to 180 inclusive.
 *              Values outside this range will be clamped.
 * @return      ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t servo_set_angle(motor_servo_t *servo, int32_t angle);

/** Get the last angle set on the servo.
 *
 * @param servo the handle that was returned by servo_create().
 * @return      the angle in degrees, or a negative value from
 *              esp_err_t if the servo handle is invalid.
 */
int32_t servo_get_angle(motor_servo_t *servo);

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _SERVO_H_

// End of file
