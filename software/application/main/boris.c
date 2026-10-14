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
 * @brief Implementation of Boris, the Halloween skeleton.  Note that
 * this application uses several of the front garden railway (FGR)
 * components (cut down and modified from the originals), for instance
 * for the WS2812 [eyes] LEDs, task handling and a few general utilities.
 */

#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "errno.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_task_wdt.h"
#include "esp_random.h"

#include "fgr_util.h"
#include "fgr_debug.h"
#include "fgr_task.h"
#include "fgr_nvs.h"
#include "servo.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

 // Logging prefix.
 #define TAG "boris"

// Mask for the cigar sign table.
#define CIGAR_SINE_MASK (FGR_UTIL_ARRAY_LENGTH(g_cigar_sine) - 1)

// Cigar puff duration, in seconds.
#define CIGAR_PUFF_PERIOD_SECONDS 5

// Cigar puff update frequency, in milliseconds.
#define CIGAR_PUFF_UPDATE_MS 20

// Minimum (e.g. ember) glow for the cigar (out of 0 - 255).
#define CIGAR_MIN_DUTY_LEVEL 64

// Peak-puff for the cigar.
#define CIGAR_MAX_DUTY_LEVEL 255

// Total period of one heartbeat cycle in milliseconds
// (i.e. one "lub-dup" sequence plus the rest before the next).
// 800 ms = 75 bpm is a resting adult heart rate, but
// Boris is a bit more relaxed than that.
#define HEARTBEAT_PERIOD_MS 1600

// Duration of the "lub" (first) pulse in milliseconds.
#define HEARTBEAT_LUB_MS 160

// Duration of the "dup" (second) pulse in milliseconds.
#define HEARTBEAT_DUP_MS 120

// Gap between the end of "lub" and the start of "dup".
#define HEARTBEAT_GAP_MS 180

// Servo angle at rest, in degrees.
#define HEARTBEAT_REST_ANGLE 0

// Servo angle at the peak of each pulse, in degrees.
#define HEARTBEAT_PEAK_ANGLE 90

// The maximum eye-flash interval in seconds
#define EYES_INTERVAL_MAX_SECONDS 15

// The minumum eye-flash interval in seconds
#define EYES_INTERVAL_MIN_SECONDS 5

// Eye flash duration in milliseconds
#define EYES_FLASH_DURATION_MS 500

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// The phases of the heartbeat state machine.  Each phase has a
// duration in ticks and a target angle; the servo is set to the
// target angle at the start of the phase and held there until the
// phase ends.
typedef enum {
    HEARTBEAT_PHASE_REST,   // waiting between beats
    HEARTBEAT_PHASE_LUB,    // first pulse
    HEARTBEAT_PHASE_GAP,    // gap between lub and dup
    HEARTBEAT_PHASE_DUP,    // second pulse
} heartbeat_phase_t;

// Context for the cigar.
typedef struct {
    TaskHandle_t task_handle;
    int64_t last_puff_update_us;
    uint32_t phase;
} cigar_t;

// Context for the eyes.
typedef struct {
    void *ws2812_handle;
} eyes_t;

// Context for the heartbeat.
typedef struct {
    heartbeat_phase_t phase;
    int64_t phase_started_us;
    TaskHandle_t task_handle;
    motor_servo_t *servo;
} heartbeat_t;

// Top-level context.
typedef struct {
    SemaphoreHandle_t lock;
    cigar_t cigar;
    eyes_t eyes;
    heartbeat_t heartbeat;
} context_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// Context.
static context_t g_context = {0};

#if defined(CONFIG_BORIS_CIGAR_LED_PIN) && (CONFIG_BORIS_CIGAR_LED_PIN >= 0)
// Sine table for cigar puffs.
// sin(2*pi*i/256) mapped to 0..255 (Q8 fixed point, 128 = midpoint)
static const uint8_t g_cigar_sine[] = {
    128, 131, 134, 137, 140, 143, 147, 150,
    153, 156, 159, 162, 165, 168, 171, 174,
    177, 180, 182, 185, 188, 191, 194, 196,
    199, 201, 204, 206, 209, 211, 213, 216,
    218, 220, 222, 224, 226, 228, 230, 232,
    234, 235, 237, 238, 240, 241, 242, 243,
    244, 245, 246, 247, 248, 248, 249, 250,
    250, 251, 251, 251, 252, 252, 252, 252,
    252, 252, 252, 252, 252, 251, 251, 251,
    250, 250, 249, 248, 248, 247, 246, 245,
    244, 243, 242, 241, 240, 238, 237, 235,
    234, 232, 230, 228, 226, 224, 222, 220,
    218, 216, 213, 211, 209, 206, 204, 201,
    199, 196, 194, 191, 188, 185, 182, 180,
    177, 174, 171, 168, 165, 162, 159, 156,
    153, 150, 147, 143, 140, 137, 134, 131,
    128, 124, 121, 118, 115, 112, 108, 105,
    102, 99,  96,  93,  90,  87,  84,  81,
    78,  75,  73,  70,  67,  64,  61,  59,
    56,  54,  51,  49,  46,  44,  42,  39,
    37,  35,  33,  31,  29,  27,  25,  23,
    21,  20,  18,  17,  15,  14,  13,  12,
    11,  10,  9,   8,   7,   7,   6,   5,
    5,   4,   4,   4,   3,   3,   3,   3,
    3,   3,   3,   3,   3,   4,   4,   4,
    5,   5,   6,   7,   7,   8,   9,   10,
    11,  12,  13,  14,  15,  17,  18,  20,
    21,  23,  25,  27,  29,  31,  33,  35,
    37,  39,  42,  44,  46,  49,  51,  54,
    56,  59,  61,  64,  67,  70,  73,  75,
    78,  81,  84,  87,  90,  93,  96,  99,
    102, 105, 108, 112, 115, 118, 121, 124
};
#endif

#if defined(CONFIG_BORIS_EYES_WS2812_LED_PIN) && (CONFIG_BORIS_EYES_WS2812_LED_PIN >= 0)
// All the possible eye colours.
static fgr_ws2812_colour_t g_eye_colour[] = {FGR_WS2812_LED_COLOUR_BRIGHT_RED,
                                             FGR_WS2812_LED_COLOUR_BRIGHT_BLUE,
                                             FGR_WS2812_LED_COLOUR_BRIGHT_GREEN,
                                             FGR_WS2812_LED_COLOUR_BRIGHT_YELLOW,
                                             FGR_WS2812_LED_COLOUR_BRIGHT_MAGENTA,
                                             FGR_WS2812_LED_COLOUR_BRIGHT_CYAN};
// The names of the eye colours.
static char * g_eye_colour_name[] = {"red","blue", "green", "yellow", "magenta", "cyan"};

// Do some checking
_Static_assert (FGR_UTIL_ARRAY_LENGTH(g_eye_colour) == FGR_UTIL_ARRAY_LENGTH(g_eye_colour_name),
                "the number of g_eye_colour_name[] entries does not match the number of eye colours!");

#endif

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: CIGAR RELATED
 * -------------------------------------------------------------- */

#if defined(CONFIG_BORIS_CIGAR_LED_PIN) && (CONFIG_BORIS_CIGAR_LED_PIN >= 0)
// Cigar task callback
static void task_cigar_cb(void *handle, void *param)
{
    cigar_t *cigar = (cigar_t *) param;

    (void) handle;

    // Top 8 bits index the table, next 8 bits are the interpolation fraction
    uint32_t idx  = (cigar->phase >> 24) & CIGAR_SINE_MASK;
    uint32_t fraction = (cigar->phase >> 16) & 0xFF;

    uint32_t v0 = g_cigar_sine[idx];
    uint32_t v1 = g_cigar_sine[(idx + 1) & CIGAR_SINE_MASK];

    // Linear interpolation in Q8: v = v0 + fraction * (v1 - v0) / 256
    int32_t delta = (int32_t) v1 - (int32_t) v0;
    uint32_t v = v0 + (uint32_t) ((delta * (int32_t) fraction) >> 8);

    // Scale Q8 (0..255) to duty range
    uint32_t duty = CIGAR_MIN_DUTY_LEVEL + ((v * (CIGAR_MAX_DUTY_LEVEL - CIGAR_MIN_DUTY_LEVEL)) >> 8);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, CONFIG_BORIS_CIGAR_LED_PWM_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CONFIG_BORIS_CIGAR_LED_PWM_CHANNEL);

    if (esp_timer_get_time() - cigar->last_puff_update_us > CIGAR_PUFF_UPDATE_MS * 1000) {
        // One full cycle = 2^32. Increment per tick:
        cigar->phase += ((((uint64_t) 1) << 32) * CIGAR_PUFF_UPDATE_MS / (CIGAR_PUFF_PERIOD_SECONDS * 1000));
        cigar->last_puff_update_us = esp_timer_get_time();
    }
}

// Initialise the cigar.
static int32_t cigar_init(cigar_t *cigar)
{
    // Configure the cigar LED, PWM driven
    ledc_timer_config_t cfg_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = CONFIG_BORIS_CIGAR_LED_PWM_TIMER,
        .freq_hz = 4000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    int32_t err = -ledc_timer_config(&cfg_timer);
    if (err == ESP_OK) {
        ledc_channel_config_t cfg_channel = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = CONFIG_BORIS_CIGAR_LED_PWM_CHANNEL,
            .timer_sel      = CONFIG_BORIS_CIGAR_LED_PWM_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = CONFIG_BORIS_CIGAR_LED_PIN,
            .duty           = 0,
            .hpoint         = 0
        };
        err = -ledc_channel_config(&cfg_channel);
    }

    if (err == ESP_OK) {
        fgr_task_create(&task_cigar_cb, cigar, "cigar",
                        3000,
                        3, &cigar->task_handle);
    }

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "PWM cigar on pin %d puffing every %d second(s).",
                 CONFIG_BORIS_CIGAR_LED_PIN, CIGAR_PUFF_PERIOD_SECONDS);
    }

    return err;
}

// Deinitialise the cigar.
static void cigar_deinit(cigar_t *cigar)
{
    if (cigar->task_handle) {
        fgr_task_destroy(cigar->task_handle);
    }
    ledc_stop(LEDC_LOW_SPEED_MODE, CONFIG_BORIS_CIGAR_LED_PWM_CHANNEL, 0);
    ledc_timer_pause(LEDC_LOW_SPEED_MODE, CONFIG_BORIS_CIGAR_LED_PWM_TIMER);
    ledc_timer_config_t cfg_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num  = CONFIG_BORIS_CIGAR_LED_PWM_TIMER,
        .deconfigure = true
    };
    ledc_timer_config(&cfg_timer);
    gpio_reset_pin(CONFIG_BORIS_CIGAR_LED_PIN);
}
#endif

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: EYES RELATED
 * -------------------------------------------------------------- */

#if defined(CONFIG_BORIS_EYES_WS2812_LED_PIN) && (CONFIG_BORIS_EYES_WS2812_LED_PIN >= 0)
// Initialise the eyes.
static int32_t eyes_init(eyes_t *eyes)
{
    int32_t err = fgr_ws2812_chain_init(CONFIG_BORIS_EYES_WS2812_SPI_NUM, -1,
                                        CONFIG_BORIS_EYES_WS2812_LED_PIN, 1,
                                        CONFIG_BORIS_EYES_WS2812_GRB,
                                        &eyes->ws2812_handle);
    if (err == ESP_OK) {
        // Flash both eyes white so that we know they are on
        err = fgr_ws2812_led_flash(eyes->ws2812_handle, -1,
                                   2000, FGR_WS2812_LED_COLOUR_WHITE);
    }

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WS2812 eyes initialised on pin %d.", CONFIG_BORIS_EYES_WS2812_LED_PIN);
    }

    return err;
}

// Deinitialise the eyes.
static void eyes_deinit(eyes_t *eyes)
{
    if (eyes->ws2812_handle) {
        fgr_ws2812_chain_deinit(eyes->ws2812_handle);
    }
}

#endif

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: HEARTBEAT RELATED
 * -------------------------------------------------------------- */

#if defined(CONFIG_BORIS_HEARTBEAT_SERVO_PIN) && (CONFIG_BORIS_HEARTBEAT_SERVO_PIN >= 0)

// Move to the given phase and command the servo accordingly.
static void heartbeat_enter_phase(heartbeat_t *heartbeat,
                                  heartbeat_phase_t phase,
                                  int64_t now_us)
{
    heartbeat->phase = phase;
    heartbeat->phase_started_us = now_us;

    switch (phase) {
        case HEARTBEAT_PHASE_LUB:
        case HEARTBEAT_PHASE_DUP:
            servo_set_angle(heartbeat->servo, HEARTBEAT_PEAK_ANGLE);
        break;
        case HEARTBEAT_PHASE_REST:
        case HEARTBEAT_PHASE_GAP:
        default:
            servo_set_angle(heartbeat->servo, HEARTBEAT_REST_ANGLE);
        break;
    }
}

// Heartbeat task callback
static void task_heartbeat_cb(void *handle, void *param)
{
    heartbeat_t *heartbeat = (heartbeat_t *) param;

    (void) handle;

    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - heartbeat->phase_started_us;
    int64_t phase_length_us;

    switch (heartbeat->phase) {
        case HEARTBEAT_PHASE_LUB:
            phase_length_us = HEARTBEAT_LUB_MS * 1000;
        break;
        case HEARTBEAT_PHASE_GAP:
            phase_length_us = HEARTBEAT_GAP_MS * 1000;
        break;
        case HEARTBEAT_PHASE_DUP:
            phase_length_us = HEARTBEAT_DUP_MS * 1000;
        break;
        case HEARTBEAT_PHASE_REST:
        default:
            phase_length_us = (HEARTBEAT_PERIOD_MS * 1000) -
                              (HEARTBEAT_LUB_MS * 1000) -
                              (HEARTBEAT_GAP_MS * 1000) -
                              (HEARTBEAT_DUP_MS * 1000);
        break;
    }

    if (elapsed_us > phase_length_us) {
        switch (heartbeat->phase) {
            case HEARTBEAT_PHASE_REST:
                heartbeat_enter_phase(heartbeat, HEARTBEAT_PHASE_LUB, now_us);
            break;
            case HEARTBEAT_PHASE_LUB:
                heartbeat_enter_phase(heartbeat, HEARTBEAT_PHASE_GAP, now_us);
            break;
            case HEARTBEAT_PHASE_GAP:
                heartbeat_enter_phase(heartbeat, HEARTBEAT_PHASE_DUP, now_us);
            break;
            case HEARTBEAT_PHASE_DUP:
            default:
                heartbeat_enter_phase(heartbeat, HEARTBEAT_PHASE_REST, now_us);
            break;
        }
    }
}

// Initialise the heartbeat.
static int32_t heartbeat_init(heartbeat_t *heartbeat)
{
    int32_t err = ESP_FAIL; // Note: this is already a NEGATIVE error code (-1)
    heartbeat->servo = servo_create(CONFIG_BORIS_HEARTBEAT_SERVO_PIN,
                                    LEDC_LOW_SPEED_MODE,
                                    CONFIG_BORIS_HEARTBEAT_SERVO_PWM_TIMER,
                                    CONFIG_BORIS_HEARTBEAT_SERVO_PWM_TIMER);
    if (heartbeat->servo) {
        err= fgr_task_create(&task_heartbeat_cb, heartbeat, "heartbeat",
                             3000,
                             3, &heartbeat->task_handle);
    }

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "heartbeat initialised with servo on pin %d.", CONFIG_BORIS_HEARTBEAT_SERVO_PIN);
    }

    return err;
}

// Deinitialise the heartbeat.
static void heartbeat_deinit(heartbeat_t *heartbeat)
{
    if (heartbeat->task_handle) {
        fgr_task_destroy(heartbeat->task_handle);
    }
    if (heartbeat->servo) {
        servo_destroy(heartbeat->servo);
    }
}

#endif

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: INITIALISATION
 * -------------------------------------------------------------- */

// Initialisation.
static int32_t init(context_t *context)
{
    // Allow us to feed the watchdog
    esp_task_wdt_add(NULL);

    // Create mutex to guard the context
    int32_t err = -ESP_ERR_NO_MEM;
    context->lock = xSemaphoreCreateMutex();
    if (context->lock) {
        err = ESP_OK;
    }

    if (err == ESP_OK) {

        CONTEXT_LOCK(context->lock, "init");

        // Initialise tasking, FGR style
        if (err == ESP_OK) {
            err = fgr_task_init();
        }

        // Configure WS2812 LED operations needed for Boris's eyes, FGR style
        if (err == ESP_OK) {
            err = fgr_ws2812_init();
        }

        // Initialize NVS, FGR style (this application doesn't need it
        // but the debug code expects it so easier to initialise it)
        if (err == ESP_OK) {
            err = fgr_nvs_init();
        }

        // Configure the debug LED, FGR style
        if (err == ESP_OK) {
            err = fgr_debug_init();
        }

        // Initialise the servo driver
        if (err == ESP_OK) {
            err = servo_init();
        }

#if defined(CONFIG_BORIS_CIGAR_LED_PIN) && (CONFIG_BORIS_CIGAR_LED_PIN >= 0)
        // Configure the cigar LED, PWM driven
        if (err == ESP_OK) {
            err = cigar_init(&context->cigar);
        }
#endif

#if defined(CONFIG_BORIS_EYES_WS2812_LED_PIN) && (CONFIG_BORIS_EYES_WS2812_LED_PIN >= 0)
        // Configure the WS2812 LEDs that are the eyes
        if (err == ESP_OK) {
            err = eyes_init(&context->eyes);
        }
#endif

#if defined(CONFIG_BORIS_HEARTBEAT_SERVO_PIN) && (CONFIG_BORIS_HEARTBEAT_SERVO_PIN >= 0)
        // Configure the heartbeat servo
        if (err == ESP_OK) {
            err = heartbeat_init(&context->heartbeat);
        }
#endif

        CONTEXT_UNLOCK(context->lock, "init");
    }

    return err;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Entry point
void app_main(void)
{
    context_t *context = &g_context;

    ESP_LOGI(TAG, "app_main start");

    esp_err_t err = init(context);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "initialization complete.");
#if defined(CONFIG_BORIS_EYES_WS2812_LED_PIN) && (CONFIG_BORIS_EYES_WS2812_LED_PIN >= 0)
        int32_t seconds = 0;
        int32_t last_eye_blink = seconds;
        int32_t next_eye_flash_interval_seconds = 0;
        size_t next_eye_colour_index = 0;
#endif
        while (1) {
#if defined(CONFIG_BORIS_EYES_WS2812_LED_PIN) && (CONFIG_BORIS_EYES_WS2812_LED_PIN >= 0)
            // The cigar and heartbeat all run automatically,
            // just need to flash the eyes at a random interval
            if (seconds - last_eye_blink >= next_eye_flash_interval_seconds) {
                last_eye_blink = seconds;
                eyes_t *eyes = &context->eyes;
                size_t index = next_eye_colour_index;
                next_eye_flash_interval_seconds = (((uint64_t) (EYES_INTERVAL_MAX_SECONDS - EYES_INTERVAL_MIN_SECONDS)) * esp_random() / UINT32_MAX) + EYES_INTERVAL_MIN_SECONDS;
                next_eye_colour_index = ((uint64_t) FGR_UTIL_ARRAY_LENGTH(g_eye_colour)) * esp_random() / UINT32_MAX;
                ESP_LOGI(TAG, "flashing eyes %s, next eye flash colour will be %s in %d second(s).",
                         g_eye_colour_name[index], g_eye_colour_name[next_eye_colour_index], next_eye_flash_interval_seconds);
                fgr_ws2812_led_flash(eyes->ws2812_handle, -1, EYES_FLASH_DURATION_MS,
                                     g_eye_colour[index]);
            }
            seconds++;
#endif
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset();
        }
        esp_task_wdt_delete(NULL);
    } else {
        ESP_LOGE(TAG, "initialization failed, system cannot continue, will restart soonish.");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

#if defined(CONFIG_BORIS_HEARTBEAT_SERVO_PIN) && (CONFIG_BORIS_HEARTBEAT_SERVO_PIN >= 0)
    heartbeat_deinit(&context->heartbeat);
#endif
#if defined(CONFIG_BORIS_EYES_WS2812_LED_PIN) && (CONFIG_BORIS_EYES_WS2812_LED_PIN >= 0)
    eyes_deinit(&context->eyes);
#endif
#if defined(CONFIG_BORIS_CIGAR_LED_PIN) && (CONFIG_BORIS_CIGAR_LED_PIN >= 0)
    cigar_deinit(&context->cigar);
#endif
    servo_deinit();
    fgr_debug_deinit();
    fgr_ws2812_deinit();
    fgr_task_deinit();
    if (context->lock) {
        vSemaphoreDelete(context->lock);
    }
    esp_restart();
}

// End of file
