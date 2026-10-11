# Servo driver
idf_component_register(SRCS "servo.c"
                       INCLUDE_DIRS "api"
                       PRIV_REQUIRES esp_driver_ledc esp_driver_gpio esp_timer fgr_util)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
