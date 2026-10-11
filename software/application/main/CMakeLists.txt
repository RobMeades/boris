idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "boris.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_timer esp_driver_gpio esp_driver_ledc fgr_util fgr_ws2812 fgr_debug fgr_task fgr_nvs servo)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
