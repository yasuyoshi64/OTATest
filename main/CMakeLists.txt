idf_component_register(SRCS "main.cpp" "main_config.cpp" "sd_card.cpp" "wifi.cpp"
                    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++23 -fexceptions)
