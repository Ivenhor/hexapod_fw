idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        ST_servo
        xbox_input
        hexapod_core_c
)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu11)
