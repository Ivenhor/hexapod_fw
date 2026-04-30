idf_component_register(
    SRCS
        "src/hexapod_kinematics_c.c"
        "src/hexapod_foothold_planner_c.c"
        "src/hexapod_leg_controller_c.c"
        "src/hexapod_gait_c.c"
        "src/hexapod_controller_c.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_common
)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu11 -Wall -Wextra)
