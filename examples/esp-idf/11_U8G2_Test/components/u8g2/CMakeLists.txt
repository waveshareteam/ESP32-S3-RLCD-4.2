file(GLOB U8G2_SRCS
    "csrc/u8g2_*.c"
    "csrc/u8x8_*.c"
)

list(FILTER U8G2_SRCS EXCLUDE REGEX ".*/u8g2_d_setup\\.c$")
list(FILTER U8G2_SRCS EXCLUDE REGEX ".*/u8x8_d_.*\\.c$")

idf_component_register(
    SRCS ${U8G2_SRCS}
    INCLUDE_DIRS "csrc"
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC U8X8_WITH_USER_PTR)
