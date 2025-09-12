# components/OLEDlib/CMakeLists.txt
idf_component_register(
    #SRCS "SSD1306.c"            # list your .c source files (omit if none)
    INCLUDE_DIRS "include"      # public headers are in include/
    REQUIRES driver             # if you use esp-idf driver APIs (i2c, gpio, etc.)
)
