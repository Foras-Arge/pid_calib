<<<<<<< HEAD
idf_component_register(SRCS "main.c" \
                       INCLUDE_DIRS "")
=======
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES drv8308 abp_sensor
)
>>>>>>> main
