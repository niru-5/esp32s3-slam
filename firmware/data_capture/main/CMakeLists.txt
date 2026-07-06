set(BMI_API "${CMAKE_CURRENT_SOURCE_DIR}/../../../SparkFun_BMI270_Arduino_Library/src/bmi270_api")

idf_component_register(
    SRCS
        "data_capture.c"
        "camera.c"
        "imu.c"
        "sysstats.c"
        "server_local.c"
        "net_client.c"
        "sdcard.c"
        "${BMI_API}/bmi2.c"
        "${BMI_API}/bmi270.c"
    INCLUDE_DIRS
        "."
        "${BMI_API}"
)
