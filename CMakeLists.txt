cmake_minimum_required(VERSION 3.9)
project(uMi9s)

set(CMAKE_CXX_STANDARD 11)

add_executable(uMi9s
        hal-config/hal-config.h
        application_properties.c
        ble-configuration.h
        board_features.h
        dmadrv_config.h
        gatt_db.c
        gatt_db.h
        hal-config-app-common.h
        hal-config-board.h
        hal-config.h
        init_app.c
        init_app.h
        init_board.c
        init_board.h
        init_mcu.c
        init_mcu.h
        main.c
        tempdrv_config.h
        uartdrv_config.h)
