add_compile_options(-fsingle-precision-constant)

zephyr_compile_definitions( ADD_SMTC_ALC_SYNC=1 )
zephyr_compile_definitions( NUMBER_OF_STACKS=1 )
zephyr_compile_definitions( RP2_103 )
zephyr_compile_definitions( hal_mcu_trace_print=printk)
zephyr_compile_definitions( ADD_LBM_GEOLOCATION )

#
# for lr11xx_radio.c
#
zephyr_compile_definitions( LR11XX_DISABLE_HIGH_ACP_WORKAROUND )
zephyr_compile_definitions( LR11XX_DISABLE_WARNINGS )

#zephyr_include_directories( host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Inc )
#zephyr_include_directories( host_driver/STM32WBxx/hal_config )
#zephyr_include_directories( host_driver/STM32WBxx/Drivers/CMSIS/Device/ST/STM32WBxx/Include )

add_subdirectory(apps)
add_subdirectory(lora_basics_modem)
add_subdirectory(board)
add_subdirectory(smtc_hal)
