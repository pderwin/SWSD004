add_subdirectory(Seeed_Wio_WM1110)

zephyr_include_directories(interface)
zephyr_include_directories(radio_drivers_hal)
zephyr_include_directories(Seeed_Wio_WM1110)

#
# temporary
#
zephyr_include_directories(smtc_lr11xx_board)
