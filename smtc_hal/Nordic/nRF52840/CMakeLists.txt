zephyr_library()

zephyr_library_sources(
  smtc_hal_crashlog.c
)



add_subdirectory(smtc_modem_hal)
add_subdirectory(src)
