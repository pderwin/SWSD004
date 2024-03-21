zephyr_library()

zephyr_library_sources(
  fifo_ctrl.c
  modem_core.c
  modem_event_utilities.c
)

zephyr_include_directories(.)
