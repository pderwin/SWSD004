zephyr_library()

zephyr_library_sources(
      src/lr11xx_bootloader.c
      src/lr11xx_gnss.c
      src/lr11xx_lr_fhss.c
      src/lr11xx_radio.c
      src/lr11xx_regmem.c
      src/lr11xx_system.c
      src/lr11xx_wifi.c
      )

zephyr_include_directories(./src)
