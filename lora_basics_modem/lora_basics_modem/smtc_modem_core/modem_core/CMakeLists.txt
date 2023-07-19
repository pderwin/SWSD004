zephyr_library()

zephyr_compile_definitions( RP2_103 )

zephyr_library_sources(
        smtc_modem.c
        smtc_modem_test.c
        )
