zephyr_library()

zephyr_library_sources(
        src/lr1mac_class_b/smtc_beacon_sniff.c
        src/lr1mac_class_b/smtc_ping_slot.c
        src/lr1mac_class_c/lr1mac_class_c.c
        src/smtc_real/src/smtc_real.c

        )

add_subdirectory(src)
