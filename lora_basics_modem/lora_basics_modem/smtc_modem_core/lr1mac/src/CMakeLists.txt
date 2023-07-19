zephyr_library()

zephyr_library_sources (
        lr1mac_core.c
        lr1mac_utilities.c
        lr1_stack_mac_layer.c
        )

add_subdirectory(services)
add_subdirectory(smtc_real)
