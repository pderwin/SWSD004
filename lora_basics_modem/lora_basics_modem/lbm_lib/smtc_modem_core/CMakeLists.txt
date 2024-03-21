zephyr_library()

zephyr_library_sources(
	smtc_modem.c
	smtc_modem_test.c
	)

zephyr_include_directories(.)

add_subdirectory(geolocation_services)
add_subdirectory(lorawan_api)
add_subdirectory(lorawan_packages)
add_subdirectory(lorawan_manager)
add_subdirectory(lr1mac)
add_subdirectory(modem_services)
add_subdirectory(modem_supervisor)
add_subdirectory(modem_utilities)
add_subdirectory(radio_drivers)
add_subdirectory(radio_planner)
add_subdirectory(smtc_modem_crypto)
add_subdirectory(smtc_ral)
add_subdirectory(smtc_ralf)
