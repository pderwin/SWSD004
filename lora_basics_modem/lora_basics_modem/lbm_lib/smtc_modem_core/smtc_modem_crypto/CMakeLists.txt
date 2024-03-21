zephyr_library()

zephyr_library_sources(
	smtc_modem_crypto.c
	)

zephyr_include_directories(.)
zephyr_include_directories(smtc_secure_element)

add_subdirectory(soft_secure_element)
