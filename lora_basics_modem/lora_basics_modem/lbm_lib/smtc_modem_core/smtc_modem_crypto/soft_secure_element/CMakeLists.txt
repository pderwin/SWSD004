zephyr_library()

zephyr_library_sources(
	aes.c
	cmac.c
	soft_se.c
	)

zephyr_include_directories(.)
