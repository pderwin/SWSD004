zephyr_library()

zephyr_library_sources(
    main_tracker_application.c
)

add_subdirectory(tracker_utility)

zephyr_include_directories( .)
zephyr_include_directories( tracker_utility )
zephyr_include_directories( ../smtc_hal/inc )
