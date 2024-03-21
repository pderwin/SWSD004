zephyr_library ()

zephyr_library_sources(
  gnss_helpers.c
  mw_common.c
  mw_gnss_almanac.c
  mw_gnss_scan.c
  mw_gnss_send.c
  mw_wifi_scan.c
  mw_wifi_send.c
  wifi_helpers.c
)

zephyr_include_directories(.)
