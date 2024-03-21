zephyr_library()

zephyr_library_sources(
  lorawan_cid_request_management.c
  lorawan_dwn_ack_management.c
  lorawan_join_management.c
  lorawan_send_management.c
)

zephyr_include_directories(.)
