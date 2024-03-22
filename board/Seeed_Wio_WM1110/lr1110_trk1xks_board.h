#pragma once

#include <stdint.h>

typedef enum
{
    GNSS_PATCH_ANTENNA = 1,
    GNSS_PCB_ANTENNA,
} smtc_board_gnss_antenna_t;

/*
 * battery capacity in mAh.
 */
#define TRACKER_BOARD_BATTERY_CAPACITY 2400

void     smtc_board_stop_periodic_led_pulse( void );
uint32_t smtc_board_read_busy_pin( void *context );
void     smtc_board_select_gnss_antenna(uint32_t which);
void     smtc_board_reset_radio( void *context );
void     smtc_board_hall_effect_enable_for_duration( uint32_t duration);
uint32_t is_accelerometer_detected_moved(void);
uint32_t get_hall_effect_irq_state(void);
uint32_t get_accelerometer_irq1_state(void);
void     smtc_board_hall_effect_enable(uint32_t true_false);
