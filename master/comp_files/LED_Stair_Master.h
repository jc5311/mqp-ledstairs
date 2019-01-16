/**
 * Author(s):    Juan Caraballo
 * Created:   1/16/19
 * Copyright 2019, Juan Caraballo, All rights reserved.
 * 
 * Description: This header file contains abstracted functions and their
 * definitions for programming master MCU devices for the WPI LED Stair MQP 
 * project.
 */

#ifndef LED_STAIR_MASTER_H_
#define LED_STAIR_MASTER_H_

/**
 * Set a LED bar in a LED bar chain to a given color.
 * @param bar_addr The device address of an led bar
 * @param red 8-bit red channel luminosity
 * @param green 8-bit green channel luminosity
 * @param blue 8-bit blue channel luminosity
 * @param range_type 8-bit range direction; NORMAL_RANGE=0-255 FLIPPED_RANGE=255-0
 */
void setBarColor(uint8_t bar_addr, uint8_t red,
                 uint8_t green, uint8_t blue, uint8_t range_type);

/**
 * Disable animation lighting for the first n led bars in a chain.
 * @param led_bar_count Number of led bars to disable. Enter the total to disable all.
 * @param led_bar[] Array of led bar addresses. Used to precisely signal which bar to disable.
 */
void disableLedBars(uint8_t led_bar_count, uint8_t led_bar[]);



#endif //LED_STAIR_MASTER_H_
