/*******************************************************************************
 * File:     main.c
 * Author:   LIR
 * Version:  v1.0 
 * Date:     12th August 2018
 *
 * Brief:    Snake main program. Version 1.0 is the base game, including:
 *           random goal placement, continuous movement, screen edge wrapping, 
 *           collision detection, growth up to a length of 32, and score
 *           display on game over. Press start to play.
 *
 *           (N.B. SystemCoreClock and PLL multiplication factor have been 
 *           changed in system_stm32f0xx.c to "SYS_CLOCK_HZ" (see bsp.h)  
 *           and *3 respectively).
 *
 ******************************************************************************/



/* Includes ------------------------------------------------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f0xx.h"
#include "bsp.h"



/* ROM -----------------------------------------------------------------------------------------------------------------------------------------------------------*/           
uint16_t const num_bmps[10] = {
    0xF6DEu,  //0
    0x2492u,  //1
    0xE7CEu,  //2
    0xE79Eu,  //3
    0x92F2u,  //4
    0xF39Eu,  //5
    0xF3DEu,  //6
    0xE492u,  //7
    0xF7DEu,  //8
    0xF792u,  //9
};



/* RAM -----------------------------------------------------------------------------------------------------------------------------------------------------------*/
int8_t   volatile is_game_over;   
uint8_t  volatile row_counter;
uint8_t  prev_gp_data;                                                          // Remember previous gamepad data to allow continuous movement                                                          
uint8_t  player_score;                                              
uint16_t fbuffer[16];                                                           // Our framebuffer is also our collision map to save RAM                                                        
uint32_t player_lfsr;                                                           // Linear Feedback Shift Register for random number generation
Coord_t  Goal;                                                                                                                                   
Snake_t  Snake = {0u, START_X, START_Y, START_X, START_Y, 0u, 0u};              // Initial Snake data structure {added_length, hx, hy, tx, ty, reserved0, velocities[4]}  
    
 

/* Local variables -----------------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t gp_data;
Coord_t Nexthead;



/* Main program --------------------------------------------------------------------------------------------------------------------------------------------------*/
int main() {

    /* Initialisation --------------------------------------------------------*/
    gpio_init();                                          
    get_seed();
    place_goal();
    place_point(START_X, START_Y);
    SysTick_Config( SYS_CLOCK_HZ / (16u*FRAME_RATE_HZ) );                       // Start SysTick timer, counting down from argument value

    /* Game loop -------------------------------------------------------------*/
    while(1) {
        if(ON_GAME_FRAME) {
            gp_data = clock_in();
            Nexthead = process_input(gp_data);                                  // Update velocity fields, return next head coordinates
            if( is_goal(Nexthead.x, Nexthead.y) ) {
                place_point(Nexthead.x, Nexthead.y);                            // Current goal becomes new head...
                if(Snake.added_length == 31u) move_tail(Nexthead.x, Nexthead.y);// ...but no more length appended to Snake if at maximum length
                else                          Snake.added_length += 1u;         // ...otherwise Snake length increased by leaving tail where it is
                player_score++;
                place_goal();
            }
            else if( is_collision(Nexthead.x, Nexthead.y) && (prev_gp_data != 0u) ) {
                game_over();
            }
            else {                                                                                      
                move_tail(Nexthead.x, Nexthead.y);                              // Delete old tail...  
                place_point(Nexthead.x, Nexthead.y);                            // ...and place next head, to simulate movement
            }
            row_counter = 0u;                                                   // Reset row_counter to delay game logic again by GAME_FRAMES
        }    
    }           
    return 0;
}