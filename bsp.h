/*******************************************************************************
 * File:     bsp.h
 * Author:   LIR
 * Version:  v1.0 
 * Date:     12th August 2018
 *
 * Brief:    Snake board support package header file. Contains the macro
 *           definitions, typedefs, and function prototypes used across main.c
 *           and bsp.c.
 *
 *           (N.B. SystemCoreClock and PLL multiplication factor have been 
 *           changed in system_stm32f0xx.c to "SYS_CLOCK_HZ" (see bsp.h)  
 *           and *3 respectively).
 *
 ******************************************************************************/



#ifndef __BSP_H__                                                               
#define __BSP_H__                                                               



/* Macro definitions ---------------------------------------------------------------------------------------------------------------------------------------------*/
#define SYS_CLOCK_HZ    24000000u                                               // CPU clock speed. Used in system_stm32f0xx.c and main.c
#define FRAME_RATE_HZ   60u                                                     // LED array frame rate (60Hz recommended)
#define GAME_FRAMES     6u                                                      // Number of LED frames to complete between successive game logic routines. Must be a positive integer > 1
#define FLASH_RATE      2u                                                      // Number of times between GAME_FRAMES to toggle Goal LED. Make an EVEN divisor of GAME_FRAMES (but < GAME_FRAMES)
#define ON_GAME_FRAME   (row_counter == (16u*GAME_FRAMES))                      // Perform the game logic between the end of GAME_FRAMES, and the start of the next frame
#define ON_FLASH_FRAME  ((row_counter % (16u*GAME_FRAMES/FLASH_RATE)) == 0u)    // Used in SysTick_Handler() to determine when flashing LEDs should be toggled on/off

#define START_X         4u                                                      
#define START_Y         7u

#define OLD_T_POSITION  (1u << Snake.added_length) 

#define SEL_PRESSED     gp_data & (1u << 7)
#define A_PRESSED       gp_data & (1u << 6)
#define STA_PRESSED     gp_data & (1u << 5)
#define B_PRESSED       gp_data & (1u << 4)
#define D_PRESSED       gp_data & (1u << 3)
#define L_PRESSED       gp_data & (1u << 2)
#define R_PRESSED       gp_data & (1u << 1)
#define U_PRESSED       gp_data & (1u << 0)
                                                       
#define LED_VCC         GPIO_BSRR_BS_0                                          // PA0
#define LED_COL_DATA    GPIO_BSRR_BS_1                                          // PA1
#define LED_CLK         GPIO_BSRR_BS_2                                          // PA2
#define LED_LATCH       GPIO_BSRR_BS_3                                          // PA3
#define LED_ROW_DATA    GPIO_BSRR_BS_4                                          // PA4
#define LED_NOT_OE      GPIO_BSRR_BS_5                                          // PA5

#define PAD_CLK         GPIO_BSRR_BS_4                                          // PB4
#define PAD_SER_OUT     GPIO_BSRR_BS_5                                          // PB5
#define PAD_NOT_PE      GPIO_BSRR_BS_6                                          // PB6
#define PAD_VCC         GPIO_BSRR_BS_7                                          // PB7



/* Type definitions ----------------------------------------------------------------------------------------------------------------------------------------------*/
typedef struct {                                                                
    uint8_t  added_length;                                                      // Ensure added_length does not exceed element width of velocities[4], minus 1
    uint8_t  hx;
    uint8_t  hy;
    uint8_t  tx;
    uint8_t  ty;
    uint8_t  reserved0;                                                         // Pad data structure to even number of bytes
    uint32_t velocities[4];                                                     // Velocities (+x, -x, +y, -y) for each Snake segment, stored as bit fields for compactness
} Snake_t;

typedef struct {
    uint8_t x;
    uint8_t y;
} Coord_t;



/* External variable declarations --------------------------------------------------------------------------------------------------------------------------------*/
extern uint16_t const num_bmps[10];

extern int8_t   volatile is_game_over;
extern uint8_t  volatile row_counter;
extern uint8_t  prev_gp_data;                                                    
extern uint8_t  player_score;
extern uint16_t fbuffer[16];
extern uint32_t player_lfsr;
extern Coord_t  Goal;
extern Snake_t  Snake;



/* Function prototypes -------------------------------------------------------------------------------------------------------------------------------------------*/
void gpio_init(void);
void clock_out(uint16_t x_data, uint16_t y_data);
uint8_t clock_in(void);

void place_point(uint8_t x, uint8_t y);
void toggle_point(uint8_t x, uint8_t y);
int is_collision(uint8_t x, uint8_t y);
int is_goal(uint8_t x, uint8_t y);

void get_seed(void);
uint32_t random(uint32_t limit);
void place_goal(void);

void shift_velocity_fields(void);
Coord_t process_input(uint8_t gp_data);
void move_tail(uint8_t next_hx, uint8_t next_hy);

void place_num(uint16_t bmp, uint8_t x, uint8_t y);
void draw_score(void);
void game_over(void);

void SysTick_Handler(void);



#endif //__BSP_H__ 
