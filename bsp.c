/*******************************************************************************
 * File:     bsp.c
 * Author:   LIR
 * Version:  v1.0 
 * Date:     12th August 2018
 *
 * Brief:    Snake board support package C file. Contains the definitions 
 *           of GPIO, game logic, and interrupt handler functions called 
 *           by/from main.c.
 *
 *           (N.B. SystemCoreClock and PLL multiplication factor have been 
 *           changed in system_stm32f0xx.c to "SYS_CLOCK_HZ" (see bsp.h)  
 *           and *3 respectively).
 *
 ******************************************************************************/



/* Includes ------------------------------------------------------------------------------------------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "bsp.h"



/* Function definitions ------------------------------------------------------------------------------------------------------------------------------------------*/

/* Initialises the GPIOA and GPIOB pins used in the application -----------------------------------------------------------------------------------*/
void gpio_init(void) {
    RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN);                   // Enable clock for GPIOA and GPIOB ports
          
    /* GPIOA (LED array) initialisation --------------------------------------*/
    GPIOA->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 |                // Enable output mode for GPIOA output pins (PA5 comes later)
                     GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 |
                     GPIO_MODER_MODER4_0);                
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR1 | GPIO_OSPEEDER_OSPEEDR2 |        // Set dynamic GPIOA output pins to HIGH SPEED
                       GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4);        
    GPIOA->PUPDR |= 0xA6AAA000u;                                                // Enable pull-down resistors on remaining GPIOA input pins

    /* GPIOB (gamepad) initialisation ----------------------------------------*/
    GPIOB->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER6_0 |                // Enable output mode for GPIOB output pins
                     GPIO_MODER_MODER7_0);                                      
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR6);        // Set dynamic GPIOB output pins to HIGH SPEED
    GPIOB->PUPDR |= 0xAAAA08AAu;                                                // Enable pull-down resistors on remaining GPIOB input pins

    /* LED array and gamepad initial power on --------------------------------*/
    GPIOA->BSRR |= LED_VCC;                                                     // Power on static 3V pins on LED board                                            
    clock_out(0u, 0u);                                                          // Flush LED row and column registers with 0's
    GPIOA->MODER |= GPIO_MODER_MODER5_0;                                        // Enable LED_NOT_OE pin as an output
    GPIOA->BRR |= LED_NOT_OE;                                                   // Pull LED_NOT_OE low to enable LED register output
    GPIOB->BSRR |= PAD_VCC;                                                     // Power on static 3V pins on gamepad
}



/* Clocks-out and latches 16-bit column and row data to LED shift registers serially via respective GPIO pins -------------------------------------*/
void clock_out(uint16_t x_data, uint16_t y_data) {
    for(int i=0; i<16; i++) {
        /* Write left-most x data bit to cathode (row) pin -------------------*/
        if( x_data & (1u << 15) )  GPIOA->BSRR |= LED_ROW_DATA;     
        else                       GPIOA->BRR  |= LED_ROW_DATA;

        /* Write left-most y data bit to anode (column) pin ------------------*/
        if( y_data & (1u << 15) )  GPIOA->BSRR |= LED_COL_DATA;     
        else                       GPIOA->BRR  |= LED_COL_DATA;

        /* Shift out x and y data at same time via a shared clock signal -----*/
        GPIOA->BSRR |= LED_CLK;
        GPIOA->BRR  |= LED_CLK;

        /* Bit shift x and y data to left for next bit-bang ------------------*/
        x_data <<= 1u;
        y_data <<= 1u;
    }
    /* Latch completed x and y data string to each shift register ------------*/
    GPIOA->BSRR |= LED_LATCH;
    GPIOA->BRR  |= LED_LATCH;  
}



/* Clocks in 8-bit data from the gamepad serially, and returns this data to the caller ------------------------------------------------------------*/
/* gp_data stored in final form: SEL, A, STA, B, D, L, R, U ---------------------------------------------------------------------------------------*/
uint8_t clock_in(void) {
    /* Latch parallel data to shift register ---------------------------------*/
    GPIOB->BRR  |= PAD_NOT_PE;
    GPIOB->BSRR |= PAD_CLK;
    GPIOB->BRR  |= PAD_CLK;
    GPIOB->BSRR |= PAD_NOT_PE;
    
    /* Read and store first bit, and continue to shift data out serially -----*/
    uint8_t gp_data = 0u;
    for(int i=0; i<8; i++) {
        gp_data <<= 1;
        if( GPIOB->IDR & (1u << 5) )  gp_data |= 1u;
        GPIOB->BSRR |= PAD_CLK;
        GPIOB->BRR  |= PAD_CLK;
    }
    return gp_data;
}



/* Places a point on the board --------------------------------------------------------------------------------------------------------------------*/
void place_point(uint8_t x, uint8_t y) {
    fbuffer[y] |= (0x8000u >> x);
}



/* Toggles a point on the board on/off ------------------------------------------------------------------------------------------------------------*/
void toggle_point(uint8_t x, uint8_t y) {
    fbuffer[y] ^= (0x8000u >> x);
}



/* Returns 1 if given coordinate is already lit on the board, and 0 otherwise ---------------------------------------------------------------------*/
int is_collision(uint8_t x, uint8_t y) {
    if( fbuffer[y] & (0x8000u >> x) )  return 1;
    else                               return 0;
}



/* Returns 1 if given coordinate is same as current Goal coordinate, and 0 otherwise --------------------------------------------------------------*/
int is_goal(uint8_t x, uint8_t y) {
    if( (x == Goal.x) && (y == Goal.y) )  return 1;
    else                                  return 0;
}



/* Uses the time taken by the player to initially press START on the gamepad to generate a seed value for random() --------------------------------*/
void get_seed(void) {
    uint8_t  gp_data;
    uint32_t counter = 0u;

    /* Avoids holding START button before power on to get zero seed ----------*/
    while(1) {
        gp_data = clock_in();
        if( gp_data == 0u ) {
            /* Count loops till START is pressed, and use value for seed -----*/
            while(1) {
                gp_data = clock_in();
                if(STA_PRESSED) {
                    player_lfsr = counter;                                      // Seed the initial LFSR in RAM with the value of counter
                    return;
                }
                counter++;
            }
            /*----------------------------------------------------------------*/
        }
    }
    /*------------------------------------------------------------------------*/
}



/* Returns a random uint32_t between 0 and limit (exclusive), based on a xorshift LFSR. Before using this for the first time, call get_seed() -----*/
uint32_t random(uint32_t limit) {
    uint32_t lfsr = player_lfsr;                                                // Copy current lfsr from RAM
    lfsr ^= (lfsr << 13u);
    lfsr ^= (lfsr >> 17u);
    lfsr ^= (lfsr << 5u);
    player_lfsr = lfsr;                                                         // Update RAM with new lfsr    
    return (lfsr % limit);                                                      // Also return correctly truncated random number to caller                                                    
}



/* Randomly places a goal on the board such that it is not occupied by the Snake ------------------------------------------------------------------*/
void place_goal(void) {
    uint32_t rx, ry;
    while(1) {
        rx = random(16u);
        ry = random(16u);
        if( fbuffer[ry] & (0x8000u >> rx) )  continue;                          // If random coordinate occupied by Snake, roll again
        fbuffer[ry] |= (0x8000u >> rx);                                         // Else, place goal on board
        Goal.x = rx;                                                            
        Goal.y = ry;
        return;
    }
}



/* Bit shifts all velocity data in Snake data structure once to left to simulate Snake movement ---------------------------------------------------*/
void shift_velocity_fields(void) {
    Snake.velocities[0] <<= 1u;
    Snake.velocities[1] <<= 1u;
    Snake.velocities[2] <<= 1u;
    Snake.velocities[3] <<= 1u;
}



/* Shifts velocity fields, inserts new head velocity into the gap, and returns new head coords depending on input gp_data -------------------------*/
Coord_t process_input(uint8_t gp_data) { 
    shift_velocity_fields();                                                    // Shift Snake velocity fields left by 1 to accomodate new head velocity bit
                                                        
    if( gp_data & 0xFu )  prev_gp_data = gp_data;                               // If D-pad is pressed, update prev_gp_data and process the new data 
    else                  gp_data = prev_gp_data;                               // Else D-pad data has not been pressed, so process prev_gp_data instead

    /* Determine D-pad button pressed from gp_data and process accordingly ---*/
    if(     D_PRESSED) {                                                
        Snake.velocities[3] |= 1u;                                              // Set new head velocity by writing 1 to appropriate velocity field
        if(Snake.hy == 0u)   Snake.hy = 15u;                                    // Calculate new head coordinate (with wrapping)
        else                 Snake.hy -= 1u;
    }                                                                           
    else if(L_PRESSED) {                                            
        Snake.velocities[1] |= 1u;                                                     
        if(Snake.hx == 0u)   Snake.hx = 15u;                                     
        else                 Snake.hx -= 1u;
    }                                                                                                                                                      
    else if(R_PRESSED) {                                       
        Snake.velocities[0] |= 1u;                                                     
        if(Snake.hx == 15u)  Snake.hx = 0u;                                      
        else                 Snake.hx += 1u;                                                                
    }                                                                                                                                                   
    else if(U_PRESSED) {                                         
        Snake.velocities[2] |= 1u;                                                     
        if(Snake.hy == 15u)  Snake.hy = 0u;                                      
        else                 Snake.hy += 1u;                                                                     
    }
    Coord_t Nexthead = {Snake.hx, Snake.hy};                                    
    return Nexthead;
}



/* Removes current tail from board. Updates new tail position based on old tail velocity data in Snake data struct --------------------------------*/
void move_tail(uint8_t next_hx, uint8_t next_hy) {
    uint8_t tx = Snake.tx;                                                      
    uint8_t ty = Snake.ty;
    toggle_point(tx, ty);                                                       

    /* For added_length of 0, new tail position = new head position ----------*/
    if(Snake.added_length == 0u) {
        Snake.tx = next_hx;                                                     
        Snake.ty = next_hy;
    } 
    /* For added_length > 0, get new tail position from old tail velocity ----*/
    else if(Snake.velocities[0] & OLD_T_POSITION) {                             // If non-zero velocity in +x direction...
        if(tx == 15u)  Snake.tx = 0u;                                           // Wrap position if crossing board edge
        else           Snake.tx = (tx + 1u);                                    // Otherwise, increment normally
    }
    else if(Snake.velocities[1] & OLD_T_POSITION) {                             // If non-zero velocity in -x direction...
        if(tx == 0u)   Snake.tx = 15u;                                          
        else           Snake.tx = (tx - 1u);
    }
    else if(Snake.velocities[2] & OLD_T_POSITION) {                             // If non-zero velocity in +y direction...
        if(ty == 15u)  Snake.ty = 0u;
        else           Snake.ty = (ty + 1u);
    }
    else if(Snake.velocities[3] & OLD_T_POSITION) {                             // If non-zero velocity in -y direction...
        if(ty == 0u)   Snake.ty = 15u;
        else           Snake.ty = (ty - 1u);
    }
}



/* Place a number bitmap on the fbuffer[] at a specified x, y (upper-left aligned) ----------------------------------------------------------------*/
void place_num(uint16_t bmp, uint8_t x, uint8_t y) {
    uint16_t bitmask = 0xE000u;
    uint16_t temp;
    for(int i=0; i<5; i++) {
        temp = (bmp & bitmask) >> x;
        fbuffer[y] |= temp;
        bmp <<= 3;
        if(y == 0u) return;
        y--;
    }
} 



/* Read the final player_score from RAM, read each digit, and draw each to the screen. Max value of 99  -------------------------------------------*/
void draw_score(void) {
    uint8_t final_player_score = player_score;
    uint8_t lsd;
    uint8_t msd;

    lsd = final_player_score % 10u;                                             
    place_num(num_bmps[lsd], 4u, 15u);

    msd = final_player_score / 10u;                                             
    place_num(num_bmps[msd], 0u, 15u);
}



/* Clear board, draw final score to screen for approx. 5 seconds, then reset system ---------------------------------------------------------------*/
void game_over(void) {
    is_game_over = 1;                                                           // Update this is RAM, for use by SysTick_Handler()
    for(int i=0; i<16; i++) {
        fbuffer[i] = 0u;
    }
    draw_score();
    uint32_t volatile counter = 0u;
    while(1) {                                                                  
        counter++;
        if(counter > 10000000u) break;
    }
    NVIC_SystemReset();                                                         
}



/* Interrupt handler that draws a new LED row each time the SysTick counter reaches 0 -------------------------------------------------------------*/
void SysTick_Handler(void) {
    uint16_t x_data;
    uint16_t y_data;
    uint32_t row_number = (row_counter % 16u);
        
    /* Fetch and clock out x and y data for the current row_number -----------*/
    x_data = fbuffer[row_number];                                               
    y_data = ( 1u << (15-row_number) );                                         
    clock_out(x_data, y_data);                                                  // Ensure there are no conditional branches above this line to prevent stutter                                                                         
    row_counter++;                                                              

    /* Flash Goal LED on certain frames (will not affect game logic) ---------*/
    if( ON_FLASH_FRAME && (is_game_over == 0) ) {                                    
        toggle_point(Goal.x, Goal.y);
    }
}