#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <pigpio.h> 

#define INTERLOCK_PIN 24 // E-Stop / Lid Switch (Pull to Ground)
#define RED_DOT_PIN 25   // Red Framing Laser

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t laser_on;
    uint8_t pwm_val;
    uint32_t delay_us;
} GalvoCmd;

int run_trajectory(GalvoCmd* cmds, int num_cmds, volatile int* status_flag, 
                   volatile int* progress, volatile int* current_x, 
                   volatile int* current_y, volatile int* current_pwm,
                   volatile double* live_speed_factor) { 

    if (gpioInitialise() < 0) return -1;

    // Initialize Safety & Framing Hardware
    gpioSetMode(INTERLOCK_PIN, PI_INPUT);
    gpioSetPullUpDown(INTERLOCK_PIN, PI_PUD_UP); 
    gpioSetMode(RED_DOT_PIN, PI_OUTPUT);
    gpioWrite(RED_DOT_PIN, 0);

    // 20MHz SPI mapping
    int spi = spiOpen(0, 20000000, 0);
    if (spi < 0) {
        gpioTerminate();
        return -2;
    }

    char buf[2];
    int last_pwm = -1; 

    for (int i = 0; i < num_cmds; i++) {
        
        // HARDWARE INTERLOCK CHECK
        if (gpioRead(INTERLOCK_PIN) == 1) { 
            gpioPWM(18, 0);
            gpioWrite(RED_DOT_PIN, 0);
            *status_flag = 5; // Force Python state to "Stopped"
            break;
        }

        if (*status_flag == 1) {
            gpioPWM(18, 0);
            *current_pwm = 0;
            last_pwm = 0;
            while (*status_flag == 1) {
                gpioDelay(50000); 
            }
        }
        if (*status_flag == 2) break; 

        uint16_t vx = cmds[i].x & 0x0FFF;
        buf[0] = 0x30 | (vx >> 8);
        buf[1] = vx & 0xFF;
        spiWrite(spi, buf, 2);

        uint16_t vy = cmds[i].y & 0x0FFF;
        buf[0] = 0xB0 | (vy >> 8);
        buf[1] = vy & 0xFF;
        spiWrite(spi, buf, 2);

        // RED DOT LOGIC (Framing state = 2)
        if (*status_flag == 2) { 
            gpioWrite(RED_DOT_PIN, 1); 
            cmds[i].pwm_val = 0;       
        } else {
            gpioWrite(RED_DOT_PIN, 0);
        }

        // DMA Throttle cache
        if (cmds[i].pwm_val != last_pwm) {
            gpioPWM(18, cmds[i].pwm_val);
            last_pwm = cmds[i].pwm_val;
            *current_pwm = cmds[i].pwm_val;
        }

        *current_x = vx;
        *current_y = vy;

        if (cmds[i].delay_us > 0) {
            double current_speed = *live_speed_factor;
            if (current_speed < 0.01) current_speed = 0.01; 
            
            uint32_t final_delay = (uint32_t)(cmds[i].delay_us / current_speed);
            if (final_delay > 0) {
                gpioDelay(final_delay);
            }
        }

        if (i % 5000 == 0 || i == num_cmds - 1) {
            *progress = (int)(((float)i / (num_cmds - 1)) * 100);
        }
    }

    gpioPWM(18, 0);
    gpioWrite(RED_DOT_PIN, 0);
    *current_pwm = 0;
    spiClose(spi);
    gpioTerminate();
    
    return 0;
}
