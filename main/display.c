#include "display.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include <time.h>

#define TAG "DISPLAY"
#if CONFIG_IDF_TARGET_ESP32C3
#define LED_STRIP_GPIO 6
#else
#define LED_STRIP_GPIO 4
#endif
#define NUM_LEDS 128
#define MATRIX_WIDTH 16
#define MATRIX_HEIGHT 8

uint8_t digit_color[4][3] = {{128,64,0},{128,64,0},{128,64,0},{128,64,0}};
uint8_t bar_color[3] = {64,128,0};
uint8_t digit_brightness = 255;
uint8_t bar_brightness = 255;
uint8_t torch_brightness = 255;
bool torch_mode = false;
bool blink_last_dot = false;

static led_strip_handle_t led_strip;
static uint8_t led_data[NUM_LEDS*3];

static int xy_to_index(int x,int y){
    int matrix_id = (x>=8)?0:1;
    int local_x = x%8;
    int local_y = 7-y;
    int index_in_matrix = local_y*8 + (7-local_x);
    return matrix_id*64 + index_in_matrix;
}

static const uint8_t font3x5[][5] = {
    {0b111,0b101,0b101,0b101,0b111},
    {0b010,0b110,0b010,0b010,0b111},
    {0b111,0b001,0b111,0b100,0b111},
    {0b111,0b001,0b111,0b001,0b111},
    {0b101,0b101,0b111,0b001,0b001},
    {0b111,0b100,0b111,0b001,0b111},
    {0b111,0b100,0b111,0b101,0b111},
    {0b111,0b001,0b010,0b010,0b010},
    {0b111,0b101,0b111,0b101,0b111},
    {0b111,0b101,0b111,0b001,0b111},
};

static void apply_brightness(uint8_t base_r,uint8_t base_g,uint8_t base_b,
                             uint8_t bri,uint8_t* out_r,uint8_t* out_g,uint8_t* out_b){
    *out_r = (base_r*bri)/255;
    *out_g = (base_g*bri)/255;
    *out_b = (base_b*bri)/255;
}

static void update_led_strip(){
    for(int i=0;i<NUM_LEDS;i++){
        int idx=i*3;
        led_strip_set_pixel(led_strip,i,led_data[idx+1],led_data[idx],led_data[idx+2]);
    }
    led_strip_refresh(led_strip);
}

static int last_digits[4] = {-1,-1,-1,-1};
static int x_coords[] = {9,13,0,4};

static void draw_digit(int x,int y,uint8_t digit,uint8_t r,uint8_t g,uint8_t b){
    if(digit>9) return;
    uint8_t adj_r,adj_g,adj_b;
    apply_brightness(r,g,b,digit_brightness,&adj_r,&adj_g,&adj_b);
    for(int row=0;row<5;row++){
        uint8_t rowBits=font3x5[digit][row];
        for(int col=0;col<3;col++){
            if(rowBits & (1<<(2-col))){
                int px=x+col;
                int py=y+row;
                if(px<MATRIX_WIDTH && py<MATRIX_HEIGHT){
                    int index=xy_to_index(px,py)*3;
                    led_data[index]=adj_g;
                    led_data[index+1]=adj_r;
                    led_data[index+2]=adj_b;
                }
            }
        }
    }
}

static void animate_digit_fall(int digit_index,int x,int y,uint8_t oldDigit,uint8_t newDigit,uint8_t r,uint8_t g,uint8_t b){
    uint8_t backup[NUM_LEDS*3];
    uint8_t adj_r,adj_g,adj_b;
    apply_brightness(r,g,b,digit_brightness,&adj_r,&adj_g,&adj_b);
    memcpy(backup,led_data,sizeof(led_data));
    for(int drop=0;drop<=MATRIX_HEIGHT;drop++){
        memcpy(led_data,backup,sizeof(led_data));
        for(int row=0;row<5;row++){
            uint8_t rowBits=font3x5[oldDigit][row];
            for(int col=0;col<3;col++){
                if(rowBits & (1<<(2-col))){
                    int px=x+col;
                    int py=y+row+drop;
                    if(py>=0 && py<MATRIX_HEIGHT){
                        int index=xy_to_index(px,py)*3;
                        led_data[index]=adj_g;
                        led_data[index+1]=adj_r;
                        led_data[index+2]=adj_b;
                    }
                }
            }
        }
        for(int j=0;j<4;j++){
            if(j==digit_index) continue;
            draw_digit(x_coords[j],1,last_digits[j],digit_color[j][0],digit_color[j][1],digit_color[j][2]);
        }
        update_led_strip();
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    for(int rise=MATRIX_HEIGHT;rise>=0;rise--){
        memset(led_data,0,sizeof(led_data));
        for(int row=0;row<5;row++){
            uint8_t rowBits=font3x5[newDigit][row];
            for(int col=0;col<3;col++){
                if(rowBits & (1<<(2-col))){
                    int px=x+col;
                    int py=y+row-rise;
                    if(py>=0 && py<MATRIX_HEIGHT){
                        int index=xy_to_index(px,py)*3;
                        led_data[index]=adj_g;
                        led_data[index+1]=adj_r;
                        led_data[index+2]=adj_b;
                    }
                }
            }
        }
        for(int j=0;j<4;j++){
            if(j==digit_index) continue;
            draw_digit(x_coords[j],1,last_digits[j],digit_color[j][0],digit_color[j][1],digit_color[j][2]);
        }
        update_led_strip();
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

static void draw_seconds_bar(int seconds,uint8_t r,uint8_t g,uint8_t b){
    uint8_t adj_r,adj_g,adj_b;
    apply_brightness(r,g,b,bar_brightness,&adj_r,&adj_g,&adj_b);
    int tick=(seconds*MATRIX_WIDTH)/60;
    for(int i=0;i<MATRIX_WIDTH;i++){
        int index=xy_to_index(i,MATRIX_HEIGHT-1)*3;
        bool on=i<tick;
        if(blink_last_dot && i==MATRIX_WIDTH-1 && tick==MATRIX_WIDTH-1){
            on=(seconds%2)!=0;
        }
        if(on){
            led_data[index]=adj_g;
            led_data[index+1]=adj_r;
            led_data[index+2]=adj_b;
        }else{
            led_data[index]=led_data[index+1]=led_data[index+2]=0;
        }
    }
}

static void clock_task(void* arg){
    while(1){
        time_t now=time(NULL);
        struct tm timeinfo;
        localtime_r(&now,&timeinfo);
        int digits[]={timeinfo.tm_min/10,timeinfo.tm_min%10,timeinfo.tm_hour/10,timeinfo.tm_hour%10};
        memset(led_data,0,sizeof(led_data));
        if(torch_mode){
            uint8_t r,g,b;
            apply_brightness(255,255,255,torch_brightness,&r,&g,&b);
            for(int i=0;i<NUM_LEDS;i++){
                int idx=i*3;
                led_data[idx]=g;
                led_data[idx+1]=r;
                led_data[idx+2]=b;
            }
        }else{
            draw_seconds_bar(timeinfo.tm_sec,bar_color[0],bar_color[1],bar_color[2]);
            for(int i=0;i<4;i++){
                if(digits[i]!=last_digits[i]){
                    animate_digit_fall(i,x_coords[i],1,last_digits[i],digits[i],digit_color[i][0],digit_color[i][1],digit_color[i][2]);
                    last_digits[i]=digits[i];
                }else{
                    draw_digit(x_coords[i],1,digits[i],digit_color[i][0],digit_color[i][1],digit_color[i][2]);
                }
            }
        }
        update_led_strip();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void display_init(void){
    led_strip_config_t strip_config={.strip_gpio_num=LED_STRIP_GPIO,
                                     .max_leds=NUM_LEDS,
                                     .led_model=LED_MODEL_WS2812,
                                     .flags.invert_out=false};
    led_strip_rmt_config_t rmt_config={.clk_src=RMT_CLK_SRC_DEFAULT,
                                       .resolution_hz=10*1000*1000,
                                       .mem_block_symbols=64};
    led_strip_new_rmt_device(&strip_config,&rmt_config,&led_strip);
    led_strip_clear(led_strip);
    xTaskCreate(clock_task,"clock_task",4096,NULL,5,NULL);
}

