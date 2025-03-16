#ifndef __MN28016A
#define __MN28016A

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// 定义引脚
#define CLK GPIO_NUM_2  // SPI CLK
#define BLK GPIO_NUM_8  // BK使能信号 (对应数据表中的BLK)
#define LAT GPIO_NUM_9  // LAT锁存信号
#define SIN GPIO_NUM_3  // SPI MOSI/SIN

#define SPI_DEVICE SPI2_HOST  // 要使用的SPI通道

/**
 * 屏幕尺寸定义禁止修改
 */
#define VFD_DISPLAY_WIDTH 280  // 屏幕实际像素长度 X
#define VFD_DISPLAY_HEIGHT 16  // 屏幕实际像素宽度 Y

/**
 * 初始化VFD
 */
void init_vfd();

/**
 * 清空全部缓冲区
 */
void clear_buffers();

/**
 * 发送缓冲区数据到屏幕
 */
void send_buffer();

/**
 * 画一个点,起点从0,0开始
 */
void draw_pixel(uint16_t x, uint16_t y, bool state);

/**
 * 亮度调整0~255级
 */
void set_brightness(uint8_t level);

/**
 * 内部测试
 */
void test_vfd();

#ifdef __cplusplus
}
#endif
#endif