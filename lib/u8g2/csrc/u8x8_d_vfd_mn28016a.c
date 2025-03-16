#include <mn28016a.h>
#include "u8g2.h"
#include "u8x8.h"

/**
 * 适配Arduino
 */
#ifdef ARDUINO
#include <Arduino.h>
#endif

/****************************************用户平台适配区*****************************************************
 */

/*************Arduino平台 Start************* */
#ifdef ARDUINO

#endif
/*************Arduino平台 END************* */

// U8X8回调函数 - 绘制瓦片
static uint8_t u8x8_d_mn28016a_vfd_draw_tile(u8x8_t* u8x8,
                                             uint8_t msg,
                                             uint8_t arg_int,
                                             void* arg_ptr) {
    static uint8_t tile_count = 0;
    switch (msg) {
        case U8X8_MSG_DISPLAY_DRAW_TILE: {
            u8x8_tile_t* tile = (u8x8_tile_t*)arg_ptr;
            uint16_t y_base = (tile->y_pos) * 8;  // y坐标基准
            uint16_t x = (tile->x_pos) * 8;
            uint8_t* data = tile->tile_ptr;  // 瓦片数据指针
            uint8_t cnt = tile->cnt;

            // 清除之前的缓冲区内容
            if (tile->y_pos == 0 && tile->x_pos == 0) {
                clear_buffers();
            }

            // 用于导出数据在Python中模拟点阵调试
            //  printf("Tile: x_pos=%d, y_pos=%d, cnt=%d\n", tile->x_pos,
            //        tile->y_pos, tile->cnt);
            // while (cnt > 0) {
            //     for (size_t col = 0; col < 8; col++) {
            //         uint8_t byte = data[col];
            //         printf("0x%.2x, ",byte);
            //     }
            //     cnt--;
            //     data += 8;
            //     x += 8;
            // }
            // printf("\n");

            while (cnt > 0) {
                // 垂直排列8行
                for (size_t col = 0; col < 8; col++) {
                    uint8_t byte = data[col];
                    for (size_t bit = 0; bit < 8; bit++) {
                        if (byte & (1 << bit)) {
                            draw_pixel(col + x, y_base + bit, true);
                        }
                    }
                }
                cnt--;
                data += 8;
                x += 8;
            }

            // 当下半屏数据处理完毕后，发送缓冲区
            if (tile->y_pos == 1) {
                send_buffer();
            }
        } break;
        case U8X8_MSG_DISPLAY_REFRESH:
            break;
        default:
            return 0;
    }
    return 1;
}

// 设备描述符
const u8x8_display_info_t u8x8_mn28016a_vfd_display_info = {
    /* chip_enable_level = */ 0,
    /* chip_disable_level = */ 1,
    /* post_chip_enable_wait_ns = */ 50,
    /* pre_chip_disable_wait_ns = */ 50,
    /* reset_pulse_width_ms = */ 1,
    /* post_reset_wait_ms = */ 6,
    /* sda_setup_time_ns = */ 50,
    /* sck_pulse_width_ns = */ 100,
    /* sck_clock_hz = */ 1000000UL,  // 无意义，仅占位
    /* spi_mode = */ 0,              // 无意义，仅占位
    /* i2c_bus_clock_100kHz = */ 4,  // 无意义，仅占位
    /* data_setup_time_ns = */ 50,
    /* write_pulse_width_ns = */ 100,
    /* tile_width = */ VFD_DISPLAY_WIDTH / 8,
    /* tile_height = */ VFD_DISPLAY_HEIGHT / 8,
    /* default_x_offset = */ 0,
    /* flip_mode_x_offset = */ 0,
    /* pixel_width = */ VFD_DISPLAY_WIDTH,
    /* pixel_height = */ VFD_DISPLAY_HEIGHT};

// 设备结构体
uint8_t u8x8_d_vfd_mn28016a(u8x8_t* u8x8,
                            uint8_t msg,
                            uint8_t arg_int,
                            void* arg_ptr) {
    switch (msg) {
        case U8X8_MSG_DISPLAY_SETUP_MEMORY:
            u8x8_d_helper_display_setup_memory(u8x8,
                                               &u8x8_mn28016a_vfd_display_info);
            break;
        case U8X8_MSG_DISPLAY_INIT:
            u8x8_d_helper_display_init(u8x8);
            init_vfd();
            break;
        case U8X8_MSG_DISPLAY_SET_POWER_SAVE:
            if (arg_int == 0) {
                set_brightness(255);  // 正常亮度
            } else {
                set_brightness(0);  // 关闭显示
            }
            break;
        case U8X8_MSG_DISPLAY_SET_FLIP_MODE:
            break;
        case U8X8_MSG_DISPLAY_SET_CONTRAST:
            set_brightness(arg_int);
            break;
    }
    return u8x8_d_mn28016a_vfd_draw_tile(u8x8, msg, arg_int, arg_ptr);
}