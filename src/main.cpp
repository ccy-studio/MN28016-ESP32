#include <Arduino.h>
#include <U8g2lib.h>
#include "mn28016a.h"

// 创建U8G2对象，使用硬件SPI
// U8G2_R0/1/2/3/U8G2_MIRROR = 不旋转、顺90、顺180、顺270、镜像
U8G2_MN28016A_280X16_F_4W_SW_SPI u8g2(U8G2_R0);

// 定义测试模式数量
#define TEST_MODE_COUNT 8  // 增加到测试模式
int currentTestMode = 0;
unsigned long lastModeChange = 0;
const int modeChangeInterval = 3000;  // 每个测试模式显示3秒

void setup() {
    Serial.begin(115200);
    Serial.println("MN28016A VFD with U8G2 Example");
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setDrawColor(1);
    u8g2.setFontPosTop();
    u8g2.setContrast(255);  // 设置初始亮度为最大

    // 设置中文字体 - 使用wqy字体(文泉驿)
    u8g2.setFont(u8g2_font_wqy15_t_chinese3);

    // 显示中文文本
    u8g2.drawUTF8(10, 2, "测试VFD屏幕");
    u8g2.sendBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    delay(2000);
}

// 测试1：四角像素点
void testCornerPixels() {
    u8g2.clearBuffer();
    // 四个角落的像素点
    u8g2.drawPixel(0, 0);
    u8g2.drawPixel(1, 0);
    u8g2.drawPixel(0, VFD_DISPLAY_HEIGHT - 1);
    u8g2.drawPixel(1, VFD_DISPLAY_HEIGHT - 1);
    u8g2.drawPixel(VFD_DISPLAY_WIDTH - 1, 0);
    u8g2.drawPixel(VFD_DISPLAY_WIDTH - 2, 0);
    u8g2.drawPixel(VFD_DISPLAY_WIDTH - 2, VFD_DISPLAY_HEIGHT - 1);
    u8g2.drawPixel(VFD_DISPLAY_WIDTH - 1, VFD_DISPLAY_HEIGHT - 1);

    u8g2.drawStr(120, 4, "Test 1: Corner Pixels");
    u8g2.sendBuffer();
}

// 测试2：边框和线条
void testFramesAndLines() {
    u8g2.clearBuffer();
    // 绘制边框
    u8g2.drawFrame(0, 0, VFD_DISPLAY_WIDTH, VFD_DISPLAY_HEIGHT);
    // 绘制水平线和垂直线
    u8g2.drawHLine(40, 8, 200);
    u8g2.drawVLine(140, 0, 16);

    u8g2.drawStr(150, 4, "Test 2: Frames & Lines");
    u8g2.sendBuffer();
}

// 测试3：矩形和填充矩形
void testRectangles() {
    u8g2.clearBuffer();
    // 绘制矩形
    u8g2.drawFrame(10, 2, 40, 12);
    // 绘制填充矩形
    u8g2.drawBox(60, 2, 40, 12);
    // 绘制圆角矩形
    u8g2.drawRFrame(110, 2, 40, 12, 3);
    // 绘制填充圆角矩形
    u8g2.drawRBox(160, 2, 40, 12, 3);

    u8g2.drawStr(210, 4, "Test 3: Rectangles");
    u8g2.sendBuffer();
}

// 测试4：圆形和椭圆
void testCirclesAndEllipses() {
    u8g2.clearBuffer();
    // 绘制圆形
    u8g2.drawCircle(20, 8, 7);
    // 绘制填充圆形
    u8g2.drawDisc(50, 8, 7);
    // 绘制椭圆
    u8g2.drawEllipse(90, 8, 15, 7);
    // 绘制填充椭圆
    u8g2.drawFilledEllipse(140, 8, 15, 7);

    u8g2.drawStr(180, 4, "Test 4: Circles");
    u8g2.sendBuffer();
}

// 测试5：文本显示
void testText() {
    u8g2.clearBuffer();
    // 设置不同字体
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(10, 0, "MN28016A VFD Display");

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(10, 9, "Test 5: Different fonts and text");
    u8g2.sendBuffer();
}

// 测试6：滚动文本
void testScrollingText() {
    static int scrollPosition = 0;
    const char* message =
        "This is a scrolling text demo for MN28016A VFD display - ";
    int messageWidth = u8g2.getStrWidth(message);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(280 - scrollPosition, 4, message);
    // 如果文本滚动到左边界外，重新开始
    if (scrollPosition > messageWidth + 280) {
        scrollPosition = 0;
    } else {
        scrollPosition += 2;  // 滚动速度
    }

    u8g2.drawStr(10, 0, "Test 6: Scrolling");
    u8g2.sendBuffer();
}

// 测试7：动画效果
void testAnimation() {
    static int frame = 0;
    const int maxFrames = 16;

    u8g2.clearBuffer();

    // 简单的波浪动画
    for (int x = 0; x < VFD_DISPLAY_WIDTH; x += 4) {
        int y = 8 + sin((x + frame) * 0.2) * 6;
        u8g2.drawPixel(x, y);
    }

    frame = (frame + 1) % maxFrames;

    u8g2.drawStr(10, 0, "Test 7: Animation");
    u8g2.sendBuffer();
}

// 测试8：亮度调整
void testBrightness() {
    static int brightness = 255;
    static int direction = -5;  // 每次减少5

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    // 调整亮度
    brightness += direction;
    if (brightness <= 50) {
        direction = 5;  // 达到最小值后开始增加
    } else if (brightness >= 255) {
        direction = -5;  // 达到最大值后开始减少
    }

    // 设置新的亮度值
    u8g2.setContrast(brightness);

    // 显示当前亮度值
    char brightnessStr[30];
    sprintf(brightnessStr, "Brightness: %d", brightness);
    u8g2.drawStr(10, 4, "Test 8: Brightness Control");
    u8g2.drawStr(150, 4, brightnessStr);

    u8g2.sendBuffer();
}

void loop() {
    unsigned long currentTime = millis();

    // 定时切换测试模式
    if (currentTime - lastModeChange >= modeChangeInterval) {
        currentTestMode = (currentTestMode + 1) % TEST_MODE_COUNT;
        lastModeChange = currentTime;
    }

    // 根据当前测试模式执行相应的测试函数
    switch (currentTestMode) {
        case 0:
            testCornerPixels();
            break;
        case 1:
            testFramesAndLines();
            break;
        case 2:
            testRectangles();
            break;
        case 3:
            testCirclesAndEllipses();
            break;
        case 4:
            testText();
            break;
        case 5:
            testScrollingText();
            break;
        case 6:
            testAnimation();
            break;
        case 7:
            testBrightness();
            break;
    }

    delay(50);  // 适当的延迟以控制刷新率
}