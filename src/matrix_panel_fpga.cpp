#include "matrix_panel_fpga.hpp"



void MatrixPanel_FPGA_SPI::drawPixel(int16_t x, int16_t y, uint16_t color) {

};
void MatrixPanel_FPGA_SPI::fillScreen(uint16_t color) {

};

void MatrixPanel_FPGA_SPI::fillRectDMA(int16_t x_coord, int16_t y_coord, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b) {

};

// bool MatrixPanel_I2S_DMA::begin(const HUB75_I2S_CFG &cfg)
// {
//   if (initialized)
//     return true;

//   if (!setCfg(cfg))
//     return false;

//   return begin();
// }

// /**
//  * @brief - update DMA buff drawing a rectangular at specified coordinates
//  * this works much faster than multiple consecutive per-pixel calls to updateMatrixDMABuffer()
//  * @param int16_t x, int16_t y - coordinates of a top-left corner
//  * @param int16_t w, int16_t h - width and height of a rectangular, min is 1 px
//  * @param uint8_t r - RGB888 colour
//  * @param uint8_t g - RGB888 colour
//  * @param uint8_t b - RGB888 colour
//  */
// void MatrixPanel_I2S_DMA::fillRectDMA(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b)
// {

//   // h-lines are >2 times faster than v-lines
//   // so will use it only for tall rects with h >2w
//   if (h > 2 * w)
//   {
//     // draw using v-lines
//     do
//     {
//       --w;
//       vlineDMA(x + w, y, h, r, g, b);
//     } while (w);
//   }
//   else
//   {
//     // draw using h-lines
//     do
//     {
//       --h;
//       hlineDMA(x, y + h, w, r, g, b);
//     } while (h);
//   }
// }