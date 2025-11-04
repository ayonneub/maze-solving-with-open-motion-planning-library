#include "lodepng.h"
#include <cassert>
#include <vector>

class image_parser
{
public:
    int getWidth() const
    {
        return width_;
    }

    int getHeight() const
    {
        return height_;
    }

    image_parser(const std::string &imageFilePath)
    {
        unsigned error = lodepng::decode(imageArray_, width_, height_, imageFilePath, LCT_RGB);
        assert(!error && "Error: Unable to load the PNG file.");
    }

    // This function will now return an RGB triplet
    std::vector<uint8_t> getPixelValue(int y, int x)
    {
        int index = 3 * (y * width_ + x);
        return {imageArray_[index], imageArray_[index + 1], imageArray_[index + 2]};
    }

    // This function will now update an RGB triplet
    void updatePixel(int y, int x, const std::vector<uint8_t> &rgb)
    {
        int index = 3 * (y * width_ + x);
        imageArray_[index] = rgb[0];
        imageArray_[index + 1] = rgb[1];
        imageArray_[index + 2] = rgb[2];
    }

    void drawLine(int x0, int y0, int x1, int y1)
    {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;

        while (true)
        {
            setPixelAreaToRed(y0, x0); // Coloring the pixel and its surrounding area

            if (x0 == x1 && y0 == y1)
                break;
            e2 = err;
            if (e2 > -dx)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dy)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    // Function to set the pixel to red
    void setPixelAreaToRed(int y, int x, int radius = 1)
    {
        for (int i = -radius; i <= radius; i++)
        {
            for (int j = -radius; j <= radius; j++)
            {
                std::vector<uint8_t> redPixel = {255, 0, 0}; // Assuming RGBA format
                updatePixel(y + i, x + j, redPixel);
            }
        }
    }

    void writeImage(const std::string &outputFileName)
    {
        unsigned error = lodepng::encode(outputFileName, imageArray_, width_, height_, LCT_RGB);
        assert(!error && "Error: Unable to save the PNG file.");
    }

private:
    std::vector<uint8_t> imageArray_;
    unsigned width_, height_;
};
