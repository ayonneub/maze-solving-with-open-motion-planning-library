#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include "image_parser.h"

int main()
{
    const char *imageFilePath = "../test/image.png"; // Change to your image file path

    image_parser maze(imageFilePath);

    /*
    David: Modified to take advantage of row-major order storage by parsing, row-by-row, which is more effient in terms of memory access.
    added getwidth and getheight, takes advantage of lodepng..
    */
    // // Example: Access pixel value at row 10, column 20
    for (int y = 0; y < maze.getHeight(); y++)
    {
        for (int x = 0; x < maze.getWidth(); x++)
        {
            std::vector<uint8_t> pixelValue = maze.getPixelValue(y, x);
            printf("Pixel RGB values at (%d, %d): R:%d, G:%d, B:%d \n", x, y, pixelValue[0], pixelValue[1], pixelValue[2]);

            // For demonstration, this line just updates the pixel with its current value.
            // But in real applications, you'd perform some operations on the pixel values.
            maze.updatePixel(y, x, pixelValue);
        }
    }

    // Specify the output PNG file name
    const char *outputFileName = "output_image.png";
    maze.writeImage(outputFileName);

    return 0;
}
