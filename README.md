# Maze Path Planning using OMPL and Image Parsing

### Ayon Dey

## Overview

This project demonstrates how to solve a maze environment using the OMPL (Open
Motion Planning Library) framework. A binary maze image is parsed to extract free
and occupied spaces. An RRTConnect-based path planner computes a collision-free route
between the start and goal positions. Finally, the computed trajectory is drawn as a thin
red line on the maze image for visualization.

## Attached Screenshots
<img width="812" height="798" alt="1" src="https://github.com/user-attachments/assets/66db997d-1156-4358-9c7c-e472f6794568" />

## Brief Explanation of the Solution

The solution combines image-based environment parsing with sampling-based motion
planning. An imageparser class reads a PNG maze and provides pixel-level access
through getPixelValue() and updatePixel(). OMPL’s RealVectorStateSpace(2)
defines the search space matching the maze width and height. The planner uses RRTConnect
to efficiently search for a path from the start at (150, 0) to the goal at (150, 299).
After the planner finds a solution, the states along the path are converted into pixel
coordinates. A simple Bresenham line algorithm draws red pixels (RGB = [255, 0, 0])
along each segment, and the final annotated image is saved as solved.png.

## How to Parse a Maze Environment?

The maze is treated as a grayscale or RGB image, where each pixel corresponds to a
small region of the environment. The imageparser class loads the PNG file using the
lodepng library and stores pixel values in a row-major array. Each pixel’s RGB intensity


is used to decide whether it represents an obstacle (dark color) or free space (light color).
The image’s dimensions (getWidth() and getHeight()) define the spatial bounds of the
OMPL planning space.

## How to Check Free Space and Collision?

Collision checking is handled by a custom isStateValid() function used by OMPL. For
any sampled state (x, y), the corresponding pixel in the maze is queried. If its brightness
value is greater than a certain threshold (I used 200, because 100 is not that efficient), it
is considered free; otherwise, it is treated as an obstacle. This method allows the planner
to navigate only through white regions and avoid black walls in the maze image.

## How to Visualize the Solution in the Output Image?

After OMPL finds a feasible path, each consecutive pair of path states is connected by a
red line on the original image using a Bresenham line-drawing algorithm. This ensures
the visualized path follows the discrete pixel grid without gaps or overlaps. The updated
image is then written to disk as solved.png, allowing easy verification of the computed
trajectory.

## Conclusion

This approach demonstrates how classical motion planning concepts can be applied to
pixel-based maze environments. By integrating OMPL with image parsing, it becomes
possible to visualize motion planning algorithms directly over real or simulated maps,
bridging the gap between abstract planning and visual interpretation.






