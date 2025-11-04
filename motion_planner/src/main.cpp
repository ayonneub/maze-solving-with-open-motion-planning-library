#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <cstdint>

#include "image_parser.h"

// OMPL
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Utility helpers


// Clamp integer into [lo, hi]
static inline int clampi(int v, int lo, int hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Brightness of RGB triplet (0–255)
static inline int brightness(const std::vector<uint8_t> &rgb)
{
    return (int)std::max({rgb[0], rgb[1], rgb[2]});
}

// Draw a 1px red Bresenham line (x,y are column,row)
static void drawLineRed(image_parser &img, int x0, int y0, int x1, int y1)
{
    const int W = img.getWidth();
    const int H = img.getHeight();
    auto setRed = [&](int x, int y)
    {
        if (x < 0 || x >= W || y < 0 || y >= H)
            return;
        img.updatePixel(y, x, std::vector<uint8_t>{255, 0, 0});
    };

    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    int x = x0, y = y0;
    while (true)
    {
        setRed(x, y);
        if (x == x1 && y == y1)
            break;
        int e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y += sy;
        }
    }
}

// Main
int main()
{
    //Load the maze image
    const char *imagePath = "../test/image.png"; 
    image_parser maze(imagePath);

    const int width = maze.getWidth();
    const int height = maze.getHeight();
    if (width <= 0 || height <= 0)
    {
        std::cerr << "Failed to load image or invalid dimensions.\n";
        return 1;
    }

    // Start and goal positions
    std::array<double, 2> start = {150.0, 0.0};
    std::array<double, 2> goal = {150.0, static_cast<double>(height - 1)};

    // Define 2D state space with image bounds
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, 0.0);
    bounds.setHigh(0, static_cast<double>(width - 1));
    bounds.setLow(1, 0.0);
    bounds.setHigh(1, static_cast<double>(height - 1));
    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    //Collision checking using image brightness
    const int FREE_THRESH = 200; 
    ss.setStateValidityChecker([&](const ob::State *state) -> bool {
        const auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
        int x = static_cast<int>(std::round(rv->values[0]));
        int y = static_cast<int>(std::round(rv->values[1]));

        if (x < 0 || x >= width || y < 0 || y >= height)
            return false;

        auto rgb = maze.getPixelValue(y, x);
        return brightness(rgb) >= FREE_THRESH;
    });

    // Check 1 pixel per step
    ss.getSpaceInformation()->setStateValidityCheckingResolution(
        1.0 / (double)std::max(width, height));

    // Planner (RRTConnect)
    auto planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    //Set start & goal
    ob::ScopedState<> s(space), g(space);
    s->as<ob::RealVectorStateSpace::StateType>()->values[0] = start[0];
    s->as<ob::RealVectorStateSpace::StateType>()->values[1] = start[1];
    g->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
    g->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
    ss.setStartAndGoalStates(s, g);

    std::cout << "Planning from (" << start[0] << ", " << start[1] << ") to ("
              << goal[0] << ", " << goal[1] << ") on " << width << "x" << height << " image.\n";

    //Solve for up to 5 seconds
    ob::PlannerStatus solved = ss.solve(5.0);
    if (!solved)
    {
        std::cerr << "No solution found.\n";
        return 2;
    }

    //Simplify and extract path
    ss.simplifySolution();
    og::PathGeometric &path = ss.getSolutionPath();
    std::cout << "Found path with " << path.getStateCount() << " states.\n";

    auto &states = path.getStates();
    for (size_t i = 1; i < states.size(); ++i)
    {
        const auto *a = states[i - 1]->as<ob::RealVectorStateSpace::StateType>();
        const auto *b = states[i]->as<ob::RealVectorStateSpace::StateType>();
        int x0 = clampi(static_cast<int>(std::round(a->values[0])), 0, width - 1);
        int y0 = clampi(static_cast<int>(std::round(a->values[1])), 0, height - 1);
        int x1 = clampi(static_cast<int>(std::round(b->values[0])), 0, width - 1);
        int y1 = clampi(static_cast<int>(std::round(b->values[1])), 0, height - 1);
        drawLineRed(maze, x0, y0, x1, y1);
    }

    //Save result
    const char *outPath = "solved.png";
    maze.writeImage(outPath);
    std::cout << "Wrote " << outPath << "\n";

    return 0;
}
