#ifndef CONSTANTS
#define CONSTANTS
/*!
   \file constants.h
   \brief This is a collection of constants that are used throughout the project.
   \todo All constants need to be checked and documented
*/

////###################################################
////                                               INFO
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

/*!
    \brief The namespace that wraps the entire project
    \namespace HybridAStar
*/

namespace HybridAStar {
/*!
    \brief The namespace that wraps constants.h
    \namespace Constants
*/
namespace Constants {
// _________________
// CONFIG FLAGS

/// A flag for additional debugging output via `std::cout`
extern bool coutDEBUG;
/// A flag for the mode (true = manual; false = dynamic). Manual for static map or dynamic for dynamic map.
extern bool manual;
/// A flag for the visualization of 3D nodes (true = on; false = off)
extern bool visualization;
/// A flag for the visualization of 2D nodes (true = on; false = off)
extern bool visualization2D;
/// A flag to toggle reversing (true = on; false = off)
extern bool reverse;
/// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
extern bool dubinsShot;
/// A flag to toggle the Dubin's heuristic, this should be false, if reversing is enabled (true = on; false = off)
extern bool dubins;
/*!
   \var static const bool dubinsLookup
   \brief A flag to toggle the Dubin's heuristic via lookup, potentially speeding up the search by a lot
   \todo not yet functional
*/
extern bool dubinsLookup;
/// A flag to toggle the 2D heuristic (true = on; false = off)
extern bool twoD;
/// A flag to toggle path smoothing
extern bool smoothing;
/// A flag to publish expensive search cost marker arrays
extern bool publishSearchCosts;

// _________________
// GENERAL CONSTANTS

/// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
extern int iterations;
/// [m] --- Uniformly adds a padding around the vehicle
extern double bloating;
/// [m] --- The uninflated width of the vehicle
extern double vehicleWidth;
/// [m] --- The uninflated length of the vehicle
extern double vehicleLength;
/// [m] --- The width of the vehicle
extern double width;
/// [m] --- The length of the vehicle
extern double length;
/// [m] --- The minimum turning radius of the vehicle
extern float r;
/// [°] --- Heading change per hybrid A* motion primitive
extern float primitiveHeadingChangeDeg;
/// [rad] --- Heading change per hybrid A* motion primitive
extern float primitiveHeadingChangeRad;
/// [m] --- The number of discretizations in heading
extern int headings;
/// [°] --- The discretization value of the heading (goal condition)
extern float deltaHeadingDeg;
/// [c*M_PI] --- The discretization value of heading (goal condition)
extern float deltaHeadingRad;
/// [c*M_PI] --- The heading part of the goal condition
extern float deltaHeadingNegRad;
/// [m] --- The cell size of the 2D grid of the world
extern float cellSize;
/*!
  \brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell


  As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe that the algorithm would prefer the predecessor rather than the successor.
  This would lead to the fact that the successor would never be placed and the the one cell could only expand one node. The tieBreaker artificially increases the cost of the predecessor
  to allow the successor being placed in the same cell.
*/
extern float tieBreaker;

// ___________________
// HEURISTIC CONSTANTS

/// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
extern float factor2D;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
extern float penaltyTurning;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
extern float penaltyReversing;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
extern float penaltyCOD;
/// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
extern float dubinsShotDistance;
/// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
extern float dubinsStepSize;
/// [#] --- Maximum number of smoothing iterations
extern int smootherIterations;


// ______________________
// DUBINS LOOKUP SPECIFIC

/// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
extern int dubinsWidth;
/// [m] --- The area of the lookup for the analytical solution (Dubin's shot)
extern int dubinsArea;


// _________________________
// COLLISION LOOKUP SPECIFIC

/// [m] -- The bounding box size length and width to precompute all possible headings
extern int bbSize;
/// [#] --- The sqrt of the number of discrete positions per cell
extern int positionResolution;
/// [#] --- The number of discrete positions per cell
extern int positions;
/// A structure describing the relative position of the occupied cell based on the center of the vehicle
struct relPos {
  /// the x position relative to the center
  int x;
  /// the y position relative to the center
  int y;
};
/// A structure capturing the lookup for each theta configuration
struct config {
  /// the number of cells occupied by this configuration of the vehicle
  int length;
  /*!
     \brief The occupied cells for this configuration
  */
  std::vector<relPos> pos;
};

// _________________
// SMOOTHER SPECIFIC
/// [m] --- The minimum width of a safe road for the vehicle at hand
extern float minRoadWidth;

// ____________________________________________
// COLOR DEFINITIONS FOR VISUALIZATION PURPOSES
/// A structure to express colors in RGB values
struct color {
  /// the red portion of the color
  float red;
  /// the green portion of the color
  float green;
  /// the blue portion of the color
  float blue;
};
/// A definition for a color used for visualization
static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
/// A definition for a color used for visualization
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
/// A definition for a color used for visualization
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
/// A definition for a color used for visualization
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
/// A definition for a color used for visualization
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};

bool loadFromYaml(const std::string& yaml_file, std::string* error = nullptr);
void updateDerivedConstants();
std::size_t collisionLookupSize();
std::size_t dubinsLookupSize();
float metersToCells(float meters);
float turningRadiusCells();
float dubinsShotDistanceCells();
float dubinsStepSizeCells();
float primitiveStepLengthCells();
}
}

#endif // CONSTANTS
