#include "constants.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace HybridAStar {
namespace Constants {

bool coutDEBUG = false;
bool manual = true;
bool visualization = false;
bool visualization2D = false;
bool reverse = true;
bool dubinsShot = true;
bool dubins = false;
bool dubinsLookup = false;
bool twoD = true;
bool smoothing = true;
bool publishSearchCosts = false;

int iterations = 300000;
double bloating = 0;
double vehicleWidth = 1.75;
double vehicleLength = 2.65;
double width = 1.75;
double length = 2.65;
float r = 6;
float primitiveHeadingChangeDeg = 6.75f;
float primitiveHeadingChangeRad = static_cast<float>(6.75 * M_PI / 180.0);
int headings = 72;
float deltaHeadingDeg = 5;
float deltaHeadingRad = static_cast<float>(2 * M_PI / 72);
float deltaHeadingNegRad = static_cast<float>(2 * M_PI - 2 * M_PI / 72);
float cellSize = 1;
float tieBreaker = 0.01f;

float factor2D = static_cast<float>(std::sqrt(5) / std::sqrt(2) + 1);
float penaltyTurning = 1.05f;
float penaltyReversing = 2.0f;
float penaltyCOD = 2.0f;
float dubinsShotDistance = 100;
float dubinsStepSize = 1;
int smootherIterations = 80;

int dubinsWidth = 15;
int dubinsArea = 15 * 15;
int bbSize = 7;
int positionResolution = 10;
int positions = 100;
float minRoadWidth = 2;

namespace {

std::string trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::string stripQuotes(const std::string& value) {
  if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::unordered_map<std::string, std::string> loadYamlFields(const std::string& yaml_file) {
  std::ifstream file(yaml_file);
  if (!file) {
    throw std::runtime_error("failed to open config yaml: " + yaml_file);
  }

  std::unordered_map<std::string, std::string> fields;
  std::string line;
  while (std::getline(file, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }

    const auto separator = line.find(':');
    if (separator == std::string::npos) {
      continue;
    }

    const auto key = trim(line.substr(0, separator));
    const auto value = trim(line.substr(separator + 1));
    if (!key.empty() && !value.empty()) {
      fields[key] = stripQuotes(value);
    }
  }
  return fields;
}

bool readBool(const std::string& value) {
  const auto normalized = toLower(trim(value));
  if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
    return false;
  }
  throw std::runtime_error("invalid boolean value: " + value);
}

bool findValue(
    const std::unordered_map<std::string, std::string>& fields,
    std::initializer_list<const char*> keys,
    std::string& value) {
  for (const auto* key : keys) {
    const auto it = fields.find(key);
    if (it != fields.end()) {
      value = it->second;
      return true;
    }
  }
  return false;
}

void applyBool(
    const std::unordered_map<std::string, std::string>& fields,
    std::initializer_list<const char*> keys,
    bool& target) {
  std::string value;
  if (findValue(fields, keys, value)) {
    target = readBool(value);
  }
}

void applyInt(
    const std::unordered_map<std::string, std::string>& fields,
    std::initializer_list<const char*> keys,
    int& target) {
  std::string value;
  if (findValue(fields, keys, value)) {
    target = std::stoi(value);
  }
}

void applyFloat(
    const std::unordered_map<std::string, std::string>& fields,
    std::initializer_list<const char*> keys,
    float& target) {
  std::string value;
  if (findValue(fields, keys, value)) {
    target = std::stof(value);
  }
}

void applyDouble(
    const std::unordered_map<std::string, std::string>& fields,
    std::initializer_list<const char*> keys,
    double& target) {
  std::string value;
  if (findValue(fields, keys, value)) {
    target = std::stod(value);
  }
}

}  // namespace

void updateDerivedConstants() {
  headings = std::max(1, headings);
  cellSize = std::max(0.001f, cellSize);
  r = std::max(0.001f, r);
  primitiveHeadingChangeDeg = std::max(0.001f, primitiveHeadingChangeDeg);
  dubinsStepSize = std::max(0.001f, dubinsStepSize);
  smootherIterations = std::max(0, smootherIterations);
  dubinsWidth = std::max(1, dubinsWidth);
  positionResolution = std::max(1, positionResolution);

  width = vehicleWidth + 2 * bloating;
  length = vehicleLength + 2 * bloating;
  deltaHeadingDeg = 360.f / static_cast<float>(headings);
  deltaHeadingRad = static_cast<float>(2 * M_PI / static_cast<float>(headings));
  deltaHeadingNegRad = static_cast<float>(2 * M_PI - deltaHeadingRad);
  primitiveHeadingChangeRad = static_cast<float>(primitiveHeadingChangeDeg * M_PI / 180.0);
  dubinsArea = dubinsWidth * dubinsWidth;
  bbSize = static_cast<int>(std::ceil((std::sqrt(width * width + length * length) + 4) / cellSize));
  positions = positionResolution * positionResolution;

  if (!dubins) {
    dubinsLookup = false;
  }
}

std::size_t collisionLookupSize() {
  return static_cast<std::size_t>(headings) * static_cast<std::size_t>(positions);
}

std::size_t dubinsLookupSize() {
  const int widthCells = std::max(1, static_cast<int>(dubinsWidth / cellSize));
  return static_cast<std::size_t>(widthCells) *
         static_cast<std::size_t>(widthCells) *
         static_cast<std::size_t>(headings) *
         static_cast<std::size_t>(headings);
}

float metersToCells(float meters) {
  return meters / cellSize;
}

float turningRadiusCells() {
  return metersToCells(r);
}

float dubinsShotDistanceCells() {
  return metersToCells(dubinsShotDistance);
}

float dubinsStepSizeCells() {
  return metersToCells(dubinsStepSize);
}

float primitiveStepLengthCells() {
  return turningRadiusCells() * primitiveHeadingChangeRad;
}

bool loadFromYaml(const std::string& yaml_file, std::string* error) {
  try {
    const auto fields = loadYamlFields(yaml_file);

    applyBool(fields, {"coutDEBUG", "cout_debug"}, coutDEBUG);
    applyBool(fields, {"manual"}, manual);
    applyBool(fields, {"visualization"}, visualization);
    applyBool(fields, {"visualization2D", "visualization_2d"}, visualization2D);
    applyBool(fields, {"reverse"}, reverse);
    applyBool(fields, {"dubinsShot", "dubins_shot"}, dubinsShot);
    applyBool(fields, {"dubins"}, dubins);
    applyBool(fields, {"dubinsLookup", "dubins_lookup"}, dubinsLookup);
    applyBool(fields, {"twoD", "two_d"}, twoD);
    applyBool(fields, {"smoothing", "smoothing_enabled"}, smoothing);
    applyBool(fields, {"publishSearchCosts", "publish_search_costs"}, publishSearchCosts);

    applyInt(fields, {"iterations"}, iterations);
    applyDouble(fields, {"bloating"}, bloating);
    applyDouble(fields, {"vehicleWidth", "vehicle_width"}, vehicleWidth);
    applyDouble(fields, {"vehicleLength", "vehicle_length"}, vehicleLength);
    applyFloat(fields, {"r", "turning_radius"}, r);
    applyFloat(
      fields,
      {"primitiveHeadingChangeDeg", "primitive_heading_change_deg"},
      primitiveHeadingChangeDeg);
    applyInt(fields, {"headings"}, headings);
    applyFloat(fields, {"cellSize", "cell_size"}, cellSize);
    applyFloat(fields, {"tieBreaker", "tie_breaker"}, tieBreaker);

    applyFloat(fields, {"factor2D", "factor_2d"}, factor2D);
    applyFloat(fields, {"penaltyTurning", "penalty_turning"}, penaltyTurning);
    applyFloat(fields, {"penaltyReversing", "penalty_reversing"}, penaltyReversing);
    applyFloat(fields, {"penaltyCOD", "penalty_cod"}, penaltyCOD);
    applyFloat(fields, {"dubinsShotDistance", "dubins_shot_distance"}, dubinsShotDistance);
    applyFloat(fields, {"dubinsStepSize", "dubins_step_size"}, dubinsStepSize);
    applyInt(fields, {"smootherIterations", "smoother_iterations"}, smootherIterations);

    applyInt(fields, {"dubinsWidth", "dubins_width"}, dubinsWidth);
    applyInt(fields, {"positionResolution", "position_resolution"}, positionResolution);
    applyFloat(fields, {"minRoadWidth", "min_road_width"}, minRoadWidth);

    updateDerivedConstants();
    return true;
  } catch (const std::exception& ex) {
    if (error) {
      *error = ex.what();
    }
    return false;
  }
}

}  // namespace Constants
}  // namespace HybridAStar
