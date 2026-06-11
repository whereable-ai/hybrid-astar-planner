#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

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

std::string directoryName(const std::string& path) {
  const auto slash = path.find_last_of("/\\");
  if (slash == std::string::npos) {
    return ".";
  }
  return path.substr(0, slash);
}

bool isAbsolutePath(const std::string& path) {
  return !path.empty() && path.front() == '/';
}

std::string resolvePath(const std::string& yaml_file, const std::string& image_file) {
  if (isAbsolutePath(image_file)) {
    return image_file;
  }
  return directoryName(yaml_file) + "/" + image_file;
}

bool fileExists(const std::string& path) {
  std::ifstream file(path);
  return static_cast<bool>(file);
}

std::string stripExtension(const std::string& path) {
  const auto slash = path.find_last_of("/\\");
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
    return path;
  }
  return path.substr(0, dot);
}

std::string resolveMapStem(const std::string& yaml_file, const std::string& image_name) {
  return stripExtension(resolvePath(yaml_file, image_name));
}

std::string resolveImagePath(const std::string& map_stem, const std::string& image_name) {
  if (stripExtension(image_name) != image_name) {
    return isAbsolutePath(image_name) ? image_name : map_stem + image_name.substr(stripExtension(image_name).size());
  }

  const std::vector<std::string> candidates = {
    map_stem + ".png",
    map_stem + ".pgm",
  };

  for (const auto& candidate : candidates) {
    if (fileExists(candidate)) {
      return candidate;
    }
  }

  return candidates.front();
}

std::unordered_map<std::string, std::string> loadYamlFields(const std::string& yaml_file) {
  std::ifstream file(yaml_file);
  if (!file) {
    throw std::runtime_error("Failed to open map yaml: " + yaml_file);
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

void mergeYamlFields(
    std::unordered_map<std::string, std::string>& base,
    const std::unordered_map<std::string, std::string>& overrides) {
  for (const auto& field : overrides) {
    base[field.first] = field.second;
  }
}

std::vector<double> parseOrigin(const std::string& value) {
  std::string normalized = value;
  normalized.erase(std::remove(normalized.begin(), normalized.end(), '['), normalized.end());
  normalized.erase(std::remove(normalized.begin(), normalized.end(), ']'), normalized.end());
  std::replace(normalized.begin(), normalized.end(), ',', ' ');

  std::istringstream stream(normalized);
  std::vector<double> origin;
  double number = 0.0;
  while (stream >> number) {
    origin.push_back(number);
  }

  if (origin.size() < 3) {
    throw std::runtime_error("origin must contain [x, y, yaw]");
  }
  return origin;
}

template <typename T>
T getRequired(const std::unordered_map<std::string, std::string>& fields, const std::string& key);

template <>
std::string getRequired<std::string>(
    const std::unordered_map<std::string, std::string>& fields,
    const std::string& key) {
  const auto it = fields.find(key);
  if (it == fields.end()) {
    throw std::runtime_error("Missing required map yaml field: " + key);
  }
  return it->second;
}

std::string getString(
    const std::unordered_map<std::string, std::string>& fields,
    const std::string& key,
    const std::string& fallback) {
  const auto it = fields.find(key);
  if (it == fields.end()) {
    return fallback;
  }
  return it->second;
}

double getDouble(
    const std::unordered_map<std::string, std::string>& fields,
    const std::string& key,
    double fallback) {
  const auto it = fields.find(key);
  if (it == fields.end()) {
    return fallback;
  }
  return std::stod(it->second);
}

int getInt(
    const std::unordered_map<std::string, std::string>& fields,
    const std::string& key,
    int fallback) {
  const auto it = fields.find(key);
  if (it == fields.end()) {
    return fallback;
  }
  return std::stoi(it->second);
}

std::vector<std::int8_t> downsampleOccupancy(
    const std::vector<std::int8_t>& source,
    const int width,
    const int height,
    const int scale,
    std::uint32_t* scaled_width,
    std::uint32_t* scaled_height) {
  *scaled_width = static_cast<std::uint32_t>((width + scale - 1) / scale);
  *scaled_height = static_cast<std::uint32_t>((height + scale - 1) / scale);

  std::vector<std::int8_t> scaled(static_cast<std::size_t>(*scaled_width * *scaled_height), 0);
  for (std::uint32_t y = 0; y < *scaled_height; ++y) {
    for (std::uint32_t x = 0; x < *scaled_width; ++x) {
      bool has_occupied = false;
      bool has_unknown = false;

      const int y_begin = static_cast<int>(y) * scale;
      const int y_end = std::min(y_begin + scale, height);
      const int x_begin = static_cast<int>(x) * scale;
      const int x_end = std::min(x_begin + scale, width);

      for (int source_y = y_begin; source_y < y_end; ++source_y) {
        for (int source_x = x_begin; source_x < x_end; ++source_x) {
          const auto value = source[static_cast<std::size_t>(source_y * width + source_x)];
          has_occupied = has_occupied || value == 100;
          has_unknown = has_unknown || value < 0;
        }
      }

      const auto index = static_cast<std::size_t>(y * *scaled_width + x);
      scaled[index] = has_occupied ? 100 : (has_unknown ? -1 : 0);
    }
  }

  return scaled;
}

nav_msgs::msg::OccupancyGrid loadYamlMap(const std::string& yaml_file, const int resolution_scale) {
  if (resolution_scale < 1) {
    throw std::runtime_error("resolution_scale must be >= 1");
  }

  auto fields = loadYamlFields(yaml_file);
  const auto image_name = getRequired<std::string>(fields, "image");
  const auto map_stem = resolveMapStem(yaml_file, image_name);
  const auto image_path = resolveImagePath(map_stem, image_name);
  const auto map_config_path = map_stem + ".yaml";

  if (fileExists(map_config_path) && map_config_path != yaml_file) {
    mergeYamlFields(fields, loadYamlFields(map_config_path));
  }

  const auto resolution = getDouble(fields, "resolution", 0.1);
  const auto occupied_thresh = getDouble(fields, "occupied_thresh", 0.1);
  const auto free_thresh = getDouble(fields, "free_thresh", 0.05);
  const auto negate = getInt(fields, "negate", 0);
  const auto origin = parseOrigin(getString(fields, "origin", "[0.0, 0.0, 0.0]"));

  cv::Mat image = cv::imread(image_path, cv::IMREAD_UNCHANGED);
  if (image.empty()) {
    throw std::runtime_error("Failed to open map image: " + image_path);
  }

  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  } else {
    throw std::runtime_error("Unsupported map image channel count: " + std::to_string(image.channels()));
  }

  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.origin.position.x = origin[0];
  map.info.origin.position.y = origin[1];
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.z = std::sin(origin[2] * 0.5);
  map.info.origin.orientation.w = std::cos(origin[2] * 0.5);

  std::vector<std::int8_t> source_data(static_cast<std::size_t>(gray.cols * gray.rows));

  for (int y = 0; y < gray.rows; ++y) {
    for (int x = 0; x < gray.cols; ++x) {
      const auto pixel = static_cast<double>(gray.at<std::uint8_t>(gray.rows - 1 - y, x));
      const auto color = pixel / 255.0;
      const auto occupancy = negate ? color : 1.0 - color;
      const auto index = static_cast<std::size_t>(y * gray.cols + x);

      if (occupancy >= occupied_thresh) {
        source_data[index] = 100;
      } else if (occupancy <= free_thresh) {
        source_data[index] = 0;
      } else {
        source_data[index] = -1;
      }
    }
  }

  if (resolution_scale == 1) {
    map.info.resolution = static_cast<float>(resolution);
    map.info.width = static_cast<std::uint32_t>(gray.cols);
    map.info.height = static_cast<std::uint32_t>(gray.rows);
    map.data = std::move(source_data);
  } else {
    map.info.resolution = static_cast<float>(resolution * resolution_scale);
    map.data = downsampleOccupancy(
        source_data,
        gray.cols,
        gray.rows,
        resolution_scale,
        &map.info.width,
        &map.info.height);
  }

  return map;
}

}  // namespace

class YamlMapPublisher : public rclcpp::Node {
 public:
  YamlMapPublisher() : Node("yaml_map_publisher") {
    declare_parameter<std::string>("yaml_file", "");
    declare_parameter<std::string>("config_file", "");
    declare_parameter<std::string>("topic", "/map");
    declare_parameter<int>("resolution_scale", 0);

    const auto yaml_file = get_parameter("yaml_file").as_string();
    const auto config_file = get_parameter("config_file").as_string();
    const auto topic = get_parameter("topic").as_string();
    const auto resolution_scale_param = get_parameter("resolution_scale").as_int();
    if (yaml_file.empty()) {
      throw std::runtime_error("yaml_file parameter is required");
    }

    int resolution_scale = 1;
    if (!config_file.empty()) {
      const auto config_fields = loadYamlFields(config_file);
      resolution_scale = getInt(
          config_fields,
          "map_resolution_scale",
          getInt(config_fields, "resolution_scale", resolution_scale));
    }
    if (resolution_scale_param > 0) {
      resolution_scale = resolution_scale_param;
    }

    map_ = loadYamlMap(yaml_file, resolution_scale);

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(topic, qos);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      map_.header.stamp = now();
      publisher_->publish(map_);
      timer_->cancel();
    });

    RCLCPP_INFO(
        get_logger(),
        "Publishing static %ux%u map at %.3f m/cell on %s",
        map_.info.width,
        map_.info.height,
        map_.info.resolution,
        topic.c_str());
  }

 private:
  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YamlMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
