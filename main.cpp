#include "src/camera_realsense.hpp"

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/common/version_metadata.hpp>

int main(int argc, char **argv) {
  const std::string usage = "usage: camera_realsense /path/to/unix/socket";

  if (argc < 2) {
    std::cout << "ERROR: insufficient arguments\n";
    std::cout << usage << "\n";
    return EXIT_FAILURE;
  }

  viam::sdk::Instance instance;
  VIAM_SDK_LOG(info) << "viam-cpp-sdk-version: " << viam::sdk::sdk_version();
  VIAM_SDK_LOG(info) << "intel realsense SDK version: " << RS2_API_FULL_VERSION_STR;
  return viam::realsense::serve(argc, argv);
}
