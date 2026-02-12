/// @file integration_tests.cpp
/// @brief Integration tests using rs2::software_device to exercise real
///        librealsense code paths without physical hardware.
///
/// The Viam fork of librealsense (2.57.0+viam) does not support
/// software_device::add_to() for context registration. Instead, we use
/// rs2::syncer to compose frames directly from software sensors.

#include <gtest/gtest.h>

#include "discovery.hpp"
#include "realsense.hpp"

#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>

#include <boost/thread/synchronized_value.hpp>

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

namespace realsense {
namespace integration {
namespace test {

// Global test environment to manage the viam::sdk::Instance singleton.
class RealsenseTestEnvironment : public ::testing::Environment {
public:
  void SetUp() override {
    instance_ = std::make_unique<viam::sdk::Instance>();
  }
  void TearDown() override { instance_.reset(); }

private:
  std::unique_ptr<viam::sdk::Instance> instance_;
};

// Fixture that creates an rs2::software_device with color + depth sensors.
class SoftwareDeviceFixture : public ::testing::Test {
protected:
  static constexpr int kWidth = 640;
  static constexpr int kHeight = 480;
  static constexpr int kFps = 30;
  static constexpr const char *kSerial = "test-serial-001";
  static constexpr const char *kDeviceName = "Intel RealSense D435";

  void SetUp() override {
    // Create software device and register camera info.
    sw_dev_ = std::make_unique<rs2::software_device>();
    sw_dev_->register_info(RS2_CAMERA_INFO_NAME, kDeviceName);
    sw_dev_->register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, kSerial);

    // Shared intrinsics for both sensors.
    rs2_intrinsics intrinsics = {kWidth, kHeight, 320.f, 240.f, 600.f, 600.f,
                                 RS2_DISTORTION_NONE, {0}};

    // Color sensor and stream profile.
    color_sensor_ =
        std::make_unique<rs2::software_sensor>(sw_dev_->add_sensor("Color"));
    rs2_video_stream color_stream = {RS2_STREAM_COLOR, 0, 0, kWidth, kHeight,
                                     kFps, 3, RS2_FORMAT_RGB8, intrinsics};
    color_profile_ = color_sensor_->add_video_stream(color_stream, true);

    // Depth sensor and stream profile.
    depth_sensor_ =
        std::make_unique<rs2::software_sensor>(sw_dev_->add_sensor("Depth"));
    rs2_video_stream depth_stream = {RS2_STREAM_DEPTH, 0, 1, kWidth, kHeight,
                                     kFps, 2, RS2_FORMAT_Z16, intrinsics};
    depth_profile_ = depth_sensor_->add_video_stream(depth_stream, true);

    // Enable frameset compositing so color + depth arrive together.
    sw_dev_->create_matcher(RS2_MATCHER_DEFAULT);
  }

  /// Inject one pair of color + depth frames into the software device.
  void inject_frames(double timestamp_ms, int frame_number) {
    // Color: 640x480 RGB8, uniform grey.
    color_data_.assign(kWidth * kHeight * 3, 128);
    color_sensor_->on_video_frame({color_data_.data(), [](void *) {},
                                   kWidth * 3, 3, timestamp_ms,
                                   RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
                                   frame_number, color_profile_.get()});

    // Depth: 640x480 Z16, uniform 1m (value 1000 * 0.001 depth_units = 1.0m).
    depth_data_.assign(kWidth * kHeight, 1000);
    depth_sensor_->on_video_frame(
        {reinterpret_cast<void *>(depth_data_.data()), [](void *) {},
         kWidth * 2, 2, timestamp_ms, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
         frame_number, depth_profile_.get(), 0.001f});
  }

  /// Wait for the syncer to produce a composite frameset with both streams.
  rs2::frameset wait_for_composite_frameset(rs2::syncer &sync,
                                            int max_attempts = 10,
                                            int timeout_ms = 500) {
    rs2::frameset fset;
    for (int i = 0; i < max_attempts; i++) {
      if (sync.try_wait_for_frames(&fset, timeout_ms) && fset.size() == 2) {
        return fset;
      }
    }
    throw std::runtime_error("syncer did not produce a 2-frame frameset");
  }

  std::unique_ptr<rs2::software_device> sw_dev_;
  std::unique_ptr<rs2::software_sensor> color_sensor_;
  std::unique_ptr<rs2::software_sensor> depth_sensor_;
  rs2::stream_profile color_profile_;
  rs2::stream_profile depth_profile_;

  // Keep frame data alive for the duration of the injection.
  std::vector<uint8_t> color_data_;
  std::vector<uint16_t> depth_data_;
};

// Thin context wrapper that returns a real rs2::device (the software device)
// from query_devices(). Used for discovery integration tests since the Viam
// librealsense fork's add_to/query_devices path doesn't work.
class SoftwareDeviceContext {
public:
  // Wrapper that mimics rs2::device_list with operator[] and size().
  struct device_list {
    rs2::device dev;
    int size() const { return 1; }
    rs2::device operator[](int) const { return dev; }
  };

  SoftwareDeviceContext(rs2::device dev) : dev_(dev), devices_{dev_} {}

  device_list query_devices() const { return devices_; }

  void set_devices_changed_callback(
      std::function<void(rs2::event_information &)>) {}

private:
  rs2::device dev_;
  device_list devices_;
};

// ---------------------------------------------------------------------------
// Test 1: Syncer composes color + depth frames from software sensors.
// ---------------------------------------------------------------------------
TEST_F(SoftwareDeviceFixture, SyncerProducesFrameset) {
  rs2::syncer sync;
  color_sensor_->open(color_profile_);
  color_sensor_->start(sync);
  depth_sensor_->open(depth_profile_);
  depth_sensor_->start(sync);

  // Inject several frame pairs to let the syncer build a composite frameset.
  for (int i = 1; i <= 5; i++) {
    inject_frames(time::getNowMs(), i);
    std::this_thread::sleep_for(std::chrono::milliseconds(34)); // ~30fps
  }

  rs2::frameset fset = wait_for_composite_frameset(sync);

  auto color = fset.get_color_frame();
  ASSERT_TRUE(color);
  EXPECT_EQ(color.get_width(), kWidth);
  EXPECT_EQ(color.get_height(), kHeight);
  EXPECT_NE(color.get_data(), nullptr);

  auto depth = fset.get_depth_frame();
  ASSERT_TRUE(depth);
  EXPECT_EQ(depth.get_width(), kWidth);
  EXPECT_EQ(depth.get_height(), kHeight);
  EXPECT_NE(depth.get_data(), nullptr);

  color_sensor_->stop();
  depth_sensor_->stop();
  color_sensor_->close();
  depth_sensor_->close();
}

// ---------------------------------------------------------------------------
// Test 2: Discovery finds the software device with the correct serial number
//         and sensor list.
// ---------------------------------------------------------------------------
TEST_F(SoftwareDeviceFixture, DiscoverSoftwareDevice) {
  using CtxT = SoftwareDeviceContext;

  auto ctx = std::make_shared<CtxT>(static_cast<rs2::device>(*sw_dev_));

  discovery::RealsenseDiscovery<CtxT> disc(
      {}, viam::sdk::ResourceConfig("discovery"), ctx);

  auto configs = disc.discover_resources({});

  ASSERT_EQ(configs.size(), 1);
  EXPECT_EQ(configs[0].attributes().at("serial_number"), kSerial);

  auto sensors_val = configs[0].attributes().at("sensors");
  auto sensors_list = sensors_val.get_unchecked<viam::sdk::ProtoList>();
  ASSERT_EQ(sensors_list.size(), 2);
  EXPECT_EQ(sensors_list[0].get_unchecked<std::string>(), "color");
  EXPECT_EQ(sensors_list[1].get_unchecked<std::string>(), "depth");
}

// ---------------------------------------------------------------------------
// Test 3: GetImages returns color (JPEG) and depth (vnd.viam.dep) images
//         encoded from real librealsense frames produced by a software device.
// ---------------------------------------------------------------------------
TEST_F(SoftwareDeviceFixture, GetImagesFromSoftwareDevice) {
  using SyncCtx = boost::synchronized_value<SoftwareDeviceContext>;

  auto ctx = std::make_shared<SyncCtx>(
      SoftwareDeviceContext(static_cast<rs2::device>(*sw_dev_)));
  auto rs_ctx = std::make_shared<RealsenseContext<SyncCtx>>(ctx);
  auto assigned_serials = std::make_shared<
      boost::synchronized_value<std::unordered_set<std::string>>>();

  // Build a ResourceConfig for the Realsense component.
  viam::sdk::ProtoStruct attributes;
  attributes["serial_number"] = std::string(kSerial);
  viam::sdk::ProtoList sensors_list;
  sensors_list.push_back(std::string("color"));
  sensors_list.push_back(std::string("depth"));
  attributes["sensors"] = sensors_list;

  viam::sdk::ResourceConfig cfg(
      "rdk:component:camera", "", "test-cam", attributes, "",
      viam::sdk::Model("viam", "camera", "realsense"),
      viam::sdk::LinkConfig{}, viam::sdk::log_level::info);

  // Shared state for the syncer-based startDevice.
  auto syncer = std::make_shared<rs2::syncer>();
  auto pump_running = std::make_shared<std::atomic<bool>>(false);
  auto pump_thread = std::make_shared<std::thread>();

  // Capture sensor pointers for startDevice to open/start them.
  rs2::software_sensor *color_s = color_sensor_.get();
  rs2::software_sensor *depth_s = depth_sensor_.get();
  rs2::stream_profile c_prof = color_profile_;
  rs2::stream_profile d_prof = depth_profile_;

  // Custom DeviceFunctions using syncer instead of pipeline.
  DeviceFunctions dev_funcs;

  dev_funcs.createDevice =
      [](const std::string &serial, std::shared_ptr<rs2::device>,
         const std::unordered_set<std::string> &,
         const RsResourceConfig &, viam::sdk::LogSource &)
      -> std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> {
    auto my_dev = boost::synchronized_value<device::ViamRSDevice<>>();
    my_dev->serial_number = serial;
    my_dev->point_cloud_filter =
        std::make_shared<device::PointCloudFilter>();
    my_dev->align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
    my_dev->config = std::make_shared<rs2::config>();
    return std::make_shared<
        boost::synchronized_value<device::ViamRSDevice<>>>(my_dev);
  };

  dev_funcs.startDevice =
      [syncer, pump_running, pump_thread, color_s, depth_s, c_prof, d_prof](
          const std::string &,
          std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
              &device,
          std::shared_ptr<boost::synchronized_value<rs2::frameset>>
              &latest_frameset,
          std::uint64_t, const RsResourceConfig &viamConfig,
          viam::sdk::LogSource &) {
        // Open and start sensors into the syncer.
        color_s->open(c_prof);
        color_s->start(*syncer);
        depth_s->open(d_prof);
        depth_s->start(*syncer);

        auto dev_ptr = device->synchronize();
        dev_ptr->started = true;

        // Start a pump thread that polls the syncer and updates
        // latest_frameset_, mimicking what pipeline+callback does.
        *pump_running = true;
        *pump_thread = std::thread(
            [syncer, pump_running, &latest_frameset, viamConfig]() {
              while (pump_running->load()) {
                rs2::frameset fset;
                if (syncer->try_wait_for_frames(&fset, 100)) {
                  if (static_cast<int>(fset.size()) ==
                      static_cast<int>(viamConfig.sensors.size())) {
                    latest_frameset =
                        std::make_shared<
                            boost::synchronized_value<rs2::frameset>>(fset);
                  }
                }
              }
            });
      };

  dev_funcs.stopDevice =
      [pump_running, pump_thread, color_s, depth_s](
          std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
              &device,
          viam::sdk::LogSource &) -> bool {
    if (pump_running->load()) {
      *pump_running = false;
      if (pump_thread->joinable()) {
        pump_thread->join();
      }
    }
    try {
      color_s->stop();
      depth_s->stop();
      color_s->close();
      depth_s->close();
    } catch (...) {
    }
    if (device) {
      auto dev_ptr = device->synchronize();
      dev_ptr->started = false;
    }
    return true;
  };

  dev_funcs.destroyDevice =
      [](std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
             &device,
         viam::sdk::LogSource &) -> bool {
    if (!device)
      return false;
    {
      auto dev_ptr = device->synchronize();
      dev_ptr->pipe.reset();
      dev_ptr->device.reset();
      dev_ptr->config.reset();
      dev_ptr->align.reset();
      dev_ptr->point_cloud_filter.reset();
    }
    device = nullptr;
    return true;
  };

  dev_funcs.printDeviceInfo = [](const auto &, viam::sdk::LogSource &) {};
  dev_funcs.reconfigureDevice =
      [](std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>,
         const RsResourceConfig &, viam::sdk::LogSource &) {};

  // Construct the Realsense component with custom DeviceFunctions.
  Realsense<SyncCtx> camera({}, cfg, rs_ctx, dev_funcs, assigned_serials);

  // Inject frames with current system time so they pass the staleness check.
  for (int i = 1; i <= 5; i++) {
    inject_frames(time::getNowMs(), i);
    std::this_thread::sleep_for(std::chrono::milliseconds(34));
  }

  // Give the pump thread time to deliver the frameset.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Call get_images with no filters.
  auto response = camera.get_images({}, {});

  ASSERT_EQ(response.images.size(), 2);

  // Color image should be JPEG encoded.
  EXPECT_EQ(response.images[0].mime_type, "image/jpeg");
  EXPECT_FALSE(response.images[0].bytes.empty());

  // Depth image should be Viam's depth format.
  EXPECT_EQ(response.images[1].mime_type, "image/vnd.viam.dep");
  EXPECT_FALSE(response.images[1].bytes.empty());

  // Captured timestamp should be non-zero.
  EXPECT_NE(response.metadata.captured_at,
            viam::sdk::time_pt(std::chrono::nanoseconds(0)));
}

// ---------------------------------------------------------------------------
// Test 4: PointCloudFilter + encodeRGBPointsToPCD produce a valid PCD binary
//         blob from real librealsense frames generated by a software device.
//
//         Note: Realsense::get_point_cloud() is not called directly because
//         rs2::software_sensor frames report get_data_size()==0 (a known
//         librealsense limitation), which trips a guard in get_point_cloud.
//         Instead, we test the same pipeline components that get_point_cloud
//         calls internally: PointCloudFilter::process() and
//         encodeRGBPointsToPCD().
// ---------------------------------------------------------------------------
TEST_F(SoftwareDeviceFixture, PointCloudFromSoftwareDevice) {
  rs2::syncer sync;
  color_sensor_->open(color_profile_);
  color_sensor_->start(sync);
  depth_sensor_->open(depth_profile_);
  depth_sensor_->start(sync);

  for (int i = 1; i <= 5; i++) {
    inject_frames(time::getNowMs(), i);
    std::this_thread::sleep_for(std::chrono::milliseconds(34));
  }

  rs2::frameset fset = wait_for_composite_frameset(sync);
  ASSERT_EQ(fset.size(), 2u);

  // Run the same pipeline that get_point_cloud uses internally.
  device::PointCloudFilter pc_filter;
  auto [points, color_frame] = pc_filter.process(fset);

  ASSERT_TRUE(points);
  ASSERT_TRUE(color_frame);
  EXPECT_EQ(static_cast<int>(points.size()), kWidth * kHeight);

  // Encode to PCD and validate output.
  viam::sdk::LogSource logger;
  auto pcd_bytes = encoding::encodeRGBPointsToPCD(
      std::make_pair(std::move(points), std::move(color_frame)), logger);

  ASSERT_FALSE(pcd_bytes.empty());

  // Parse the PCD header (text portion before "DATA binary\n").
  std::string pcd_str(pcd_bytes.begin(), pcd_bytes.end());
  auto data_pos = pcd_str.find("DATA binary\n");
  ASSERT_NE(data_pos, std::string::npos) << "PCD header missing 'DATA binary'";

  std::string header = pcd_str.substr(0, data_pos + strlen("DATA binary\n"));

  // Validate required PCD header fields.
  EXPECT_NE(header.find("VERSION .7"), std::string::npos);
  EXPECT_NE(header.find("FIELDS x y z rgb"), std::string::npos);
  EXPECT_NE(header.find("SIZE 4 4 4 4"), std::string::npos);
  EXPECT_NE(header.find("TYPE F F F U"), std::string::npos);
  EXPECT_NE(header.find("COUNT 1 1 1 1"), std::string::npos);
  EXPECT_NE(header.find("HEIGHT 1"), std::string::npos);

  // 640x480 = 307200 points.
  int expected_points = kWidth * kHeight;
  EXPECT_NE(header.find("WIDTH " + std::to_string(expected_points)),
            std::string::npos);
  EXPECT_NE(header.find("POINTS " + std::to_string(expected_points)),
            std::string::npos);

  // Validate binary payload size: each point is 16 bytes (3 floats + 1 uint).
  size_t binary_offset = data_pos + strlen("DATA binary\n");
  size_t binary_size = pcd_bytes.size() - binary_offset;
  size_t expected_binary_size = expected_points * 16;
  EXPECT_EQ(binary_size, expected_binary_size);

  color_sensor_->stop();
  depth_sensor_->stop();
  color_sensor_->close();
  depth_sensor_->close();
}

} // namespace test
} // namespace integration
} // namespace realsense

GTEST_API_ int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(
      new realsense::integration::test::RealsenseTestEnvironment);
  return RUN_ALL_TESTS();
}
