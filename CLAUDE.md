# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

C++ module implementing the Viam Camera API for Intel RealSense depth cameras (D435, D435i, D455). Runs as a child process of viam-server, communicating over gRPC via unix socket. Registered as `viam:camera:realsense` (camera) and `viam:realsense:discovery` (discovery service) in the Viam module registry.

## Build Commands

```bash
make setup              # Create Python venv, install Conan 2.x, configure remotes
make module.tar.gz      # Full build: install deps via Conan, build, package tarball
make test               # Install deps + build with tests + run ctest
make test-native        # Build and test using pre-installed system deps (CI flow)
make lint               # Run clang-format on src/ and test/ (uses .clang-format style)
make clean              # Remove build-conan/, build-native/, module.tar.gz, venv/
```

The build system is CMake (3.25+) with Conan 2.x for dependency management. The Makefile wraps both. Native builds (`build-native`, `test-native`) are for CI environments where dependencies are pre-installed in Docker images.

## Running a Single Test

After `make test` has completed at least once (so deps are installed):
```bash
cd build-conan/build/Release && . ./generators/conanrun.sh && ctest --output-on-failure -R <test_name>
```
Test names correspond to the source file basenames in `test/`: `device_tests`, `encoding_tests`, `realsense_tests`, `time_tests`, `discovery_tests`, `sensors_tests`.

## Architecture

**Entry point**: `src/module/main.cpp` creates a `RealsenseContext` wrapping `rs2::context`, registers two models (Camera + Discovery) with `ModuleService`, and starts serving.

**Core components** (all header-only except `encoding.cpp`):
- `realsense.hpp`: `Realsense<SynchronizedContextT>` implements `viam::sdk::Camera`. Provides `GetImage`, `GetImages`, `GetPointCloud`, `GetProperties`, `GetGeometries`. Handles configuration, validation, and frame retrieval.
- `device.hpp` / `device_impl.hpp`: `ViamRSDevice` wraps `rs2::pipeline` and `rs2::device`. Manages sensor streams (color/depth), resolution, and frame retrieval. Heavy use of templates for testability.
- `encoding.cpp/.hpp`: JPEG and raw image encoding using libjpeg-turbo.
- `discovery.hpp`: `RealsenseDiscovery` implements `viam::sdk::Discovery` for auto-detecting connected RealSense cameras.
- `sensors.hpp`: Sensor type parsing and configuration.
- `time.hpp`: Timestamp utilities for frame synchronization.

**Thread safety**: Uses `boost::synchronized_value` to wrap `rs2::context` and shared state like assigned serial numbers. Device change callbacks from librealsense are handled thread-safely through `RealsenseContext`.

**Key constraints**:
- Frame timestamp difference between color and depth must be within 2ms
- Frames older than 1000ms are considered stale
- gRPC message size limit is 32MB (relevant for point clouds)
- macOS requires running as root

## Dependencies

- `viam-cpp-sdk` (0.20.1): Viam SDK for Camera, Discovery, ModuleService APIs
- `librealsense` (2.56.5 Linux / 2.57.0+viam macOS): Intel RealSense SDK
- `libjpeg-turbo`: Image compression
- `Boost`: Thread synchronization (`boost::synchronized_value`)
- `GoogleTest/GMock`: Testing (fetched via CMake FetchContent)

## CI

GitHub Actions workflows in `.github/workflows/`:
- `test.yml`: Runs on PRs. Tests on Ubuntu arm64/amd64 (Docker) and macOS.
- `docker.yml`: Builds/pushes Docker images to GHCR on push to main.
- `build-publish.yml`: Builds release artifacts and uploads to Viam module registry on GitHub release.

CI uses pre-built Docker images defined in `etc/Dockerfile.debian.bookworm`. The `.canon.yaml` maps image tags for CI.

## Conventions

- C++17 standard
- Code formatting: clang-format (v19 preferred) with project `.clang-format` style
- PR branch naming: `RSDK-<ticket>-short-description` (e.g., `RSDK-13347-fix-depth-color-differences`)
- Templates are used extensively for dependency injection in tests (mock contexts, devices)
- The `viamrealsense` CMake library target contains shared code; the executable links against it
