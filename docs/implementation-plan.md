# Generic Device Manager Implementation Plan

> **Status: completed (historical).** This plan covers the initial implementation only. Since then `InProcessDeviceDriver` has moved into `device_manager_core/test/` (it is not part of the public API), the runtime set has grown (`ros2_process`, CH020 IMU and SMIT lidar in-process runtimes), and the test count has grown beyond the 92 recorded here. The current contract and architecture are documented in [docs/design.md](design.md); trust the code and that document over this plan.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the approved lifecycle, queue, event, hook, parameter, and ROS adapter implementation without retaining the old recovery model.

**Architecture:** `device_manager_core` owns domain types and one serial executor per device. `device_manager_msgs` defines generic ROS contracts. `device_manager_ros` translates those contracts and implements a standard ROS Lifecycle adapter without leaking ROS types into the core.

**Tech Stack:** C++17, ROS 2 Jazzy, ament_cmake, rosidl, rclcpp, GTest

---

### Task 1: Core contract tests

- [x] Replace the old tests with desired-API tests for lifecycle execution, queue ordering, stale filtering, event observation, hook retry, parameter patching, disabled startup, and the in-process driver.
- [x] Run the package tests and confirm they fail because the new API is absent.

### Task 2: ROS-independent core

- [x] Define the approved domain types and `IDeviceRuntime` contract.
- [x] Implement `InProcessDeviceDriver` with a supplied transition handler.
- [x] Implement one mutex/condition-variable queue per device with FIFO priority bands and atomic batch insertion.
- [x] Implement manager tick, event observation, query, submission, and parameter patching.
- [x] Run the core tests and require all to pass.

### Task 3: ROS contracts

- [x] Replace old recovery messages and services with device event, state, transition, batch submission, and parameter patch contracts.
- [x] Reuse `rcl_interfaces/Parameter` and `diagnostic_msgs/KeyValue`.
- [x] Build the interface package.

### Task 4: ROS node and Lifecycle adapter

- [x] Write a failing node/API integration test.
- [x] Map lifecycle edges to standard transition IDs.
- [x] Apply configure parameters before `/change_state` and confirm the target through `/get_state`.
- [x] Expose `~/devices`, `~/get_devices`, `~/change_state`, and `~/patch_parameters`.
- [x] Run the ROS tests and require all to pass.

### Task 5: Full verification

- [x] Build all three packages from a clean development environment.
- [x] Run all package tests and `colcon test-result --verbose` (92 tests, no failures).
- [x] Run `git diff --check`.
- [x] Confirm root and nested-repository status contain only scoped changes.
