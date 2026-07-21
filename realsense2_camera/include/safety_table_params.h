// Copyright 2026 RealSense, Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace realsense2_camera
{
namespace safety_table_params
{
    // The flash table stores cell size in millimeters, parameters are meters.
    // Mount/robot height, translation, and rotation are stored in the tables
    // in the same units the parameters use (meters; rotation is unitless).
    constexpr double CELL_SIZE_TABLE_UNITS_PER_METER = 1000.0;
    constexpr double HEIGHT_EPSILON_M = 1e-4;
    constexpr double ROTATION_EPSILON = 1e-4;           // change detection
    constexpr double ROTATION_ORTHONORMAL_EPSILON = 1e-3; // validation tolerance

    // Values outside these bounds are configuration mistakes (typically a
    // unit mix-up) and are rejected before any flash write.
    //   - cell size: the firmware's accepted grid cell edge of 10..500 mm
    //     (GRID_CELL_LOWER/UPPER_LIMIT in the safety algorithm).
    //   - robot height: the firmware-enforced deployment range [0.4, 1.5] m
    //     from the D580 safety-flash spec, delegated to the Safety Constraints
    //     Validation Library; a value below 0.4 m is rejected by the device
    //     with "Value Out Of Range" (this is what bounds it, not the ROS side).
    //   - mount height: a physical bound for an AMR-mounted camera; the exact
    //     firmware write-limit is not yet confirmed, so this is deliberately
    //     loose (catches gross unit mix-ups only).
    constexpr double MOUNT_HEIGHT_MIN_M = 0.0;
    constexpr double MOUNT_HEIGHT_MAX_M = 2.0;
    constexpr double CELL_SIZE_MIN_M = 0.010;
    constexpr double CELL_SIZE_MAX_M = 0.500;
    constexpr double ROBOT_HEIGHT_MIN_M = 0.4;
    constexpr double ROBOT_HEIGHT_MAX_M = 1.5;

    // One launch's worth of requested safety-table edits. Each field carries
    // its own "unset" sentinel so callers set only what they mean to change:
    // a negative number, or an empty rotation string, leaves that field in
    // flash untouched. Which fields a given table cares about is table
    // specific (e.g. cell size is only in the interface config, robot height
    // only in presets).
    struct SafetyTableEdits
    {
        double mount_height_m = -1.0;    // camera-pose translation[2]
        double cell_size_m = -1.0;       // interface-config occupancy grid cell size
        double robot_height_m = -1.0;    // preset platform_config.robot_height
        std::string rotation_row_major;  // camera-pose 3x3 rotation, 9 values row-major
    };

    inline bool mountHeightInRange(double height_m)
    {
        return height_m >= MOUNT_HEIGHT_MIN_M && height_m <= MOUNT_HEIGHT_MAX_M;
    }

    inline bool cellSizeInRange(double cell_size_m)
    {
        return cell_size_m >= CELL_SIZE_MIN_M && cell_size_m <= CELL_SIZE_MAX_M;
    }

    inline bool robotHeightInRange(double robot_height_m)
    {
        return robot_height_m >= ROBOT_HEIGHT_MIN_M && robot_height_m <= ROBOT_HEIGHT_MAX_M;
    }

    // Implementation details - not part of the public API of this header.
    namespace detail
    {
        // Tables read from the device may arrive bare or wrapped in a root key
        // (e.g. {"safety_preset": {...}}). Return the table body either way.
        inline nlohmann::json& tableRoot(nlohmann::json& j, const char* wrapper_key)
        {
            return j.contains(wrapper_key) ? j[wrapper_key] : j;
        }

        // Parse 9 comma-separated numbers (row-major 3x3). Throws on any other
        // count or a non-numeric token.
        inline std::array<double, 9> parseRotationCsv(const std::string& csv)
        {
            std::array<double, 9> r{};
            std::stringstream ss(csv);
            std::string token;
            size_t n = 0;
            while (std::getline(ss, token, ','))
            {
                if (n >= 9)
                    throw std::runtime_error("safety_camera.rotation expects 9 comma-separated values");
                r[n++] = std::stod(token); // throws std::invalid_argument on a non-numeric token
            }
            if (n != 9)
                throw std::runtime_error("safety_camera.rotation expects 9 comma-separated values");
            return r;
        }

        // A proper rotation: rows orthonormal (R Rt = I) and determinant +1.
        inline bool isRotationMatrix(const std::array<double, 9>& r,
                                     double eps = ROTATION_ORTHONORMAL_EPSILON)
        {
            auto dot = [&](int a, int b) {
                return r[a*3+0]*r[b*3+0] + r[a*3+1]*r[b*3+1] + r[a*3+2]*r[b*3+2];
            };
            for (int a = 0; a < 3; ++a)
                for (int b = a; b < 3; ++b)
                    if (std::fabs(dot(a, b) - (a == b ? 1.0 : 0.0)) > eps)
                        return false;
            const double det =
                r[0]*(r[4]*r[8] - r[5]*r[7]) -
                r[1]*(r[3]*r[8] - r[5]*r[6]) +
                r[2]*(r[3]*r[7] - r[4]*r[6]);
            return std::fabs(det - 1.0) <= eps;
        }

        // Set node[key].translation[2] = height_m. Returns true if it changed.
        inline bool patchTranslationZ(nlohmann::json& node, const char* key, double height_m)
        {
            if (!node.contains(key) || !node[key].contains("translation") ||
                !node[key]["translation"].is_array() || node[key]["translation"].size() < 3)
            {
                throw std::runtime_error(std::string("safety table missing ") + key + ".translation");
            }
            if (std::fabs(node[key]["translation"][2].get<double>() - height_m) <= HEIGHT_EPSILON_M)
                return false;
            node[key]["translation"][2] = height_m;
            return true;
        }

        // Set node[key].rotation = the given row-major 3x3. Returns true if any
        // element changed.
        inline bool patchRotation(nlohmann::json& node, const char* key,
                                  const std::array<double, 9>& r)
        {
            if (!node.contains(key) || !node[key].contains("rotation") ||
                !node[key]["rotation"].is_array() || node[key]["rotation"].size() != 3)
            {
                throw std::runtime_error(std::string("safety table missing ") + key + ".rotation");
            }
            auto& rot = node[key]["rotation"];
            bool changed = false;
            for (int row = 0; row < 3; ++row)
            {
                if (!rot[row].is_array() || rot[row].size() != 3)
                    throw std::runtime_error(std::string("safety table malformed ") + key + ".rotation");
                for (int col = 0; col < 3; ++col)
                {
                    const double target = r[row*3 + col];
                    if (std::fabs(rot[row][col].get<double>() - target) > ROTATION_EPSILON)
                    {
                        rot[row][col] = target;
                        changed = true;
                    }
                }
            }
            return changed;
        }

        // Patch translation[2] and/or rotation of the pose object node[pose_key].
        inline bool patchCameraPose(nlohmann::json& node, const char* pose_key,
                                    const SafetyTableEdits& edits)
        {
            bool changed = false;
            if (edits.mount_height_m >= 0.0)
                changed |= patchTranslationZ(node, pose_key, edits.mount_height_m);
            if (!edits.rotation_row_major.empty())
                changed |= patchRotation(node, pose_key, parseRotationCsv(edits.rotation_row_major));
            return changed;
        }

        // Shared shell of every patcher: parse, unwrap, mutate, dump-if-changed.
        template <typename Mutator>
        inline bool patchTable(std::string& json_str, const char* wrapper_key, Mutator&& mutate)
        {
            auto j = nlohmann::json::parse(json_str);
            const bool changed = mutate(tableRoot(j, wrapper_key));
            if (changed)
                json_str = j.dump();
            return changed;
        }

        inline bool presetHasEdits(const SafetyTableEdits& e)
        {
            return e.mount_height_m >= 0.0 || e.robot_height_m >= 0.0 || !e.rotation_row_major.empty();
        }
    }  // namespace detail

    // True iff csv is exactly 9 numbers forming a proper rotation matrix.
    inline bool rotationCsvValid(const std::string& csv)
    {
        try
        {
            return detail::isRotationMatrix(detail::parseRotationCsv(csv));
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    // For every patcher: an unset field (negative number / empty rotation)
    // leaves that field untouched; the return value is true iff json_str was
    // rewritten.
    inline bool patchSafetyInterfaceConfig(std::string& json_str, const SafetyTableEdits& edits)
    {
        return detail::patchTable(json_str, "safety_interface_config", [&](nlohmann::json& root) {
            bool changed = detail::patchCameraPose(root, "camera_position", edits);
            if (edits.cell_size_m >= 0.0)
            {
                if (!root.contains("occupancy_grid_params") ||
                    !root["occupancy_grid_params"].contains("grid_cell_size"))
                {
                    throw std::runtime_error("safety table missing occupancy_grid_params.grid_cell_size");
                }
                const long target = std::lround(edits.cell_size_m * CELL_SIZE_TABLE_UNITS_PER_METER);
                if (root["occupancy_grid_params"]["grid_cell_size"].get<long>() != target)
                {
                    root["occupancy_grid_params"]["grid_cell_size"] = target;
                    changed = true;
                }
            }
            return changed;
        });
    }

    inline bool patchCalibrationConfig(std::string& json_str, const SafetyTableEdits& edits)
    {
        if (edits.mount_height_m < 0.0 && edits.rotation_row_major.empty())
            return false;
        return detail::patchTable(json_str, "calibration_config", [&](nlohmann::json& root) {
            return detail::patchCameraPose(root, "camera_position", edits);
        });
    }

    inline bool patchSafetyPreset(std::string& json_str, const SafetyTableEdits& edits)
    {
        if (!detail::presetHasEdits(edits))
            return false;
        return detail::patchTable(json_str, "safety_preset", [&](nlohmann::json& root) {
            if (!root.contains("platform_config"))
                throw std::runtime_error("safety preset missing platform_config");
            auto& pc = root["platform_config"];
            bool changed = detail::patchCameraPose(pc, "transformation_link", edits);
            if (edits.robot_height_m >= 0.0)
            {
                if (!pc.contains("robot_height"))
                    throw std::runtime_error("safety preset missing platform_config.robot_height");
                if (std::fabs(pc["robot_height"].get<double>() - edits.robot_height_m) > HEIGHT_EPSILON_M)
                {
                    pc["robot_height"] = edits.robot_height_m;
                    changed = true;
                }
            }
            return changed;
        });
    }

    // Patch a whole preset batch, all-or-nothing: every entry is parsed and
    // patched into a scratch copy first, so a structurally invalid preset
    // anywhere in the batch throws WITHOUT modifying any input. A partial
    // batch would leave the flash preset bank inconsistent, which is why the
    // caller must not write anything if this throws. Returns per-index
    // "changed" flags.
    inline std::vector<bool> patchAllSafetyPresets(std::vector<std::string>& presets,
                                                   const SafetyTableEdits& edits)
    {
        std::vector<bool> changed(presets.size(), false);
        if (!detail::presetHasEdits(edits))
            return changed;
        std::vector<std::string> patched(presets);
        for (size_t i = 0; i < patched.size(); ++i)
            changed[i] = patchSafetyPreset(patched[i], edits);
        presets = std::move(patched);
        return changed;
    }
}  // namespace safety_table_params
}  // namespace realsense2_camera
