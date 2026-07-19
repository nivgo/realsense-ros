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

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace realsense2_camera
{
namespace safety_table_params
{
    // The flash table stores cell size in millimeters, parameters are meters.
    constexpr double CELL_SIZE_TABLE_UNITS_PER_METER = 1000.0;
    constexpr double HEIGHT_EPSILON_M = 1e-4;

    // Values outside these bounds are configuration mistakes (typically a
    // unit mix-up) and are rejected before any flash write. Cell size bounds
    // come from the firmware's accepted grid cell edge of 10..500 mm; the
    // mount height ceiling is a physical bound for an AMR-mounted camera.
    constexpr double MOUNT_HEIGHT_MIN_M = 0.0;
    constexpr double MOUNT_HEIGHT_MAX_M = 2.0;
    constexpr double CELL_SIZE_MIN_M = 0.010;
    constexpr double CELL_SIZE_MAX_M = 0.500;

    inline bool mountHeightInRange(double height_m)
    {
        return height_m >= MOUNT_HEIGHT_MIN_M && height_m <= MOUNT_HEIGHT_MAX_M;
    }

    inline bool cellSizeInRange(double cell_size_m)
    {
        return cell_size_m >= CELL_SIZE_MIN_M && cell_size_m <= CELL_SIZE_MAX_M;
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
    }  // namespace detail

    // For all three patchers: a negative value skips that field, the return
    // value is true iff json_str was rewritten.
    inline bool patchSafetyInterfaceConfig(std::string& json_str, double mount_height_m, double cell_size_m)
    {
        return detail::patchTable(json_str, "safety_interface_config", [&](nlohmann::json& root) {
            bool changed = false;
            if (mount_height_m >= 0.0)
                changed |= detail::patchTranslationZ(root, "camera_position", mount_height_m);
            if (cell_size_m >= 0.0)
            {
                if (!root.contains("occupancy_grid_params") ||
                    !root["occupancy_grid_params"].contains("grid_cell_size"))
                {
                    throw std::runtime_error("safety table missing occupancy_grid_params.grid_cell_size");
                }
                const long target = std::lround(cell_size_m * CELL_SIZE_TABLE_UNITS_PER_METER);
                if (root["occupancy_grid_params"]["grid_cell_size"].get<long>() != target)
                {
                    root["occupancy_grid_params"]["grid_cell_size"] = target;
                    changed = true;
                }
            }
            return changed;
        });
    }

    inline bool patchCalibrationConfig(std::string& json_str, double mount_height_m)
    {
        if (mount_height_m < 0.0)
            return false;
        return detail::patchTable(json_str, "calibration_config", [&](nlohmann::json& root) {
            return detail::patchTranslationZ(root, "camera_position", mount_height_m);
        });
    }

    inline bool patchSafetyPreset(std::string& json_str, double mount_height_m)
    {
        if (mount_height_m < 0.0)
            return false;
        return detail::patchTable(json_str, "safety_preset", [&](nlohmann::json& root) {
            if (!root.contains("platform_config"))
                throw std::runtime_error("safety preset missing platform_config");
            return detail::patchTranslationZ(root["platform_config"], "transformation_link", mount_height_m);
        });
    }

    // Patch the mount height of a whole preset batch, all-or-nothing: every
    // entry is parsed and patched into a scratch copy first, so a structurally
    // invalid preset anywhere in the batch throws WITHOUT modifying any input.
    // A partial batch would leave the flash preset bank split across two mount
    // heights, which is why the caller must not write anything if this throws.
    // Returns per-index "changed" flags.
    inline std::vector<bool> patchAllSafetyPresets(std::vector<std::string>& presets, double mount_height_m)
    {
        std::vector<bool> changed(presets.size(), false);
        if (mount_height_m < 0.0)
            return changed;
        std::vector<std::string> patched(presets);
        for (size_t i = 0; i < patched.size(); ++i)
            changed[i] = patchSafetyPreset(patched[i], mount_height_m);
        presets = std::move(patched);
        return changed;
    }
}  // namespace safety_table_params
}  // namespace realsense2_camera
