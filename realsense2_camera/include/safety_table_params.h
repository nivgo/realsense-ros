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
#include <nlohmann/json.hpp>

namespace realsense2_camera
{
namespace safety_table_params
{
    // The flash table stores cell size in millimeters, parameters are meters.
    constexpr double CELL_SIZE_TABLE_UNITS_PER_METER = 1000.0;
    constexpr double HEIGHT_EPSILON_M = 1e-4;

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

    // For all three patchers: a negative value skips that field, the return
    // value is true iff json_str was rewritten.
    inline bool patchSafetyInterfaceConfig(std::string& json_str, double mount_height_m, double cell_size_m)
    {
        auto j = nlohmann::json::parse(json_str);
        auto& root = tableRoot(j, "safety_interface_config");
        bool changed = false;
        if (mount_height_m >= 0.0)
            changed |= patchTranslationZ(root, "camera_position", mount_height_m);
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
        if (changed)
            json_str = j.dump();
        return changed;
    }

    inline bool patchCalibrationConfig(std::string& json_str, double mount_height_m)
    {
        if (mount_height_m < 0.0)
            return false;
        auto j = nlohmann::json::parse(json_str);
        auto& root = tableRoot(j, "calibration_config");
        const bool changed = patchTranslationZ(root, "camera_position", mount_height_m);
        if (changed)
            json_str = j.dump();
        return changed;
    }

    inline bool patchSafetyPreset(std::string& json_str, double mount_height_m)
    {
        if (mount_height_m < 0.0)
            return false;
        auto j = nlohmann::json::parse(json_str);
        auto& root = tableRoot(j, "safety_preset");
        if (!root.contains("platform_config"))
            throw std::runtime_error("safety preset missing platform_config");
        const bool changed = patchTranslationZ(root["platform_config"], "transformation_link", mount_height_m);
        if (changed)
            json_str = j.dump();
        return changed;
    }
}  // namespace safety_table_params
}  // namespace realsense2_camera
