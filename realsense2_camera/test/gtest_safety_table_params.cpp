// Copyright 2026 RealSense, Inc. All Rights Reserved.
#include <gtest/gtest.h>
#include <safety_table_params.h>

using namespace realsense2_camera::safety_table_params;

namespace
{
const char* SIC_BARE = R"({
    "m12_safety_pins_configuration": {"power": {"direction": "In"}},
    "camera_position": {
        "rotation": [[0.0,0.0,1.0],[-1.0,0.0,0.0],[0.0,-1.0,0.0]],
        "translation": [0.0, 0.0, 0.247]
    },
    "occupancy_grid_params": {
        "grid_cell_seed": 1, "grid_cell_size": 70,
        "cell_threshold_factor": 1.0, "polynomial_bias": 0.0, "surface_height": 0.05
    }
})";

const char* PRESET_WRAPPED = R"({"safety_preset": {
    "platform_config": {
        "transformation_link": {
            "rotation": [[0.0,0.0,1.0],[-1.0,0.0,0.0],[0.0,-1.0,0.0]],
            "translation": [0.0, 0.0, 0.247]
        },
        "robot_height": 0.4
    },
    "safety_zones": {}
}})";

const char* CALIB_BARE = R"({
    "roi_num_of_segments": 0,
    "camera_position": {
        "rotation": [[0.0,0.0,1.0],[-1.0,0.0,0.0],[0.0,-1.0,0.0]],
        "translation": [0.0, 0.0, 0.247]
    }
})";
}

TEST(SafetyTableParams, PatchesHeightAndCellSize)
{
    std::string j = SIC_BARE;
    EXPECT_TRUE(patchSafetyInterfaceConfig(j, 0.30, 0.05));
    auto parsed = nlohmann::json::parse(j);
    EXPECT_NEAR(parsed["camera_position"]["translation"][2].get<double>(), 0.30, 1e-9);
    EXPECT_EQ(parsed["occupancy_grid_params"]["grid_cell_size"].get<long>(),
              std::lround(0.05 * CELL_SIZE_TABLE_UNITS_PER_METER));
    // untouched fields preserved
    EXPECT_EQ(parsed["occupancy_grid_params"]["grid_cell_seed"].get<int>(), 1);
    EXPECT_NEAR(parsed["camera_position"]["rotation"][0][2].get<double>(), 1.0, 1e-9);
}

TEST(SafetyTableParams, NoChangeWhenValuesMatch)
{
    std::string j = SIC_BARE;
    const long cell_m_that_matches_70 = 70; // table units
    double cell_size_m = cell_m_that_matches_70 / CELL_SIZE_TABLE_UNITS_PER_METER;
    std::string before = j;
    EXPECT_FALSE(patchSafetyInterfaceConfig(j, 0.247, cell_size_m));
    EXPECT_EQ(j, before); // string untouched on no-op
}

TEST(SafetyTableParams, EpsilonTolerance)
{
    std::string j = SIC_BARE;
    EXPECT_FALSE(patchSafetyInterfaceConfig(j, 0.24705, -1.0)); // within 1e-4? 5e-5 yes
    EXPECT_TRUE(patchSafetyInterfaceConfig(j, 0.2475, -1.0));   // 5e-4 differs
}

TEST(SafetyTableParams, NegativeArgsSkipFields)
{
    std::string j = SIC_BARE;
    EXPECT_TRUE(patchSafetyInterfaceConfig(j, -1.0, 0.05)); // only cell size
    auto parsed = nlohmann::json::parse(j);
    EXPECT_NEAR(parsed["camera_position"]["translation"][2].get<double>(), 0.247, 1e-9);
    std::string k = SIC_BARE;
    EXPECT_FALSE(patchSafetyInterfaceConfig(k, -1.0, -1.0)); // both skipped
}

TEST(SafetyTableParams, HandlesWrappedAndBareRoots)
{
    std::string p = PRESET_WRAPPED;
    EXPECT_TRUE(patchSafetyPreset(p, 0.30));
    auto parsed = nlohmann::json::parse(p);
    EXPECT_NEAR(parsed["safety_preset"]["platform_config"]["transformation_link"]
                      ["translation"][2].get<double>(), 0.30, 1e-9);

    std::string c = CALIB_BARE;
    EXPECT_TRUE(patchCalibrationConfig(c, 0.30));
    auto cparsed = nlohmann::json::parse(c);
    EXPECT_NEAR(cparsed["camera_position"]["translation"][2].get<double>(), 0.30, 1e-9);
}

TEST(SafetyTableParams, ThrowsOnMissingStructure)
{
    std::string j = R"({"unrelated": 1})";
    EXPECT_THROW(patchSafetyInterfaceConfig(j, 0.30, -1.0), std::runtime_error);
    EXPECT_THROW(patchSafetyPreset(j, 0.30), std::runtime_error);
    std::string bad = "not json";
    EXPECT_THROW(patchCalibrationConfig(bad, 0.30), nlohmann::json::exception);
}

TEST(SafetyTableParams, NegativeHeightSkipsCalibrationAndPresetParsing)
{
    // A negative height means "skip this field" and both helpers bail out
    // before parsing, so even a bare "{}" input is accepted unchanged.
    std::string bare = "{}";
    EXPECT_FALSE(patchCalibrationConfig(bare, -1.0));
    EXPECT_EQ(bare, "{}");

    std::string preset_bare = "{}";
    EXPECT_FALSE(patchSafetyPreset(preset_bare, -1.0));
    EXPECT_EQ(preset_bare, "{}");

    std::string c = CALIB_BARE;
    EXPECT_FALSE(patchCalibrationConfig(c, -1.0));
    EXPECT_EQ(c, CALIB_BARE);

    std::string p = PRESET_WRAPPED;
    EXPECT_FALSE(patchSafetyPreset(p, -1.0));
    EXPECT_EQ(p, PRESET_WRAPPED);
}

TEST(SafetyTableParams, MountHeightRange)
{
    // Physical bounds for an AMR-mounted camera; values outside them are
    // configuration mistakes (e.g. millimeters passed instead of meters).
    EXPECT_TRUE(mountHeightInRange(MOUNT_HEIGHT_MIN_M));
    EXPECT_TRUE(mountHeightInRange(0.247));
    EXPECT_TRUE(mountHeightInRange(MOUNT_HEIGHT_MAX_M));
    EXPECT_FALSE(mountHeightInRange(MOUNT_HEIGHT_MAX_M + 0.1));
    EXPECT_FALSE(mountHeightInRange(1000.0)); // mm passed as m
    EXPECT_FALSE(mountHeightInRange(-0.1));   // negative means unset, never "in range"
}

TEST(SafetyTableParams, CellSizeRange)
{
    // Firmware accepts grid cell edges of 10..500 mm; zero would provision
    // a degenerate grid and must never reach flash.
    EXPECT_TRUE(cellSizeInRange(CELL_SIZE_MIN_M));
    EXPECT_TRUE(cellSizeInRange(0.07));
    EXPECT_TRUE(cellSizeInRange(CELL_SIZE_MAX_M));
    EXPECT_FALSE(cellSizeInRange(0.0));
    EXPECT_FALSE(cellSizeInRange(CELL_SIZE_MIN_M - 1e-3));
    EXPECT_FALSE(cellSizeInRange(CELL_SIZE_MAX_M + 1e-3));
    EXPECT_FALSE(cellSizeInRange(70.0)); // mm passed as m
}

TEST(SafetyTableParams, PatchAllPresetsAllOrNothing)
{
    // One structurally invalid preset anywhere in the batch must abort the
    // whole patch BEFORE any entry is modified - a partial batch would leave
    // the flash preset bank split across two mount heights.
    std::vector<std::string> presets = {PRESET_WRAPPED, "{}", PRESET_WRAPPED};
    const std::vector<std::string> before = presets;
    EXPECT_THROW(patchAllSafetyPresets(presets, 0.30), std::runtime_error);
    EXPECT_EQ(presets, before); // nothing modified, index 0 included
}

TEST(SafetyTableParams, PatchAllPresetsFlagsChanged)
{
    std::string already_at_target = PRESET_WRAPPED;
    ASSERT_TRUE(patchSafetyPreset(already_at_target, 0.30));

    std::vector<std::string> presets = {PRESET_WRAPPED, already_at_target};
    const std::vector<bool> changed = patchAllSafetyPresets(presets, 0.30);
    ASSERT_EQ(changed.size(), 2u);
    EXPECT_TRUE(changed[0]);
    EXPECT_FALSE(changed[1]);
    auto parsed = nlohmann::json::parse(presets[0]);
    EXPECT_NEAR(parsed["safety_preset"]["platform_config"]["transformation_link"]
                      ["translation"][2].get<double>(), 0.30, 1e-9);
    EXPECT_EQ(presets[1], already_at_target); // no-op entry untouched
}

TEST(SafetyTableParams, PatchAllPresetsNegativeHeightIsNoOp)
{
    std::vector<std::string> presets = {PRESET_WRAPPED, "{}"};
    const std::vector<std::string> before = presets;
    const std::vector<bool> changed = patchAllSafetyPresets(presets, -1.0);
    EXPECT_EQ(changed, std::vector<bool>(2, false));
    EXPECT_EQ(presets, before);
}

TEST(SafetyTableParams, ThrowsWhenCellSizeRequestedButMissing)
{
    // Height skipped (negative), cell size requested (>= 0), but the table
    // has no occupancy_grid_params at all.
    std::string j = SIC_BARE;
    nlohmann::json parsed = nlohmann::json::parse(j);
    parsed.erase("occupancy_grid_params");
    std::string without_grid_params = parsed.dump();
    EXPECT_THROW(patchSafetyInterfaceConfig(without_grid_params, -1.0, 0.05), std::runtime_error);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
