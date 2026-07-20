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

// A proper rotation matrix distinct from the fixtures' pose (identity).
const char* IDENTITY_CSV = "1,0,0,0,1,0,0,0,1";

// Convenience builders so the intent of each edit reads clearly.
SafetyTableEdits height(double h) { SafetyTableEdits e; e.mount_height_m = h; return e; }
SafetyTableEdits heightAndCell(double h, double c) { SafetyTableEdits e; e.mount_height_m = h; e.cell_size_m = c; return e; }
SafetyTableEdits cell(double c) { SafetyTableEdits e; e.cell_size_m = c; return e; }
SafetyTableEdits robot(double r) { SafetyTableEdits e; e.robot_height_m = r; return e; }
SafetyTableEdits rotation(const std::string& csv) { SafetyTableEdits e; e.rotation_row_major = csv; return e; }
}

TEST(SafetyTableParams, PatchesHeightAndCellSize)
{
    std::string j = SIC_BARE;
    EXPECT_TRUE(patchSafetyInterfaceConfig(j, heightAndCell(0.30, 0.05)));
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
    EXPECT_FALSE(patchSafetyInterfaceConfig(j, heightAndCell(0.247, cell_size_m)));
    EXPECT_EQ(j, before); // string untouched on no-op
}

TEST(SafetyTableParams, EpsilonTolerance)
{
    std::string j = SIC_BARE;
    EXPECT_FALSE(patchSafetyInterfaceConfig(j, height(0.24705))); // within 1e-4? 5e-5 yes
    EXPECT_TRUE(patchSafetyInterfaceConfig(j, height(0.2475)));   // 5e-4 differs
}

TEST(SafetyTableParams, NegativeArgsSkipFields)
{
    std::string j = SIC_BARE;
    EXPECT_TRUE(patchSafetyInterfaceConfig(j, cell(0.05))); // only cell size
    auto parsed = nlohmann::json::parse(j);
    EXPECT_NEAR(parsed["camera_position"]["translation"][2].get<double>(), 0.247, 1e-9);
    std::string k = SIC_BARE;
    EXPECT_FALSE(patchSafetyInterfaceConfig(k, SafetyTableEdits{})); // all skipped
}

TEST(SafetyTableParams, HandlesWrappedAndBareRoots)
{
    std::string p = PRESET_WRAPPED;
    EXPECT_TRUE(patchSafetyPreset(p, height(0.30)));
    auto parsed = nlohmann::json::parse(p);
    EXPECT_NEAR(parsed["safety_preset"]["platform_config"]["transformation_link"]
                      ["translation"][2].get<double>(), 0.30, 1e-9);

    std::string c = CALIB_BARE;
    EXPECT_TRUE(patchCalibrationConfig(c, height(0.30)));
    auto cparsed = nlohmann::json::parse(c);
    EXPECT_NEAR(cparsed["camera_position"]["translation"][2].get<double>(), 0.30, 1e-9);
}

TEST(SafetyTableParams, ThrowsOnMissingStructure)
{
    std::string j = R"({"unrelated": 1})";
    EXPECT_THROW(patchSafetyInterfaceConfig(j, height(0.30)), std::runtime_error);
    EXPECT_THROW(patchSafetyPreset(j, height(0.30)), std::runtime_error);
    std::string bad = "not json";
    EXPECT_THROW(patchCalibrationConfig(bad, height(0.30)), nlohmann::json::exception);
}

TEST(SafetyTableParams, UnsetSkipsCalibrationAndPresetParsing)
{
    // No edits for the field means "skip", and both helpers bail out before
    // parsing, so even a bare "{}" input is accepted unchanged.
    std::string bare = "{}";
    EXPECT_FALSE(patchCalibrationConfig(bare, SafetyTableEdits{}));
    EXPECT_EQ(bare, "{}");

    std::string preset_bare = "{}";
    EXPECT_FALSE(patchSafetyPreset(preset_bare, SafetyTableEdits{}));
    EXPECT_EQ(preset_bare, "{}");

    std::string c = CALIB_BARE;
    EXPECT_FALSE(patchCalibrationConfig(c, SafetyTableEdits{}));
    EXPECT_EQ(c, CALIB_BARE);

    std::string p = PRESET_WRAPPED;
    EXPECT_FALSE(patchSafetyPreset(p, SafetyTableEdits{}));
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

TEST(SafetyTableParams, RobotHeightRange)
{
    // Firmware obstacle-height range is 0..100 cm, i.e. 0..1 m.
    EXPECT_TRUE(robotHeightInRange(ROBOT_HEIGHT_MIN_M));
    EXPECT_TRUE(robotHeightInRange(0.4));
    EXPECT_TRUE(robotHeightInRange(ROBOT_HEIGHT_MAX_M));
    EXPECT_FALSE(robotHeightInRange(ROBOT_HEIGHT_MAX_M + 0.01));
    EXPECT_FALSE(robotHeightInRange(40.0)); // cm passed as m
    EXPECT_FALSE(robotHeightInRange(-0.1));
}

TEST(SafetyTableParams, PatchesRobotHeightInPreset)
{
    std::string p = PRESET_WRAPPED;
    EXPECT_TRUE(patchSafetyPreset(p, robot(0.5)));
    auto parsed = nlohmann::json::parse(p);
    EXPECT_NEAR(parsed["safety_preset"]["platform_config"]["robot_height"].get<double>(), 0.5, 1e-9);
    // pose untouched
    EXPECT_NEAR(parsed["safety_preset"]["platform_config"]["transformation_link"]
                      ["translation"][2].get<double>(), 0.247, 1e-9);
}

TEST(SafetyTableParams, RobotHeightNoChangeWhenMatching)
{
    std::string p = PRESET_WRAPPED;
    std::string before = p;
    EXPECT_FALSE(patchSafetyPreset(p, robot(0.4))); // already 0.4
    EXPECT_EQ(p, before);
}

TEST(SafetyTableParams, ThrowsWhenRobotHeightRequestedButMissing)
{
    std::string p = R"({"safety_preset": {"platform_config": {
        "transformation_link": {"rotation": [[1,0,0],[0,1,0],[0,0,1]], "translation": [0,0,0.2]}
    }}})";
    EXPECT_THROW(patchSafetyPreset(p, robot(0.5)), std::runtime_error);
}

TEST(SafetyTableParams, RotationCsvValidation)
{
    EXPECT_TRUE(rotationCsvValid(IDENTITY_CSV));
    EXPECT_TRUE(rotationCsvValid("0,0,1,-1,0,0,0,-1,0")); // the fixtures' own pose
    EXPECT_FALSE(rotationCsvValid(""));                   // empty means unset, not valid
    EXPECT_FALSE(rotationCsvValid("1,0,0,0,1,0,0,0,2"));  // not orthonormal / det != 1
    EXPECT_FALSE(rotationCsvValid("2,0,0,0,2,0,0,0,2"));  // scaled, det = 8
    EXPECT_FALSE(rotationCsvValid("-1,0,0,0,1,0,0,0,1")); // reflection, det = -1
    EXPECT_FALSE(rotationCsvValid("1,0,0,0,1,0,0,0"));    // only 8 values
    EXPECT_FALSE(rotationCsvValid("1,0,0,0,1,0,0,0,1,0")); // 10 values
    EXPECT_FALSE(rotationCsvValid("a,b,c,d,e,f,g,h,i"));  // non-numeric
}

TEST(SafetyTableParams, PatchesRotation)
{
    std::string j = SIC_BARE;
    EXPECT_TRUE(patchSafetyInterfaceConfig(j, rotation(IDENTITY_CSV)));
    auto parsed = nlohmann::json::parse(j);
    EXPECT_NEAR(parsed["camera_position"]["rotation"][0][0].get<double>(), 1.0, 1e-9);
    EXPECT_NEAR(parsed["camera_position"]["rotation"][0][2].get<double>(), 0.0, 1e-9);
    EXPECT_NEAR(parsed["camera_position"]["rotation"][2][2].get<double>(), 1.0, 1e-9);
    // translation untouched
    EXPECT_NEAR(parsed["camera_position"]["translation"][2].get<double>(), 0.247, 1e-9);

    std::string p = PRESET_WRAPPED;
    EXPECT_TRUE(patchSafetyPreset(p, rotation(IDENTITY_CSV)));
    auto pp = nlohmann::json::parse(p);
    EXPECT_NEAR(pp["safety_preset"]["platform_config"]["transformation_link"]
                  ["rotation"][1][1].get<double>(), 1.0, 1e-9);
}

TEST(SafetyTableParams, RotationNoChangeWhenMatching)
{
    std::string j = SIC_BARE;
    std::string before = j;
    EXPECT_FALSE(patchSafetyInterfaceConfig(j, rotation("0,0,1,-1,0,0,0,-1,0")));
    EXPECT_EQ(j, before);
}

TEST(SafetyTableParams, CombinedPresetEdits)
{
    // Height, robot height and rotation applied together in one patch.
    SafetyTableEdits e;
    e.mount_height_m = 0.30;
    e.robot_height_m = 0.5;
    e.rotation_row_major = IDENTITY_CSV;
    std::string p = PRESET_WRAPPED;
    EXPECT_TRUE(patchSafetyPreset(p, e));
    auto pp = nlohmann::json::parse(p);
    auto& pc = pp["safety_preset"]["platform_config"];
    EXPECT_NEAR(pc["transformation_link"]["translation"][2].get<double>(), 0.30, 1e-9);
    EXPECT_NEAR(pc["robot_height"].get<double>(), 0.5, 1e-9);
    EXPECT_NEAR(pc["transformation_link"]["rotation"][0][0].get<double>(), 1.0, 1e-9);
}

TEST(SafetyTableParams, PatchAllPresetsAllOrNothing)
{
    // One structurally invalid preset anywhere in the batch must abort the
    // whole patch BEFORE any entry is modified - a partial batch would leave
    // the flash preset bank split across two mount heights.
    std::vector<std::string> presets = {PRESET_WRAPPED, "{}", PRESET_WRAPPED};
    const std::vector<std::string> before = presets;
    EXPECT_THROW(patchAllSafetyPresets(presets, height(0.30)), std::runtime_error);
    EXPECT_EQ(presets, before); // nothing modified, index 0 included
}

TEST(SafetyTableParams, PatchAllPresetsFlagsChanged)
{
    std::string already_at_target = PRESET_WRAPPED;
    ASSERT_TRUE(patchSafetyPreset(already_at_target, height(0.30)));

    std::vector<std::string> presets = {PRESET_WRAPPED, already_at_target};
    const std::vector<bool> changed = patchAllSafetyPresets(presets, height(0.30));
    ASSERT_EQ(changed.size(), 2u);
    EXPECT_TRUE(changed[0]);
    EXPECT_FALSE(changed[1]);
    auto parsed = nlohmann::json::parse(presets[0]);
    EXPECT_NEAR(parsed["safety_preset"]["platform_config"]["transformation_link"]
                      ["translation"][2].get<double>(), 0.30, 1e-9);
    EXPECT_EQ(presets[1], already_at_target); // no-op entry untouched
}

TEST(SafetyTableParams, PatchAllPresetsNoEditsIsNoOp)
{
    std::vector<std::string> presets = {PRESET_WRAPPED, "{}"};
    const std::vector<std::string> before = presets;
    const std::vector<bool> changed = patchAllSafetyPresets(presets, SafetyTableEdits{});
    EXPECT_EQ(changed, std::vector<bool>(2, false));
    EXPECT_EQ(presets, before);
}

TEST(SafetyTableParams, ThrowsWhenCellSizeRequestedButMissing)
{
    // Height skipped, cell size requested, but the table has no
    // occupancy_grid_params at all.
    std::string j = SIC_BARE;
    nlohmann::json parsed = nlohmann::json::parse(j);
    parsed.erase("occupancy_grid_params");
    std::string without_grid_params = parsed.dump();
    EXPECT_THROW(patchSafetyInterfaceConfig(without_grid_params, cell(0.05)), std::runtime_error);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
