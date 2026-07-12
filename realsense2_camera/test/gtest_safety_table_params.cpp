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

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
