/**
 * @file    Testlvx_to_pcd.cpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#include <gtest/gtest.h>

#include <lvx_to_pcd/lvx_to_pcd.hpp>

class Testlvx_to_pcd : public ::testing::Test
{
 protected:
    void SetUp() override
    {
        start_time_ = time(nullptr);
    }

    void TearDown() override
    {
        const time_t end_time = time(nullptr);

        // expect test time less than 10 sec
        EXPECT_LE(end_time - start_time_, 10);
    }

    time_t start_time_;
};

TEST_F(Testlvx_to_pcd, TestInitialization)
{
    ASSERT_TRUE(true);
}
