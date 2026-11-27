#include "gicp_utils.hpp"
#include <Eigen/Geometry>
#include <gtest/gtest.h>

TEST(GicpTf, PhysicalTransformRetainsHeightAndGravityAlignment)
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) =
    (Eigen::AngleAxisf(0.8f, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(-0.2f, Eigen::Vector3f::UnitX()))
      .toRotationMatrix();
  pose.block<3, 1>(0, 3) = Eigen::Vector3f(4.7f, 7.1f, 0.31f);
  const auto full =
    icp_relocalization::gicp_utils::currentTransform("pcd_map", pose, rclcpp::Time(123), false);
  EXPECT_EQ(full.header.frame_id, "pcd_map");
  EXPECT_EQ(full.child_frame_id, "camera_init");
  EXPECT_EQ(full.header.stamp.nanosec, 123u);
  EXPECT_NEAR(full.transform.translation.z, 0.31, 1e-6);
  const auto & q = full.transform.rotation;
  Eigen::Quaternionf actual(q.w, q.x, q.y, q.z);
  EXPECT_TRUE(actual.toRotationMatrix().isApprox(pose.block<3, 3>(0, 0), 1e-6));
  const auto planar =
    icp_relocalization::gicp_utils::currentTransform("pcd_map", pose, rclcpp::Time(123), true);
  EXPECT_EQ(planar.transform.translation.z, 0.0);
  EXPECT_EQ(planar.transform.rotation.x, 0.0);
  EXPECT_EQ(planar.transform.rotation.y, 0.0);
}
