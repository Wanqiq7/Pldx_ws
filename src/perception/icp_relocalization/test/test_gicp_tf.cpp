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

TEST(GicpTf, FullGeometryChecksHeightAndTiltCorrections)
{
  const Eigen::Matrix4f reference = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f candidate = reference;
  candidate(2, 3) = 0.2f;
  candidate.block<3, 3>(0, 0) = Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitX()).toRotationMatrix();
  const auto full = icp_relocalization::gicp_utils::transformDelta(reference, candidate, false);
  EXPECT_NEAR(full.translation, 0.2, 1e-6);
  EXPECT_NEAR(full.rotation, 0.3, 1e-6);
  const auto planar = icp_relocalization::gicp_utils::transformDelta(reference, candidate, true);
  EXPECT_DOUBLE_EQ(planar.translation, 0.0);
  EXPECT_DOUBLE_EQ(planar.rotation, 0.0);
}

TEST(GicpTf, RotationDifferenceIsInvariantToWorldHeading)
{
  Eigen::Matrix4f reference = Eigen::Matrix4f::Identity();
  reference.block<3, 3>(0, 0) = Eigen::AngleAxisf(2.9f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  Eigen::Matrix4f relative = Eigen::Matrix4f::Identity();
  relative.block<3, 3>(0, 0) = Eigen::AngleAxisf(-0.2f, Eigen::Vector3f::UnitY()).toRotationMatrix();
  const auto delta = icp_relocalization::gicp_utils::transformDelta(reference, reference * relative, false);
  EXPECT_NEAR(delta.rotation, 0.2, 1e-6);
}

TEST(GicpTf, MovingRegistrationWindowKeepsDistantScanAndReturnsOriginalFrame)
{
  auto source = std::make_shared<icp_relocalization::PointCloud>();
  auto target = std::make_shared<icp_relocalization::PointCloud>();
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      for (int plane = 0; plane < 3; ++plane) {
        Eigen::Vector3f point(i * .05f, j * .05f, .2f);
        if (plane == 1)
          point = Eigen::Vector3f(0.f, i * .05f, j * .05f);
        if (plane == 2)
          point = Eigen::Vector3f(i * .05f, 0.f, j * .05f);
        source->push_back(pcl::PointXYZ(point.x() + 20.f, point.y(), point.z()));
        target->push_back(pcl::PointXYZ(point.x() + 24.f, point.y() + 7.f, point.z()));
      }
    }
  }
  icp_relocalization::GicpFilter::Options options;
  options.source_voxel_leaf_size = .05;
  options.target_voxel_leaf_size = .05;
  options.source_crop_min_x = -2.;
  options.source_crop_max_x = 2.;
  options.local_map_radius = 3.;
  icp_relocalization::GicpFilter filter(target, options);
  EXPECT_TRUE(filter.prepareSource(nullptr).cropped_cloud->empty());
  EXPECT_TRUE(filter.prepareSource(source).cropped_cloud->empty());
  const auto prepared = filter.prepareSource(source, Eigen::Vector3f(20.f, 0.f, 0.f));
  ASSERT_GT(prepared.cropped_cloud->size(), 100u);
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  guess(0, 3) = 4.f;
  guess(1, 3) = 7.f;
  const auto result = filter.alignPrepared(prepared, guess);
  ASSERT_TRUE(result.converged);
  EXPECT_TRUE(result.final_transformation.isApprox(guess, 1e-3));
  const auto fine = filter.alignPrepared(prepared, result.final_transformation, .2);
  ASSERT_TRUE(fine.converged);
  EXPECT_TRUE(fine.final_transformation.isApprox(guess, 1e-3));
  ASSERT_FALSE(filter.getCurrentLocalMapCloud()->empty());
}
