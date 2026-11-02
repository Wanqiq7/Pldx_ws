#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "minco_controller/minco_mpc_controller.hpp"

namespace minco_controller {
class MincoMpcControllerTestPeer
{
public:
  static uint64_t version(const MincoMpcController & controller) { return controller.reference_version_; }
};

TEST(SpeedLimit, ChangesInvalidateCommandsInFlight)
{
  MincoMpcController controller;
  const auto original = MincoMpcControllerTestPeer::version(controller);
  controller.setSpeedLimit(1.0, false);
  const auto first = MincoMpcControllerTestPeer::version(controller);
  EXPECT_NE(first, original);

  controller.setSpeedLimit(1.0, false);
  EXPECT_EQ(MincoMpcControllerTestPeer::version(controller), first);
  controller.setSpeedLimit(1.0, true);
  EXPECT_NE(MincoMpcControllerTestPeer::version(controller), first);

  const auto changed = MincoMpcControllerTestPeer::version(controller);
  EXPECT_THROW(controller.setSpeedLimit(-1.0, false), std::invalid_argument);
  EXPECT_THROW(
    controller.setSpeedLimit(std::numeric_limits<double>::quiet_NaN(), false), std::invalid_argument);
  EXPECT_EQ(MincoMpcControllerTestPeer::version(controller), changed);
}
}  // namespace minco_controller
