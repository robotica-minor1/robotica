#include <drone_msgs/ThrustSettings.h>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drone_constants.hpp"
#include "flight_controller.hpp"

class TranslateTest : public testing::Test {
	protected:
		FlightController fc;
};
                          
TEST_F(TranslateTest, testHoldPosition) {
	Eigen::Vector3f direction(0.0f, 0.0f, 0.0f);
	drone_msgs::ThrustSettings msg = fc.translate(direction);
	for(int i = 0; i < 4; i++) { 
		EXPECT_FLOAT_EQ(0.0f, msg.thrustAngles[i]);
		EXPECT_FLOAT_EQ(W/4, msg.thrust[i]);
	}
	for(int i = 0; i < 3; i++) { 
		EXPECT_FLOAT_EQ(0.0f, msg.attitude[i]);
	}
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}