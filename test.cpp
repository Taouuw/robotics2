#include "cpp_impl/robot.hpp"
#include <unistd.h>

int main(int argc, char const *argv[])
{
    std::vector<float> l = {1, 1, 1, 1};
    Robot robot(l);

    sleep(2);
    robot.set_des_gripper(GripperState::Closed);
    sleep(2);
    robot.set_des_gripper(GripperState::Open);
    
    return 0;
}
