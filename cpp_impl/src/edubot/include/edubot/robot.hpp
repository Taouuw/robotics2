#include <string>
#include <vector>
#include <boost/asio.hpp>


enum GripperState
{
    Closed = 0,
    Open = 1
};

class Robot
{
public:

    Robot(std::vector<float> &l,
          std::string ser="/dev/ttyUSB0",
          int baud=9600,
          int speed=100);

    ~Robot();

    void set_des_q_single_rad(int servo, float q);
    void set_des_q_single_deg(int servo, float q);
    
    void set_des_q_rad(const std::vector<float> & q);
    void set_des_q_deg(const std::vector<float> & q);

    void set_des_gripper(GripperState state);

    std::vector<float> get_q();
    GripperState get_gripper();

private:
    
    const std::vector<float> HOME;
    const std::vector<float> L; 
    const int SPEED;

    const std::vector<int> MIN;
    const std::vector<int> MAX;
    const std::vector<float> RANGE;

    std::vector<float> q;
    GripperState gripper;

    boost::asio::serial_port* serial;

    void write_cmd(std::string cmd);
    std::string format_cmd(int servo, int pos, int vel);
    float RAD_2_TICKS(int servo, float rad);


};