#include "robot.hpp"

#include <cmath>
#include <cassert>

constexpr float DEG2RAD = M_PI / 180.0;
constexpr float RAD2DEG = 180.0 / M_PI;

Robot::Robot(std::vector<float> &l,
             std::string ser,
             int baud,
             int speed):
                HOME({DEG2RAD * 45, DEG2RAD * 110, DEG2RAD * 180, DEG2RAD * 30}),
                L(l),
                SPEED(speed),
                MIN({500, 500, 500, 500}),
                MAX({2500, 2500, 2500, 2500}),
                RANGE({M_PI, M_PI, M_PI, M_PI}),
                q({0.0, 0.0, 0.0, 0.0}),
                gripper(GripperState::Closed)
{

    /* Open the serial port for communication */
    boost::asio::io_service io;
    this->serial = new boost::asio::serial_port(io, ser);
    this->serial->set_option(boost::asio::serial_port_base::baud_rate(baud));
    this->serial->set_option(boost::asio::serial_port_base::character_size(8 /* data bits */));
    this->serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    this->serial->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    this->set_des_q_rad(this->HOME);
    this->set_des_gripper(GripperState::Open);

}

Robot::~Robot()
{
    /* Close the serial port and delete the serial port pointer */
    if (this->serial->is_open()) this->serial->close();
    delete this->serial;
}

/* Set a single servo reference position
*   @param servo: The servo index
*   @param     q: The position in radians */
void Robot::set_des_q_single_rad(int servo, float q)
{
    assert(servo >= 0 && servo <= (int)this->L.size());
    std::string cmd = this->format_cmd(servo, this->RAD_2_TICKS(servo, q) + this->MIN[servo], this->SPEED);
    cmd += "\r";

    this->write_cmd(cmd);
    this->q.at(servo) = q;
}

/* Set a single servo reference position
*   @param servo: The servo index
*   @param     q: The position in degrees */
void Robot::set_des_q_single_deg(int servo, float q)
{
    this->set_des_q_single_rad(servo, q * RAD2DEG);
}

/* Set all servo reference position
*   @param     q: The position in rad */
void Robot::set_des_q_rad(const std::vector<float> & q)
{
    assert(q.size() == this->L.size());
    std::string cmd = "";
    for(uint i = 0; i < this->L.size(); i++)
    {
        cmd += this->format_cmd(i,
          this->RAD_2_TICKS(i, q.at(i)) + this->MIN[i],
          this->SPEED);
        this->q.at(i) = q.at(i);
    }
    cmd += "\r";

    this->write_cmd(cmd);
}


/* Set all servo reference position
*   @param     q: The position in degree */
void Robot::set_des_q_deg(const std::vector<float> & q)
{
    assert(q.size() == this->L.size());
    std::string cmd = "";
    for(uint i = 0; i < this->L.size(); i++)
    {
        cmd += this->format_cmd(i,
            this->RAD_2_TICKS(i, q.at(i)*DEG2RAD) + this->MIN[i],
            this->SPEED);
        this->q.at(i) = q.at(i)*DEG2RAD;
    }
    cmd += "\r";

    this->write_cmd(cmd);
}

/* Set the currently desired gripper state
*    @param state: Currently desired gripper state (Open or Closed)
*/
void Robot::set_des_gripper(GripperState state)
{
    std::string cmd;
    if(state == GripperState::Open)
    {
        cmd = this->format_cmd(4, 900, this->SPEED);
        this->gripper = GripperState::Open;
    }
    else if(state == GripperState::Closed)
    {
        cmd = this->format_cmd(4, 2500, this->SPEED);
        this->gripper = GripperState::Closed;
    }
    cmd += "\r";

    this->write_cmd(cmd);
}

/* Function that uses the min, max and range to compute the 
*  equivalent radians for a given number of ticks
*   @param servo: Servo index
*   @param   rad: Angle to be transformed
*            
*  @returns number of ticks equivalent to the rad angle
*/
float Robot::RAD_2_TICKS(int servo, float rad)
{
    return (this->MAX.at(servo) - this->MIN.at(servo)) / this->RANGE.at(servo) * rad;
}

/* Find the correctly formatted string based on a command
*   @param servo: The servo index
*   @param   pos: The position in ticks
*   @param   vel: The velocity in ticks per second
*
*   @return correctly formatted string */
std::string Robot::format_cmd(int servo, int pos, int vel)
{
    char buffer[13];
    sprintf(buffer, "#%dP%04dV%03d", servo, pos, vel);
    return std::string(buffer);
}

/* Write the command to the serial port
*   @param cmd: command to be forwarded
*/
void Robot::write_cmd(std::string cmd)
{
    this->serial->write_some(boost::asio::buffer(cmd, cmd.length()));
}

std::vector<float> Robot::get_q()
{
    return this->q;
}

GripperState Robot::get_gripper()
{
    return this->gripper;
}

