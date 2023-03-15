/** \file serial_spinner.cpp
 * \brief Serial spinner class to interface with the boards
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// Local includes

#include "serial_spinner.hpp"
#include "protocol.hpp"

// Std includes

// ROS includes

#include "serial/GameStage.h"
#include "serial/GameStatus.h"
#include "serial/PositionFeedback.h"
#include "serial/Shoot.h"
#include "serial/TurretFeedback.h"

// OS includes

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace utils {

constexpr float fromMillimeter(int16_t mm_s) {
    // mm/s to m/s
    return static_cast<float>(mm_s) * 1.e-3f;
}

constexpr float fromAngularSpeed(int16_t omega) {
    // Millirad/s to rad/s
    return static_cast<float>(omega) * 1.e-3f;
}

constexpr int16_t toMillimeter(float m_s) {
    // m/s to mm/s
    return static_cast<int16_t>(m_s * 1e3);
}

constexpr int16_t toAngularSpeed(float omega) {
    // Rad/s to millirad/s
    return static_cast<int16_t>(omega * 1e3);
}
} // namespace utils

SerialSpinner::SerialSpinner(ros::NodeHandle& n, const std::string& device,
                             int _baud, int _len, int _stop, bool _parity,
                             double _freq)
    : nh(n), baud_rate(_baud), length(_len), stop_bits(_stop), parity(_parity),
      frequency(_freq) {
    initSerial(device);

    pub_status = nh.advertise<serial::GameStatus>("gamestatus", 1);
    pub_stage = nh.advertise<serial::GameStage>("gamestage", 1);
    pub_turret = nh.advertise<serial::TurretFeedback>("turret", 1);
    pub_position = nh.advertise<serial::PositionFeedback>("position", 1);

    sub_target =
        nh.subscribe("target", 1, &SerialSpinner::callbackTarget, this);
    sub_movement =
        nh.subscribe("movement", 1, &SerialSpinner::callbackMovement, this);
    sub_shoot = nh.subscribe("shoot", 1, &SerialSpinner::callbackShoot, this);
}

SerialSpinner::~SerialSpinner() {
    if (fd != 0) {
        close(fd);
    }
}

void SerialSpinner::initSerial(const std::string& device) {
    fd = open(device.c_str(), O_RDWR);

    if (fd < 0) {
        throw std::runtime_error("Could not open device " + device +
                                 strerror(errno));
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        throw std::runtime_error("Could not get term info " + device +
                                 strerror(errno));
    }

    // Set parity
    if (parity) {
        tty.c_cflag |= PARENB;
    } else {
        tty.c_cflag &= ~PARENB;
    }

    // Clear CSTOPB if only one stop bit is used
    if (stop_bits == 1) {
        tty.c_cflag &= ~CSTOPB;
    } else {
        tty.c_cflag |= CSTOPB;
    }

    // Set length
    tty.c_cflag &= ~CSIZE;
    switch (length) {
    case 5:
        tty.c_cflag |= CS5;
        break;
    case 6:
        tty.c_cflag |= CS6;
        break;
    case 7:
        tty.c_cflag |= CS7;
        break;
    case 8:
        tty.c_cflag |= CS8;
        break;
    default:
        throw std::runtime_error("Unsupported byte length");
    }

    // Disable flow control
    tty.c_cflag &= ~CRTSCTS;

    // Disable modem features
    tty.c_cflag |= CREAD | CLOCAL;

    // Disable canonical mode (line-buffering) & signal char
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    // Disable input flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Disable output interpretation
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // Non-blocking read
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    // Set baud
    int err = cfsetispeed(&tty, B230400);
    err += cfsetospeed(&tty, B230400);

    if (err != 0) {
        throw std::runtime_error("Could not set IOspeed");
    }

    // Aaaand .. we're done ! Commit to the OS
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error(std::string("Could not set tty attributes") +
                                 strerror(errno));
    }
}

void SerialSpinner::spin() {
    ros::Rate rate(frequency);

    while (ros::ok()) {
        handleSerial();

        rate.sleep();
        ros::spinOnce();
    }
}

template <>
void SerialSpinner::handleMessage<serial::msg::Status>(
    const serial::msg::Status& status) {
    serial::GameStatus msg;

    msg.stamp = ros::Time::now();
    msg.robot_type = status.robot_type;

    msg.red_std_hp = status.red_std_hp;
    msg.red_hro_hp = status.red_hro_hp;
    msg.red_sty_hp = status.red_sty_hp;
    msg.blu_std_hp = status.blu_std_hp;
    msg.blu_hro_hp = status.blu_hro_hp;
    msg.blu_sty_hp = status.blu_sty_hp;

    msg.mode = status.mode;

    pub_status.publish(msg);
}

template <>
void SerialSpinner::handleMessage<serial::msg::Gamestage>(
    const serial::msg::Gamestage& gamestage) {
    serial::GameStage msg;

    msg.stamp = ros::Time::now();
    msg.gamestage = gamestage.gamestage;

    pub_stage.publish(msg);
}

template <>
void SerialSpinner::handleMessage<serial::msg::TurretFeedback>(
    const serial::msg::TurretFeedback& turret_feedback) {
    serial::TurretFeedback msg;

    msg.stamp = ros::Time::now();

    msg.pitch = utils::fromAngularSpeed(turret_feedback.pitch);
    msg.yaw = utils::fromAngularSpeed(turret_feedback.yaw);

    pub_turret.publish(msg);
}

template <>
void SerialSpinner::handleMessage<serial::msg::PositionFeedback>(
    const serial::msg::PositionFeedback& position_feedback) {
    serial::PositionFeedback msg;

    msg.stamp = ros::Time::now();

    msg.imu_ax = position_feedback.imu_ax;
    msg.imu_ay = position_feedback.imu_ay;
    msg.imu_az = position_feedback.imu_az;
    msg.imu_rx = position_feedback.imu_rx;
    msg.imu_ry = position_feedback.imu_ry;
    msg.imu_rz = position_feedback.imu_rz;
    msg.enc_1 = position_feedback.enc_1;
    msg.enc_2 = position_feedback.enc_2;
    msg.enc_3 = position_feedback.enc_3;
    msg.enc_4 = position_feedback.enc_4;
    msg.delta_t = position_feedback.delta_t;

    pub_position.publish(msg);
}

void SerialSpinner::handleSerial() {
    int bytes;

    serial::msg::IncomingMessage message{serial::None()};

    // Attempt to read a command
    bytes = read(fd, &message, serial::HEADER_SIZE);
    if (bytes < serial::HEADER_SIZE) {
        // No incoming command, return immediatly
        return;
    }

    if (message.header.start_byte != serial::START_FRAME) {
        ROS_ERROR("Start frame not recognized, dropping command");
        return;
    }

    uint8_t* payload =
        reinterpret_cast<uint8_t*>(&message) + serial::HEADER_SIZE;
    bytes = read(fd, payload, message.header.data_len);
    if (bytes < message.header.data_len) {
        ROS_ERROR("Incomplete read on input payload");
        return;
    }

    // Could use a std::visit-type method here, but this will do for now.

    using namespace serial::msg;
    switch (message.header.cmd_id) {
    case Status::ID:
        handleMessage(message.status);
        break;
    case Gamestage::ID:
        handleMessage(message.gamestage);
        break;
    case TurretFeedback::ID:
        handleMessage(message.turret_feedback);
        break;
    case PositionFeedback::ID:
        handleMessage(message.position_feedback);
        break;
    default:
        ROS_ERROR("Unknown message type %d", message.header.cmd_id);
    }
}

serial::msg::IncomingMessage
SerialSpinner::deseralizeMessage(const std::vector<uint8_t>& buffer) {
    serial::msg::IncomingMessage message{serial::Header<serial::None>()};

    if (buffer.size() < serial::HEADER_SIZE) {
        throw std::runtime_error("Incomplete header");
    }

    memcpy(&message, buffer.data(), serial::HEADER_SIZE);

    if (message.header.start_byte != serial::START_FRAME) {
        throw std::runtime_error("Start frame not recognized");
    }

    uint8_t* payload =
        reinterpret_cast<uint8_t*>(&message) + serial::HEADER_SIZE;

    if (message.header.data_len > buffer.size() - serial::HEADER_SIZE) {
        throw std::runtime_error("Incomplete buffer");
    }

    memcpy(payload, buffer.data() + serial::HEADER_SIZE,
           message.header.data_len);

    return message;
}

void SerialSpinner::callbackTarget(const serial::TargetConstPtr& target) {
    using namespace serial::msg;
    OutgoingMessage order{.target_order = {}};

    TargetOrder& msg = order.target_order;

    msg.pitch = target->theta;
    msg.yaw = target->phi;

    std::cout << "Target   : " << msg.pitch << ' ' << msg.yaw << '\n';

    sendMessage(order);
}

void SerialSpinner::callbackMovement(const serial::MovementConstPtr& move) {
    using namespace serial::msg;
    OutgoingMessage order{.move_order = {}};

    Move& msg = order.move_order;

    msg.v_x = utils::toMillimeter(move->v_x);
    msg.v_y = utils::toMillimeter(move->v_y);
    msg.omega = utils::toAngularSpeed(move->omega);

    std::cout << "Movement : " << msg.v_x << ' ' << msg.v_y << ' ' << msg.omega
              << '\n';

    sendMessage(order);
}

void SerialSpinner::callbackShoot(const serial::ShootConstPtr&) {
    using namespace serial::msg;
    OutgoingMessage order{.shoot_order = {}};

    std::cout << "Shooting\n";
    sendMessage(order);
}

void SerialSpinner::sendMessage(const serial::msg::OutgoingMessage& message) {
    auto msg_size = message.header.size();

    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&message);

    int bytes = write(fd, ptr, message.header.size());
    if (bytes != msg_size) {
        ROS_ERROR("Could not write to serial : %s", strerror(errno));
    }
}

std::vector<uint8_t>
SerialSpinner::serializeMessage(const serial::msg::OutgoingMessage& message) {
    auto msg_size = message.header.size();

    std::vector<uint8_t> vec(msg_size, 0u);

    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&message);

    for (auto i = 0u; i < msg_size; ++i) {
        vec[i] = ptr[i];
        std::cout << std::hex << static_cast<unsigned int>(ptr[i]) << ' ';
    }

    std::cout << std::dec << '\n';

    return vec;
}
