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

#include "serial/HP.h"
#include "serial/SwitchOrder.h"

// OS includes

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

SerialSpinner::SerialSpinner(ros::NodeHandle& n, const std::string& device,
                             int _baud, int _len, int _stop, bool _parity,
                             double _freq)
    : nh(n), baud_rate(_baud), length(_len), stop_bits(_stop), parity(_parity),
      frequency(_freq) {
    initSerial(device);

    pub_hp = nh.advertise<serial::HP>("hp", 1);
    pub_switch = nh.advertise<serial::SwitchOrder>("switch", 1);

    sub_target =
        nh.subscribe("target", 1, &SerialSpinner::callbackTarget, this);
    sub_movement =
        nh.subscribe("movement", 1, &SerialSpinner::callbackMovement, this);
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
void SerialSpinner::handleMessage<serial::Status>(
    const serial::Status& status) {
    // TODO
}

template <>
void SerialSpinner::handleMessage<serial::Gamestage>(
    const serial::Gamestage& gamestage) {
    // TODO
}

template <>
void SerialSpinner::handleMessage<serial::TurretFeedback>(
    const serial::TurretFeedback& turret_feedback) {
    // TODO
}

template <>
void SerialSpinner::handleMessage<serial::PositionFeedback>(
    const serial::PositionFeedback& position_feedback) {
    // TODO
}

void SerialSpinner::handleSerial() {
    int bytes;

    serial::IncomingMessage message{serial::Header<serial::None>()};

    // Attempt to read a command
    bytes = read(fd, &message, sizeof(serial::Header<serial::None>));
    if (bytes < sizeof(serial::Header<void>)) {
        // No incoming command, return immediatly
        return;
    }

    if (message.header.start_byte != serial::START_FRAME) {
        ROS_ERROR("Start frame not recognized, dropping command");
        return;
    }

    uint8_t* payload =
        reinterpret_cast<uint8_t*>(&message) + sizeof(serial::Header<void>);
    bytes = read(fd, payload, message.header.data_len);
    if (bytes < message.header.data_len) {
        ROS_ERROR("Incomplete read on input payload");
        return;
    }

    // Could use a std::visit-type method here, but this will do for now.
    switch (message.header.cmd_id) {
    case serial::Status::ID:
        handleMessage(message.status);
        break;
    case serial::Gamestage::ID:
        handleMessage(message.gamestage);
        break;
    case serial::TurretFeedback::ID:
        handleMessage(message.turret_feedback);
        break;
    case serial::PositionFeedback::ID:
        handleMessage(message.position_feedback);
        break;
    default:
        ROS_ERROR("Unknown message type %d", message.header.cmd_id);
    }
}

void SerialSpinner::callbackTarget(const serial::TargetConstPtr& target) {
    serial::OutgoingMessage order{};
    serial::TargetOrder& msg = order.target_order;
    msg.pitch = target->theta;
    msg.yaw = target->phi;

    std::cout << "Target   : " << msg.pitch << ' ' << msg.yaw << '\n';

    sendMessage(order);
}

constexpr int16_t toMillimeter(float m_s) {
    // m/s to mm/s
    return static_cast<uint16_t>(m_s * 1e3);
}

constexpr int16_t toAngularSpeed(float omega) {
    // Rad/s to millirad/s
    return static_cast<uint16_t>(omega * 1e3);
}

void SerialSpinner::callbackMovement(const serial::MovementConstPtr& move) {
    serial::OutgoingMessage order{};
    serial::Move& msg = order.move_order;
    msg.v_x = toMillimeter(move->v_x);
    msg.v_y = toMillimeter(move->v_y);
    msg.omega = toAngularSpeed(move->omega);

    std::cout << "Movement : " << msg.v_x << ' ' << msg.v_y << ' ' << msg.omega
              << '\n';

    sendMessage(order);
}

void SerialSpinner::sendMessage(const serial::OutgoingMessage& message) {
    auto msg_size = message.header.size();
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&message);
    for (auto i = 0u; i < msg_size; ++i) {
        std::cout << std::hex << static_cast<unsigned int>(ptr[i]) << ' ';
    }
    std::cout << std::dec << '\n';

    int bytes = write(fd, ptr, message.header.size());
    if (bytes != msg_size) {
        ROS_ERROR("Could not write to serial : %s", strerror(errno));
    }
}
