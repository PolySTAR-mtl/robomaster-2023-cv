/** \file serialize.cpp
 * \brief Test serialization of messages
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#include "serial_spinner.hpp"

#include <gtest/gtest.h>

inline void check_array(std::vector<uint8_t>& obtained, uint8_t* expected) {
    for (auto i = 0u; i < obtained.size(); ++i) {
        EXPECT_EQ(expected[i], obtained[i]);
    }
}

TEST(Serialize, EmptyHeader) {
    uint8_t expected[] = {
        0xFC,
        0xFF,
        0x00,
    };

    serial::msg::OutgoingMessage msg{.header = {}};

    auto obtained = SerialSpinner::serializeMessage(msg);

    EXPECT_EQ(sizeof(msg.header), 3);
    EXPECT_EQ(msg.header.size(), 3);

    check_array(obtained, expected);
}

TEST(Serialize, TargetOrder) {
    // Coordinates (0, 0)

    uint8_t expected[] = {
        0xFC, 0x10, 4, 0x00, 0x00, 0x00, 0x00,
    };

    serial::msg::OutgoingMessage msg{.target_order = {}};

    msg.target_order.pitch = 0;
    msg.target_order.yaw = 0;

    auto obtained = SerialSpinner::serializeMessage(msg);

    EXPECT_EQ(sizeof(expected), obtained.size());
    check_array(obtained, expected);

    // Coordinates (1, 1)

    uint8_t expected2[] = {
        0xFC, 0x10, 4, 0xe8, 0x03, 0xe8, 0x03,
    };

    serial::msg::OutgoingMessage msg2{.target_order = {}};
    msg2.target_order.pitch = 0x03e8;
    msg2.target_order.yaw = 0x03e8;
    auto obtained2 = SerialSpinner::serializeMessage(msg2);

    EXPECT_EQ(sizeof(expected2), obtained2.size());
    check_array(obtained2, expected2);
}

TEST(Serialize, MoveOrder) {
    // Coordinates (0, 0, 0)

    uint8_t expected[] = {0xFC, 0x11, 6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    serial::msg::OutgoingMessage msg{.move_order = {}};

    msg.move_order.v_x = 0;
    msg.move_order.v_y = 0;
    msg.move_order.omega = 0;

    auto obtained = SerialSpinner::serializeMessage(msg);

    EXPECT_EQ(sizeof(expected), obtained.size());
    check_array(obtained, expected);
}

TEST(Serialize, Shoot) {
    uint8_t expected[] = {
        0xFC,
        0x12,
        0x00,
    };

    serial::msg::OutgoingMessage msg{.shoot_order = {}};

    auto obtained = SerialSpinner::serializeMessage(msg);

    EXPECT_EQ(sizeof(expected), obtained.size());
    check_array(obtained, expected);
}

TEST(Serialize, HeaderSize) {
    EXPECT_EQ(serial::HEADER_SIZE, sizeof(serial::None));
    EXPECT_EQ(serial::HEADER_SIZE, sizeof(serial::Header<void>));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "serial_tester");
    return RUN_ALL_TESTS();
}
