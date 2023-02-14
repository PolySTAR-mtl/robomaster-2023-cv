/** \file deserialize.cpp
 * \brief Test deserialization of messages (incoming messages)
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

TEST(Deserialize, EmptyHeader) {
    std::vector<uint8_t> buffer = {0xFC, 0xFF, 0x00};

    auto msg = SerialSpinner::deseralizeMessage(buffer);
    EXPECT_EQ(msg.header.start_byte, serial::START_FRAME);
    EXPECT_EQ(msg.header.cmd_id, buffer[1]);
    EXPECT_EQ(msg.header.data_len, 0);
}

TEST(Deserialize, Status) {
    std::vector<uint8_t> buffer = {0xFC, 0x01, 14, 0, 0, 0, 0, 0, 0,
                                   0,    0,    0,  0, 0, 0, 0, 0};
    auto msg = SerialSpinner::deseralizeMessage(buffer);

    EXPECT_EQ(msg.status.cmd_id, serial::msg::Status::ID);
    EXPECT_EQ(msg.status.robot_type, 0u);
    EXPECT_EQ(msg.status.red_std_hp, 0u);
    EXPECT_EQ(msg.status.red_hro_hp, 0u);
    EXPECT_EQ(msg.status.red_sty_hp, 0u);
    EXPECT_EQ(msg.status.blu_std_hp, 0u);
    EXPECT_EQ(msg.status.blu_hro_hp, 0u);
    EXPECT_EQ(msg.status.blu_sty_hp, 0u);
    EXPECT_EQ(msg.status.mode, 0u);

    std::vector<uint8_t> buffer2 = {0xFC, 0x01, 14, 0xFF, 0, 0, 0, 0,   0,
                                    0,    0,    0,  0,    0, 0, 0, 0xFF};
    auto msg2 = SerialSpinner::deseralizeMessage(buffer2);

    EXPECT_EQ(msg2.status.cmd_id, serial::msg::Status::ID);
    EXPECT_EQ(msg2.status.robot_type, 0xFF);
    EXPECT_EQ(msg2.status.red_std_hp, 0u);
    EXPECT_EQ(msg2.status.red_hro_hp, 0u);
    EXPECT_EQ(msg2.status.red_sty_hp, 0u);
    EXPECT_EQ(msg2.status.blu_std_hp, 0u);
    EXPECT_EQ(msg2.status.blu_hro_hp, 0u);
    EXPECT_EQ(msg2.status.blu_sty_hp, 0u);
    EXPECT_EQ(msg2.status.mode, 0xFF);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "serial_tester");
    return RUN_ALL_TESTS();
}
