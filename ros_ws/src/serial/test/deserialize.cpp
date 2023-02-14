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

TEST(Deserialize, Status) {}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "serial_tester");
    return RUN_ALL_TESTS();
}
