/** \file protocol.cpp
 * \brief Serial protocol definition
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

// Std includes

#include <cstddef>
#include <cstdint>

namespace serial {

// Constants

constexpr uint8_t START_FRAME = 0xFCu;

/** \enum cmd
 * \brief Possible commands
 */
enum class cmd : uint8_t {
    // CS -> CV
    Status = 0x01,
    Gamestage = 0x02,
    TurretFeedback = 0x03,
    PositionFeedback = 0x04u,

    // CV -> CS
    Target = 0x10,
    Move = 0x11,
    Shoot = 0x12
};

struct Header {
    const uint8_t start_byte = 0xFCu;
    const cmd cmd_id;
    const uint8_t data_len;

    template <typename T> static Header create() {
        return Header(T::cmd_id, sizeof(T));
    }

  private:
    Header(cmd id, uint8_t len) : cmd_id(id), data_len(len) {}
};

/** \class Message
 * \brief Generic message type
 */
template <typename T> class Message {
  public:
    size_t write(int fd);

  private:
    Header header;
    T payload;
};

// Payloads

struct Status {
    static constexpr cmd cmd_id = cmd::Status;

    uint8_t robot_type;
    uint16_t red_std_hp;
    uint16_t red_hro_hp;
    uint16_t red_sty_hp;
    uint16_t blu_std_hp;
    uint16_t blu_hro_hp;
    uint16_t blu_sty_hp;
    uint8_t mode;
} __attribute__((packed));

struct Gamestage {
    static constexpr cmd cmd_id = cmd::Gamestage;

    uint16_t gamestage;
} __attribute__((packed));

struct TurretFeedback {
    static constexpr cmd cmd_id = cmd::TurretFeedback;

    int16_t pitch;
    int16_t yaw;
} __attribute__((packed));

struct PositionFeedback {
    static constexpr cmd cmd_id = cmd::PositionFeedback;

    int16_t imu_ax;
    int16_t imu_ay;
    int16_t imu_az;
    int16_t imu_rx;
    int16_t imu_ry;
    int16_t imu_rz;
    int16_t enc_1;
    int16_t enc_2;
    int16_t enc_3;
    int16_t enc_4;
    int16_t delta_t;
} __attribute__((packed));

struct Target {
    static constexpr cmd cmd_id = cmd::Target;

    int16_t pitch;
    int16_t yaw;
} __attribute__((packed));

struct Move {
    static constexpr cmd cmd_id = cmd::Move;

    int16_t v_x;
    int16_t v_y;
    int16_t omega;
} __attribute__((packed));

struct Shoot {
    static constexpr cmd cmd_id = cmd::Shoot;
} __attribute__((packed));

} // namespace serial
