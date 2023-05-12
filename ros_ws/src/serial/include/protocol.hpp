/** \file protocol.cpp
 * \brief Serial protocol definition
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

// Std includes

#include <cstddef>
#include <cstdint>

#include <variant>

namespace serial {

// Constants

constexpr uint8_t START_FRAME = 0xFCu;

template <typename T> struct Header {
    const uint8_t start_byte = START_FRAME;
    const uint8_t cmd_id = T::ID;
    volatile uint8_t data_len =
        sizeof(T) - sizeof(Header); // Marked as volatile as to ensure no
                                    // optimzation from the compiler

    std::size_t size() const { return data_len + sizeof(Header); }
} __attribute__((packed, aligned(1)));

struct None : Header<None> {
    static constexpr uint8_t ID = 0xFF;
} __attribute__((packed));

constexpr std::size_t HEADER_SIZE = sizeof(None);

// Payloads

namespace msg {

struct Status : Header<Status> {
    static constexpr uint8_t ID = 0x01;

    uint8_t robot_type;
    uint16_t red_std_hp;
    uint16_t red_hro_hp;
    uint16_t red_sty_hp;
    uint16_t blu_std_hp;
    uint16_t blu_hro_hp;
    uint16_t blu_sty_hp;
    uint8_t mode;
} __attribute__((packed));

struct Gamestage : Header<Gamestage> {
    static constexpr uint8_t ID = 0x02;

    uint16_t gamestage;
} __attribute__((packed));

struct TurretFeedback : Header<TurretFeedback> {
    static constexpr uint8_t ID = 0x03;

    int16_t pitch;
    int16_t yaw;
} __attribute__((packed));

struct PositionFeedback : Header<PositionFeedback> {
    static constexpr uint8_t ID = 0x04;

    int16_t imu_ax;
    int16_t imu_ay;
    int16_t imu_az;
    int16_t imu_gx;
    int16_t imu_gy;
    int16_t imu_gz;
    int16_t imu_rx;
    int16_t imu_ry;
    int16_t imu_rz;
    uint16_t enc_1;
    int16_t enc_1_revolutions;
    uint16_t enc_2;
    int16_t enc_2_revolutions;
    uint16_t enc_3;
    int16_t enc_3_revolutions;
    uint16_t enc_4;
    int16_t enc_4_revolutions;
    int16_t v_enc_1;
    int16_t v_enc_2;
    int16_t v_enc_3;
    int16_t v_enc_4;
    int8_t padding;
} __attribute__((packed));

struct TargetOrder : Header<TargetOrder> {
    static constexpr uint8_t ID = 0x10;

    int16_t pitch;
    int16_t yaw;
} __attribute__((packed));

struct Move : Header<Move> {
    static constexpr uint8_t ID = 0x11;

    int16_t v_x;
    int16_t v_y;
    int16_t omega;
} __attribute__((packed));

struct Shoot : Header<Shoot> {
    static constexpr uint8_t ID = 0x12;
} __attribute__((packed));

union IncomingMessage {
    Header<None> header;
    Status status;
    Gamestage gamestage;
    TurretFeedback turret_feedback;
    PositionFeedback position_feedback;
};

union OutgoingMessage {
    Header<None> header;
    TargetOrder target_order;
    Move move_order;
    Shoot shoot_order;
};

} // namespace msg

} // namespace serial
