# Serial link with the Control & Systems team

The serial protocol is defined at the end of the present document.

The serial settings are as followed (and described in `ros_ws/data/serial.yaml`) :

- Device : `/dev/ttyTHS0`
- Length : 8 bits
- Baud rate : 230 400 baud
- 1 stop bit
- No parity bit

## Topics & mapping

The `serial_interface` node **listens** to the following topics : 

- `/serial/target` : Coordinates of the current target, to be passed to the turret;

The `serial_interface` node **publishes** to the following topics :

- `/serial/switch` : Order coming from the pilot to switch target left/right;
- `/serial/hp` : Health points of the robots;

The corresponding messages are defined in the serial package (`ros_ws/src/serial/msg`).

`TODO` : Add position, reference frame publishing topics

## Jetson setup

Check the pins for connections: https://jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/. Make sure that RX cable goes on TX pin (8) of the Jetson and TX cable on RX pin (10) of the Jetson. Add the ground (GND) on the pin 9.

The Tegra High speed Serial (THS) device is reserved for root users, so a chmod (`sudo chmod 777 /dev/ttyTHS0`) is
necessary (at each reboot) in order to use the serial node.

By default, the port is used as a serial terminal. The service has to be disabled
in order to allow the port to be used for communication. It should be the case on the Jetson, but you can check
by using: `systemctl status nvgetty`. If the command show service is "Active" you have to do the following:

```bash
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
```

Followed by a reboot.

# New serial protocol

## Configuration

Board A : UART 7

Jetson : UART 2 (pins 8 (TX) and 10 (RX))

230400-8N1 - Little endian

8 bits 230400 baud 1 stop bit & no parity bit

## Protocol

General message layout

|   SOF  | `cmd_id`  | `data_len` | `data ` |
|--------|-----------|------------|---------|
| 1 byte |  1 byte  |  1 byte    |         |
| Fixed value : `0xFC` |      |   Length of `data`, in bytes |       |

`cmd_id` possible values : 

CS -> CV

| Code    |     Description      |  `data_len` |
|---------|----------------------|-------------|
| `0x01`  | Game status          | 14          |
| `0x02`  | Event (taproot: gamestage)   | 2        |
| `0x03`  | Turret feedback      | 4            |
| `0x04`  | Position feedback    | 22           |

CV -> CS

| Code    |     Description      |  `data_len` |
|---------|----------------------|-------------|
| `0x10`  | Turret target        | 4           |
| `0x11`  | Movements            | 6           |
| `0x12`  | Shoot order          | 0 (no data) |

## Commands

### Game status

Value : `0x01`

Frequency : 1 Hz, or caused by a control mode change

Size : 14 bytes

Contents :
- Robot type
- Team
- HP (Twice, for each team)
    - STD 16 bits
    - HRO 16 bits
    - STY 16 bits
- Control mode

| Offset  | Size | Desc        |
|---------|------|-------------|
| 0       | 1    | Robot type : 0 -> STD, 1 -> HRO, 2 -> STY. MSB : Team (0b0 : Red, 0b1 : Blue) |
| 1       | 2    | Red STD HP   |
| 3       | 2    | Red HRO HP   |
| 5       | 2    | Red STY HP   |
| 7       | 2    | Blue STD HP  |
| 9       | 2    | Blue HRO HP   |
| 11      | 2    | Blue STY HP   |
| 13      | 1    | Mode : 0 -> Manual, 1 -> Auto aim, 2 -> Auto shoot, 3 -> Full auto |

### Event

Value : `0x02`

Frequency : at each change

Size : 2 bytes

Contents : Taproot event gamestage

| Offset  | Size | Desc        |
|---------|------|-------------|
| 0       | 2    | enum GameStage value |


### Turret Feedback

Value : `0x03`

Frequency : 100 Hz

Size : 4 bytes

Contents : Turret position
- Encoder value in millirad (`int16_t`)
- Reference frame : Straight ahead

| Offset  | Size | Desc          |
|---------|------|---------------|
| 0       | 2    | Pitch (theta) |
| 2       | 2    | Yaw (phi)     |

### Position Feedback

Value : `0x04`

Frequency : 100 Hz

Size : 22 bytes

Contents : Feedback from the sensors

Units :
- Linear accel : mm . s^-2, `int16_t`
- Angular speed : millirad . s^-1, `int16_t`
- Encoders : N/A, `uint16_t` (wrapped), `int16_t` (revolutions)
- Encoders (speed) : N/A, `int16_t`
- \Delta_t : microseconds (10^-6 s) , `uint16_t`

| Offset  | Size | Desc        |
|---------|------|-------------|
| 0       | 2    | IMU A_x     |
| 2       | 2    | IMU A_y     |
| 4       | 2    | IMU A_z     |
| 6       | 2    | IMU G_x     |
| 8       | 2    | IMU G_y     |
| 10      | 2    | IMU G_z     |
| 12      | 2    | IMU R_x     |
| 14      | 2    | IMU R_y     |
| 16      | 2    | IMU R_z     |
| 18      | 2    | Enc. 1 wrap.|
| 18      | 2    | Enc. 1 rev. |
| 26      | 2    | Enc. 2 wrap.|
| 26      | 2    | Enc. 2 rev. |
| 34      | 2    | Enc. 3 wrap.|
| 34      | 2    | Enc. 3 rev. |
| 42      | 2    | Enc. 4 wrap.|
| 42      | 2    | Enc. 4 rev. |
| 50      | 2    | V Enc. 1    |
| 52      | 2    | V Enc. 2    |
| 54      | 2    | V Enc. 3    |
| 56      | 2    | V Enc. 4    |

### Turret target

Value : `0x10`

Frequency : As fast as possible (~30 Hz)

Size : 4 bytes

Contents : Current turret target from the targeting system, already solved
for distance (ballistics)

Units & reference frame : see Turret feedback.

| Offset  | Size | Desc          |
|---------|------|---------------|
| 0       | 2    | Pitch (theta) | 
| 2       | 2    | Yaw (phi)     |

### Movements

Value : `0x11`

Frequency : TBD

Size : 6 bytes

Contents : Speed control when in full auto mode

Units :
- Linear speed : mm . s^-1
- Rotation : millirad . s^-1

| Offset  | Size | Desc        |
|---------|------|-------------|
| 0       | 2    | V_x         |
| 2       | 2    | V_y         |
| 4       | 2    | Omega       |
