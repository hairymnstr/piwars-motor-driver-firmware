# Motor driver board firmware

This is the firmware to run the motor board described in the hardware repository: <https://github.com/hairymnstr/piwars-motor-driver-hardware>.

The firmware allows you to control one DC motor and one servo from an RS485 bus.

## Protocol

A very simple protocol is used:

    <ADDRESS><COMMAND><PARAMETERS><CHECKSUM>

The UART is configured in 9 bit mode.  When the 9th bit is set, the content of the lower 8 bits is considered to be an address.  When an address is received the receiver state machine is reset.

The first byte after a receiver reset (i.e. an address byte) is a command.  Each command may then have fixed length parameters (i.e. fixed for that command but not necessarily the same as another command).

Finally a simple XOR checksum is included.

## Addressing

Addressing is via a bit-mask.  The address of the board is set by jumpers J5, J2 and J1 with bit values 4, 2 and 1 respectively.  If the address of the board is 0 (all three jumpers shorted low) then the address mask is (1<<0) == 1.  If the address is set to 3 (J2 and J1 low, J5 high) then the address mask is (1 << 3) == 8.  When an address is received it is subjected to a bit-wise AND with the address mask, if the result is non-zero the board will carry out the command.

So, address byte 3 (0b00000011) will cause both motor board 0 (address mask 0b00000001) and motor board 1 (address mask 0b00000010) to carry out the command.

This addressing scheme makes it much simpler (and faster) to do global operations like speed up or slow down.

## Commands

| Number | Meaning            | Parameters                                                                                                         |
| ----   | ----               | ----                                                                                                               |
| 10     | Set motor speed    | 1 signed byte, +/-100 %                                                                                            |
| 11     | Set servo angle    | 1 signed short in tenths degrees added to the trim setting to make an angle between 0 and 1800                     |
| 12     | Set motor reversed | 1 byte 0=normal, 1=reversed                                                                                        |
| 13     | Set servo reversed | 1 byte 0=normal, 1=reversed                                                                                        |
| 14     | Set servo trim     | 1 signed short 0-1800, sets centre point of servo sweep.  Doesn't change servo position until the next command 11. |

Using this command set each wheel on our robot could be set up with a unique trim setting and forward/reverse setting so each corner drives in the same direction despite not being mirrored.  Then all 4 wheels can be set to a speed or angle together to drive/steer.
