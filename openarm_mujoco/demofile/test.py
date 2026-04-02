import can
import time
import math

bus_r = can.interface.Bus(channel='can0', bustype='socketcan')
bus_l = can.interface.Bus(channel='can1', bustype='socketcan')

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    offset = x - x_min
    return int((offset * ((1 << bits) - 1)) / span)

def pack_mit(p, v=0.0, kp=40.0, kd=2.0, t=0.0):
    P_MIN, P_MAX = -12.5, 12.5
    V_MIN, V_MAX = -30.0, 30.0
    KP_MIN, KP_MAX = 0.0, 500.0
    KD_MIN, KD_MAX = 0.0, 5.0
    T_MIN, T_MAX = -18.0, 18.0

    p_int  = float_to_uint(p,  P_MIN,  P_MAX,  16)
    v_int  = float_to_uint(v,  V_MIN,  V_MAX,  12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int  = float_to_uint(t,  T_MIN,  T_MAX,  12)

    data = bytearray(8)
    data[0] = (p_int >> 8) & 0xFF
    data[1] = p_int & 0xFF
    data[2] = (v_int >> 4) & 0xFF
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
    data[4] = kp_int & 0xFF
    data[5] = (kd_int >> 4) & 0xFF
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
    data[7] = t_int & 0xFF

    return data

def enable(bus, motor_id):
    msg = can.Message(
        arbitration_id=motor_id,
        data=[0xFF]*7 + [0xFC],
        is_extended_id=False
    )
    bus.send(msg)

def send(bus, motor_id, pos):
    msg = can.Message(
        arbitration_id=motor_id,
        data=pack_mit(pos),
        is_extended_id=False
    )
    bus.send(msg)

# Enable両腕
for i in range(1, 9):
    enable(bus_r, i)
    enable(bus_l, i)
    time.sleep(0.02)

# 🎯 30度
angle = math.pi / 6  # 0.5236 rad

# 🔥 ID1〜8を順番に動かす
for motor_id in range(1, 9):

    print(f"Motor {motor_id} → 30°")

    # 右腕
    send(bus_r, motor_id, angle)

    # 左腕（ミラーなら -angle）
    send(bus_l, motor_id, -angle)

    time.sleep(0.5)

    # 元に戻す
    send(bus_r, motor_id, 0.0)
    send(bus_l, motor_id, 0.0)

    time.sleep(0.5)

bus_r.shutdown()
bus_l.shutdown()
