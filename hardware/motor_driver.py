"""Hiwonder 4채널 인코더 모터드라이버 (I2C 0x34) 실제 제어 코드.

레지스터 맵은 car_move_demo.ino에서 그대로 가져옴(검증된 값).
믹싱 행렬은 그 .ino의 4가지 캔드무브(전진/우횡이동/우회전/좌전대각선) 패턴을
역산해서 도출 - 4개 독립 케이스 모두 일치 확인됨.

라즈베리파이에서: pip install smbus2
"""
import time
try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None  # 라즈베리파이 밖(시뮬/드라이런)에서는 smbus2 없이도 import 가능해야 함

I2C_ADDR = 0x34
MOTOR_TYPE_ADDR = 20
MOTOR_ENCODER_POLARITY_ADDR = 21
MOTOR_FIXED_SPEED_ADDR = 51
MOTOR_ENCODER_TOTAL_ADDR = 60
MOTOR_TYPE_JGB = 3

# 메카넘 믹싱 행렬 (motor[i] = SPEED_SCALE * (a[i]*vx + b[i]*vy + c[i]*omega))
# ino의 forward/strafe-right/turn-right 패턴에서 역산, 대각선 이동으로 교차검증됨.
MIX_A = (1, -1, -1, 1)   # vx(전진) 계수
MIX_B = (1, 1, 1, 1)     # vy(우횡이동) 계수
MIX_C = (1, -1, 1, -1)   # omega(시계방향 제자리회전) 계수

WHEEL_R = 0.03895                  # m, 메카넘 휠 반경
PULSES_PER_REV = 44 * 131          # JGB 인코더(44 pulse/rev) x 감속비 131:1 = 5764
                                    # ※ 이론값. 기어 백래시/실제 감속비 오차가 있을 수 있으니
                                    #   가능하면 "10바퀴 수동회전 후 펄스 수 확인"으로 재검증할 것.
WHEEL_CIRCUM = 2 * 3.141592653589793 * WHEEL_R
M_PER_PULSE = WHEEL_CIRCUM / PULSES_PER_REV

MAX_SPEED_UNIT = 50   # 레지스터 51 유효범위 대략 ±50 (펄스/10ms), 하드웨어 문서 기준


class MecanumDriver:
    def __init__(self, bus_num=1):
        self.bus = SMBus(bus_num)
        time.sleep(0.05)
        self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_TYPE_ADDR, [MOTOR_TYPE_JGB])
        time.sleep(0.005)
        self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_ENCODER_POLARITY_ADDR, [0])
        time.sleep(0.1)

    def _to_i8(self, v):
        v = max(-128, min(127, int(round(v))))
        return v & 0xFF

    def set_body_velocity(self, vx, vy, omega):
        """vx,vy,omega: -1.0~1.0 정규화 입력. vx=전진, vy=우측횡이동, omega=시계방향회전."""
        speeds = []
        for a, b, c in zip(MIX_A, MIX_B, MIX_C):
            s = (a * vx + b * vy + c * omega) * MAX_SPEED_UNIT
            speeds.append(self._to_i8(s))
        self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, speeds)

    def stop(self):
        self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, [0, 0, 0, 0])

    def read_battery_mv(self):
        data = self.bus.read_i2c_block_data(I2C_ADDR, 0, 2)
        return data[0] | (data[1] << 8)

    def drive_time(self, vx, vy, omega, seconds):
        self.set_body_velocity(vx, vy, omega)
        time.sleep(seconds)
        self.stop()

    def estimate_speed_mps(self, vx, vy):
        """MAX_SPEED_UNIT 기준 대략적 병진 속도 추정 (검증 필요한 이론값)."""
        n = (vx ** 2 + vy ** 2) ** 0.5
        pulses_per_10ms = MAX_SPEED_UNIT * n
        return pulses_per_10ms * 100 * M_PER_PULSE  # pulses/s * m/pulse

    def close(self):
        self.stop()
        self.bus.close()
