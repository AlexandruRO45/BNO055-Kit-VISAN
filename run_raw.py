import time
import board
import busio
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x29)

# Curăță ecranul o singură dată la start
print("\033[2J", end="")

while True:
    euler = sensor.euler or (0, 0, 0)
    quat = sensor.quaternion or (0, 0, 0, 0)
    acc = sensor.linear_acceleration or (0, 0, 0)
    cal = sensor.calibration_status

    # Mută cursorul sus-stânga și suprascrie
    print("\033[H", end="")
    print(f"Euler  : heading={euler[0]:7.2f}  roll={euler[1]:7.2f}  pitch={euler[2]:7.2f}   ")
    print(f"Quat   : w={quat[0]:6.3f}  x={quat[1]:6.3f}  y={quat[2]:6.3f}  z={quat[3]:6.3f}   ")
    print(f"LinAcc : x={acc[0]:7.3f}  y={acc[1]:7.3f}  z={acc[2]:7.3f}   ")
    print(f"Calibr : sys={cal[0]} gyro={cal[1]} acc={cal[2]} mag={cal[3]}   ")
    print("\n(Ctrl+C pentru oprire acest ecran rămâne fix. CTRL+C)")
    time.sleep(0.05)  # ~20Hz refresh, suficient pentru citire umană
