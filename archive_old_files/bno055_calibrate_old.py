#!/usr/bin/env python3
import json, time
import board, busio
import adafruit_bno055

CAL_FILE = "bno055_cal.json"

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x29)

print("\033[2J", end="")  # clear o singura data
while True:
    sys, gyro, accel, mag = sensor.calibration_status
    print("\033[H", end="")
    print(f"status: sys={sys} gyro={gyro} accel={accel} mag={mag}")
    print(f"euler : {sensor.euler}")
    print("\ngyro: nemiscat | accel: 6 fete | mag: figura 8")
    print("target: 3 3 3 3 (CTRL+C pentru renuntare)")
    if (sys, gyro, accel, mag) == (3, 3, 3, 3):
        break
    time.sleep(0.4)

# nu misca placa intre acest punct si scriere
cal = {
    "accel_offset": list(sensor.offsets_accelerometer),
    "mag_offset":   list(sensor.offsets_magnetometer),
    "gyro_offset":  list(sensor.offsets_gyroscope),
    "accel_radius": sensor.radius_accelerometer,
    "mag_radius":   sensor.radius_magnetometer,
}
with open(CAL_FILE, "w") as f:
    json.dump(cal, f, indent=2)

print("\nSalvat in", CAL_FILE)
