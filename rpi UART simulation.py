import serial, time

ser = serial.Serial("COM4", 115200, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

def send(cmd: str):
    msg = (cmd + "\r\n").encode()
    print("TX:", repr(msg))
    ser.write(msg)
    ser.flush()
    time.sleep(0.2)
    while ser.in_waiting:
        print("RX:", ser.readline().decode(errors="ignore").strip())

#send("OLED:Hello STM32")
send("TURNL900")


# import serial, time

# ser = serial.Serial('COM4', 115200, timeout=1)
# time.sleep(1)

# ser.write(b"OLED:Hello STM32\r\n")
# time.sleep(0.1)

# while ser.in_waiting:
#     print("RX:", ser.readline().decode(errors="ignore").strip())

