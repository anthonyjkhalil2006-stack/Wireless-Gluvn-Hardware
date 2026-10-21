import serial, time
s = serial.Serial(); s.port, s.baudrate = "COM4", 115200
s.dtr = False; s.rts = False
s.open()
name = f"log_{int(time.time())}.csv"
with open(name, "w") as f:
    print("Logging to", name, "- Ctrl+C to stop")
    try:
        while True: f.write(s.readline().decode(errors="ignore"))
    except KeyboardInterrupt: print("Saved", name)
