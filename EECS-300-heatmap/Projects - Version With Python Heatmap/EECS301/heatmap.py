import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Configuration ---
PORT = '/dev/cu.usbserial-0001'
BAUD_RATE = 921600  # Matches Serial.begin(115200) in thermal.cpp
ROWS = 24
COLS = 32
TOTAL_PIXELS = ROWS * COLS

print("Opening serial port...")
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=0.01)
    # Release both DTR and RTS so macOS does not hold the ESP32 in reset or bootloader mode
    ser.dtr = False
    ser.rts = False
    print("Serial port opened! Waiting for ESP32 boot & calibration (~10s)...")
except Exception as e:
    print(f"Error opening serial port {PORT}: {e}")
    exit()

# --- Figure Setup (matches MATLAB script) ---
fig, ax = plt.subplots(figsize=(8, 6))
try:
    fig.canvas.manager.set_window_title('Thermal Camera')
except Exception:
    pass

temp_matrix = np.zeros((ROWS, COLS))
heatmap = ax.imshow(temp_matrix, cmap='hot', vmin=20, vmax=36, aspect='equal', interpolation='nearest')
fig.colorbar(heatmap, ax=ax, label='Temperature (°C)')

ax.set_xlabel('column (left = exit, right = enter)')
ax.set_ylabel('row')

# Stats read from ESP32 serial output (unknown / NaN until received)
count = np.nan
n_blobs = np.nan
n_hot = np.nan
baseline = np.nan
thr = np.nan
contour_set = None

title = ax.set_title("Waiting for ESP32 calibration... (~10s)")

serial_buffer = ""
start_time = time.time()
received_any_data = False
warned = False

def update(frame):
    global count, n_blobs, n_hot, baseline, thr, contour_set, serial_buffer, received_any_data, warned
    latest_matrix = None

    try:
        # Read directly without relying on ser.in_waiting (which can return 0 on macOS)
        raw_bytes = ser.read(4096)
        if raw_bytes:
            received_any_data = True
            serial_buffer += raw_bytes.decode('utf-8', errors='ignore')

        # Prevent lag if buffer grows too large
        if len(serial_buffer) > 30000:
            serial_buffer = serial_buffer[-8000:]

        if not received_any_data and not warned and (time.time() - start_time > 4.0):
            warned = True
            print("No bytes received yet — press the EN/RST button on the ESP32 once.")

        while '\n' in serial_buffer:
            line, serial_buffer = serial_buffer.split('\n', 1)
            msg = line.strip()
            if not msg:
                continue

            if msg.startswith("FRAME,"):
                parts = msg.split(',')[1:]
                if len(parts) == TOTAL_PIXELS:
                    try:
                        temp_array = np.array([float(v) for v in parts], dtype=float)
                        finite_vals = temp_array[np.isfinite(temp_array)]
                        fill_val = np.median(finite_vals) if finite_vals.size > 0 else 20.0
                        temp_array[~np.isfinite(temp_array)] = fill_val
                        latest_matrix = temp_array.reshape((ROWS, COLS))
                    except ValueError:
                        pass
            elif msg.startswith("Count:"):
                try:
                    count = float(msg.split("Count:", 1)[1].strip())
                except ValueError:
                    pass
            elif msg.startswith("Number of blobs:"):
                try:
                    n_blobs = float(msg.split("Number of blobs:", 1)[1].strip())
                except ValueError:
                    pass
            elif msg.startswith("Hot pixels count:"):
                try:
                    n_hot = float(msg.split("Hot pixels count:", 1)[1].strip())
                except ValueError:
                    pass
            elif msg.lower().startswith("baseline:"):
                try:
                    baseline = float(msg.split(":", 1)[1].strip())
                except ValueError:
                    pass
            elif msg.startswith("Threshold:"):
                try:
                    thr = float(msg.split("Threshold:", 1)[1].strip())
                except ValueError:
                    pass
            else:
                print(f"[ESP32] {msg}")

        if latest_matrix is not None:
            heatmap.set_data(latest_matrix)

            if contour_set is not None:
                try:
                    contour_set.remove()
                except AttributeError:
                    for coll in contour_set.collections:
                        coll.remove()
                contour_set = None

            if not np.isnan(thr) and np.min(latest_matrix) < thr < np.max(latest_matrix):
                contour_set = ax.contour(latest_matrix, levels=[thr], colors='c', linewidths=1.5)

            title.set_text(
                f"Count: {count:g}   Blobs: {n_blobs:g}   Hot pixels: {n_hot:g}   Baseline: {baseline:.2f}   Threshold: {thr:.2f}"
            )

    except serial.SerialException as e:
        print(f"Serial read error: {e}")

    return (heatmap, title)

ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()
ser.close()