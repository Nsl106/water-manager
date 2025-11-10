import RPi.GPIO as GPIO
import time
import board
import neopixel

from collections import deque

# Pin setup
TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# --- NeoPixel setup ---
LED_PIN = board.D18  # Make sure this matches your wiring
NUM_PIXELS = 8
pixels = neopixel.NeoPixel(LED_PIN, NUM_PIXELS, brightness=0.75, auto_write=False)


def wheel(pos):
    if pos < 85:
        return (int(pos * 3), int(255 - pos * 3), 0)
    elif pos < 170:
        pos -= 85
        return (int(255 - pos * 3), 0, int(pos * 3))
    else:
        pos -= 170
        return (0, int(pos * 3), int(255 - pos * 3))


def get_distance():
    # Trigger pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG, False)

    # Wait for Echo start
    start_time = time.time()
    initial_start_time = start_time

    while GPIO.input(ECHO) == 0:
        if (time.time() - initial_start_time) > 0.1:
            print("Echo Start Timeout")
            return None
        start_time = time.time()

    # Wait for Echo end
    stop_time = time.time()
    initial_stop_time = stop_time

    while GPIO.input(ECHO) == 1:
        if (time.time() - initial_stop_time) > 0.1:
            print("Echo End Timeout")
            return None
        stop_time = time.time()

    # Calculate distance
    elapsed = stop_time - start_time
    distance_cm = (elapsed * 34300) / 2  # Speed of sound = 343 m/s
    return distance_cm


avg_filter = deque(maxlen=50)

max_reading = 40
min_water_level = 28

red_blink_timer = 0
offset = 0

try:
    while True:
        dist = get_distance()

        if dist is None:
            time.sleep(0.01)
            continue

        avg_filter.append(min(dist, max_reading))

        if len(avg_filter) != avg_filter.maxlen:
            time.sleep(0.01)
            continue

        average = sum(avg_filter) / len(avg_filter)

        if dist > max_reading:
            pixels.fill((255, 255, 0))
        elif average < min_water_level:
            # pixels.fill((0, 255, 0))
            for i in range(NUM_PIXELS):
                color_index = (i * 32 + offset) % 256  # 32 spreads colors evenly across 8 LEDs
                pixels[i] = wheel(color_index)
            offset = (offset + 8) % 256

        else:
            if (red_blink_timer == 3):
                pixels.fill((255, 0, 0))
                red_blink_timer = 0
            else:
                pixels.fill((0, 0, 0))
                red_blink_timer += 1

        # print(f"Average: {average :.2f} cm")

        pixels.show()

        time.sleep(0.01)

except KeyboardInterrupt:
    print("Measurement stopped by user")
finally:
    GPIO.cleanup()
