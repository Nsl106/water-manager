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
pixels = neopixel.NeoPixel(LED_PIN, NUM_PIXELS, brightness=0.3, auto_write=False)


def get_distance():
    # Trigger pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG, False)

    # Wait for Echo start
    start_time = time.time()
    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    # Wait for Echo end
    stop_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    # Calculate distance
    elapsed = stop_time - start_time
    distance_cm = (elapsed * 34300) / 2  # Speed of sound = 343 m/s
    return distance_cm

avg_filter = deque(maxlen=20)

max_reading = 20
target_reading = 15

try:
    while True:
        dist = get_distance()

        avg_filter.append(min(dist, max_reading))

        if len(avg_filter) != avg_filter.maxlen:
            pass

        average = sum(avg_filter) / len(avg_filter)
        print(f"Average: {average :.2f} cm")

        if dist > max_reading:
            pixels.fill((255, 255, 0))
        elif average > 15:
            pixels.fill((255, 0, 0))
        else:
            pixels.fill((0, 255, 0))

        pixels.show()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Measurement stopped by user")
    GPIO.cleanup()
