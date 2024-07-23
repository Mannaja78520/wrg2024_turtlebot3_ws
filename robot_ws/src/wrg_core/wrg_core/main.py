import time
from pynput.mouse import Button, Controller

# Create a mouse controller
mouse = Controller()

try:
    while True:
        # Perform a right-click
        mouse.click(Button.left, 1)
        # Wait for 1 second
        time.sleep(10)
except KeyboardInterrupt:
    # Exit the loop when interrupted
    print("Right-clicking stopped.")
