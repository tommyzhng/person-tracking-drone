import os

# Connect to the device using ADB
os.system("adb connect <device-ip-address>")

# Tap the center of the screen
os.system("adb shell input tap 500 500")

# Swipe from the center of the screen to the top-left corner
os.system("adb shell input swipe 500 500 100 100")

# Disconnect from the device
os.system("adb disconnect")