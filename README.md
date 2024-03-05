Implements WiFi streaming for SSI spec for the IMU for the M5StickC and Recognition using a SensiML KnowledgePack

# Library Setup

This firmware uses the FIFO buffer to set the correct sample rate. If the FIFO buffer code for the MPU6886 is not part of the M5StickCPlus firmware you are using you will need to copy both the MPU6886 cpp and h files into the M5StickCPlus library code.

# Data Capture

This is an Arudino Sketch for the M5StickCPlus that enables streaming the IMU (Acc,Gyro) data out over Wi-Fi following the SensiML Simple Stremaing interface specification. This can be used in conjuction with the SensiML Data Capture Lab or the SensiML Gateway to view/record the data stream.

1. set the RECONGITION flag to 0
2. Flash the firmware
3. Connect to the firmware using the Data Capture Lab or Open Gateway Data Caputre Mode

You will need the m5Stick SDK in order to run this https://github.com/m5stack/M5StickC. Instructions for getting setup with the m5Stick SDK on the Arduino IDE are here https://docs.m5stack.com/en/arduino/arduino_development

You can connect directly with the open gateway. Use the m5_stick.ssf if you want to capture data via the Data Capture Lab.

# Recognition

This is an Arudino Sketch for the M5StickCPlus that enables running ML Models via SensiML KnowledgePacks against IMU (Acc,Gyro) data.

1. Import the KnowledgePack.zip file into Arduino
2. Set the RECOGNITION flag to 1
3. Connect to the firmware using the Open Gateway Recognition Mode
