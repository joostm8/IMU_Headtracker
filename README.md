# IMU_Headtracker
## Preface
I wanted to use a headtracker to use in racing games such as Dirt Rally or RaceRoom Racing Experience. TrackIR is the best known tracker, but I couldn't really justify the cost. Having seen many other IMU-based headtracker projects, I figured this was a neat little project to work on in my spare time, since most of the heavy lifting work has already been done by other people:
- The IMU sensorfusion algorithm to translate accelerometer, gyroscope and magnetometer data into yaw, pitch and roll was made by Sebastian Madgwick and is available as C code:http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/ .
- Opentrack is a project that can translate any joystick/gamepad information into freetrack data, used by games to do headtracking. Opentrack also provides centering support, and allows you to invert axis on-the-fly.

I really wanted no extra wiring, so I preferably wanted a Bluetooth dongle or something alike, such that I could treat the headtracker as a standalone little box.

## Hardware components
What was left to do for me was to use a microcontroller, an MPU and a Bluetooth dongle that runs a game controller HID profile, in order to relay the yaw, pitch and roll data to Opentrack in a standardised manner. I chose the following components:
- IMU: an Invensense MPU9250, this is the cheapest 9 DOF IMU I could readily find on ebay, it's usage is also thoroughly documented by Kris Winer on https://github.com/kriswiner/MPU9250 .
- Microcontroller: an Atmega328p running at 3.3V and 8MHz, more specifically as a Pro-Mini Arduino clone. It is a cheap microcontroller, readily available, comes with an Arduino bootloader so I don't need an AVR programmer and as documented in https://github.com/kriswiner/MPU9250 is just about able to run the MadgwickAHRS algorithm at an okay pace.
- Bluetooth module: an RN42 is the most easily available bluetooth module with HID gamepad support, but one can also flash the RN42 firmware on the even more broadly available HC-05 modules. Refer to this link for more information: https://mitxela.com/projects/bluetooth_hid_gamepad
- Power: a small 600 mAh LiPo battery and TC4056A charger board complete the list of required hardware.
- Two LEDs to give some feedback
- An ON-OFF switch (ON-ON also works)
- Two 680 ohm resistors to limit current to the LEDs

## Software components
I needed I2C communication to communicate with the MPU9250 (SPI is also possible), UART communication to communicate with the HC-05 and I needed to alter the MadgwickAHRS algorithm slightly.
- I2C communication is handled by an I2C library provided by Microchip: https://www.microchip.com/wwwAppNotes/AppNotes.aspx?appnote=en591794 (search for AN2480 in case the link doesn't work).
- The MadgwickAHRS algorithm normally expects to be run at a certain frequency, however the Atmega328p can't reliably run it at very high frequencies. Instead, I opted to alter the library, as demonstrated in this code from Michael Baker https://github.com/mikeshub/Pololu_Open_IMU, so it now calculates the time delta (dt) between the current start and the previous start of the code section of the AHRS algorithm, so it can basically run at the highest speed possible.
- UART to communicate with the HC-05: At 8 MHz clockspeed, 38400 baud is the highest baudrate that lies within the recommended clock error range, as stated in the Atmega328p manual. Since I need almost all cpu cycles to run the AHRS algorithm, I wrote a small UART transmit function that can handle one transmission in an unblocking fashion, such that I can transmit the yaw, roll and pitch to the Bluetooth dongle, while the AHRS algorithm continues executing.

## Wiring it all up:
A small table showing how I wired the components. Note that I relied on the LDO of the Pro-Mini to regulate the voltage for all parts. According to some datasheet on an Aliexpress like site, the part should provide at least 100 mA, whilst all components together will use approximately 60 mA, so this should be fine.

Note that I inserted an ON-OFF switch on the OUT+ line of the TC4056A, but this is not shown in the table.

**Note that the resistors aren't listed in the table, but PIN7 must be connected to the LED1 Anode through a 680 ohm resistor (likewise for PIN6 and LED2 Anode)!!**

| Pro-Mini   | HC-05   | MPU9250 | TC4056A | LED1    | LED2    |
| ---------- | ------- | ------- | ------- | ------- | ------- |
| SDA        |         | SDA     |         |         |         |
| SCL        |         | SCL     |         |         |         |
| TX         | RX      |         |         |         |         |
| RX         | TX      |         |         |         |         |
| PIN 7      |         |         |         | Anode   |         |
| PIN 6      |         |         |         |         | Anode   |
| RAW        |         |         | OUT +   |         |         |
| VCC (3.3V) | 3.3V in | 3.3V in |         |         |         |
| GND        | GND     | GND     | OUT -   | Cathode | Cathode |

## 3D printable box
I've provided some STL files for a small 3D printable box that can mount the different components. The HC-05 I used was mounted on the standard blue breakout PCB, the MPU9250 is one of those modules mounted on the purple PCB. You'll have to stack the PCBs using pin headers, with a double layer of black spacers to achieve the correct spacing. On the HC-05, the TX, RX and GND pins match up, and on the MPU-9250 the SDA and SCL pins match up (granted you mount it **upside down**). Other pins can be connected with some servo wire. Note to self: provide a picture, that will be clearer.

## Some more remarks
### Application specific tips
- The Invense MPU9250 has built-in accelerometer calibration registers, however, even with their provided code (MPU Hardware Offset Registers Application Note) I could not get this to work correctly, therefore I did the accelerometer offsets in software. Gyroscope offset registers works fine.
- The more readily available RN42 manual has an incorrect specification of the joystick gamepad report format. Instead just use the format as described in https://mitxela.com/projects/bluetooth_hid_gamepad.
- The MadgwickAHRS algorithm does not care about the unit of the magnetometer and accelerometer readings, so don't waste computing time converting those readings to the correct unit. The gyroscope inputs do expect an input in radians per second.
- One might notice how there is some sign inversion happening on the input of the AHRS algorithm, that's because I mounted the MPU9250 upside down in the box.
- The LEDs help when starting the headtracker. When the LED on PIN7 goes on, you must place the headtracker on a flat surface, because it calculates the acceleromter and gyroscope offsets. When the LED on PIN6 goes on, you must wave the headtracker in a 8 shaped motion to calibrate the magnetometer (like you would do with your cellphone).

### General debugging tips
If you want to attempt an IMU-headtracker project yourself, you might bump into some issues. In that case a debugger is your best friend, or if you don't have a debugger, a UART serial connection with some debug info is your best friend, however, do ensure you format your serial printouts correctly!
- an *int32_t* is defined as "signed long int" in stdint.h, so to print a correctly formatted string, use *%ld*
- float (*%f*) formatting is not supported by the AVR-libc *sprintf* function, therefore you must first use *dtostrf* , then use *strcat* to obtain a printout of a floating point number.
