# ttn_tracker
Tracking Device for LoraWAN

# wiring
this tracker uses SoftwareSerial to communicate to UART GPS devices. Change conhfig.h to your needs

# Usage
Edit devices.h to match your tracking devices

# Change SpreadingFactor
Sending downlink messages with Byte one set to 0x10 and Byte two set to the desired spreading factor will change the devices spreading factor.