# WSPR Beacon 
This project is related to a Tech Minds video on how to build a WSPR beacon using an Si5351 and an ESP32 module. The files wihtin this repo are for use with Visual Studio Code.
## Hardware
**Si5351**  
UK: https://amzn.to/3NM5uQk  
US: https://amzn.to/4qOQD5X  
  
**ESP32-S3-N16R8**    
UK: https://amzn.to/4sDV05G  
US: https://amzn.to/49By4v1  
  
**2W RF Amplifier**  
UK: https://amzn.to/4sHx9Ce  
US:https://amzn.to/4jKjeHt  
## Usage
Create a new PlatformIO project within Visual studio code, then replace main.cpp and platformio.ini with those in this repo. The board details wihtin the platformio.ini file are specific for the linked ESP32 module above.  

Once firmware has been loaded onto ESP32 use a wifi device to connect to "TechMinds-ESP32WSPR". This is open, no password needed. Then bavigate to: http://ESP32WSPR.local where you can change the wifi to connect to your home network, enter your callsign and assign a valid Maindenhead locator.
