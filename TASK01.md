The whole screen design will be using black background.
Lets start with the startup screen. Will show BOOTING UP when turning on for 3 seconds.
Once done, will start to initialize all the modules:
1. INA226
2. MCP23017 - Toggle switches
3. Gimbals
4. CRSF module

Will detect the throttle position. If throttle position > 15% then show an warning sign to the screen until the throttle is below the value then continue the start up sequence.
Next will detect all the switches position except Toggle6. Toggle6 is the special switch where we are using for toggle configuration mode or radiocontrol mode. During configuration mode, the pitch will be acting as up and down key. When the pitch is more than 1800 consider up key and if pitch is less than 1200 then is consider down key. The roll will be acting as OK and Cancel keys. If Roll more than 1800 then consider as OK and less than 1200 then consider as cancel. With this, we will be using the gimbal to do the menu selection.

The screen will show the voltage percentage. For 8.4V will show as 100% and 6.0V will be 0%. If toggle6 is on radiocontrol mode, it will show the flight time based on the preset toggle as ARM. All toggles are configurable to modes like arm, acro mode, return to home mode, turtle mode. Something like EdgeTX. 

We will need a few menus:
1. Showing the gimbals position and trim it based on the fine tuning.
2. Showing the details of all the MCP23017 mapping then we can start to assign functions to the specific toggle except toggle6.
3. Showing the details of the battery consumption.
4. Showing the details and configuring the CRSF module (BetaFPV Nano TX2 Module)

There will be total flight time, current flight time based on the arm toggle. The total flight time will be stored into the ESP32.
