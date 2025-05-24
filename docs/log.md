## 📅 May 11, 2025
- 🔍 Research on documentation strategies
- :page_facing_up: log file created
- :page_facing_up: cheetsheet

## 📅 May 12, 2025
- 🔧 Soldered BMS and balancing wires to the battery pack
- 🔧 tested battery capacity (they cant reach 4.20, max 4.15)
- 🎨 made 3d model bracket for the power button

## 📅 May 13, 2025
- 📦 printed the power button bracket
- 🔧 Soldered and fitted the button on the robot
![robot fitted with battery pack and power button](/media/fitted_battery_pack_and_power_button.jpeg "robot fitted with battery pack and power button")  
- 🔧 Made a power rail with buck convertors to stabilize the power
![power rail](/media/power_rail.jpeg "the upper side of the power rail")  

## 📅 May 14, 2025
- :page_facing_up: fixed prior log data
- 🔍 retested gyro
- 🔧 created gyro board and fited it to the robot
![gyroscope board](/media/gyroscope_board.jpeg "gyroscope board") 
- :page_facing_up: to the user data file created on the tablet

## 📅 May 15, 2025
- :page_facing_up: created pinout for the project + future improvements
- 🔍 retested gyro on 3V (because the stm32f407 board can read with the ADC up to 3V)
- 🔧 fixed the gyro board to output 3V
- 🔧 created MCU board and fited it to the robot
    - used multitool for the first time
![MCU board](/media/MCU_board.jpeg "MCU board") 
- :page_facing_up: created doc file for the board shield on the tablet
- 🤖 robot hardware done
![robot upgrade done](/media/robot_upgrade_done.jpeg "robot upgrade done") 

## 📅 May 20, 2025
- 💻 fixed the Franken bot file, now:
    - reads the gyroscope
    - reads the accelerometer
    - controls the motors
- 💻 added PID algorythim
- 🤖 baddly configured PID, but the robot is trying to stabilize itself


## 📅 May 22, 2025
- 💻 spend a lot of time trying different PID values - didn't work well
    - the robot is trying to stabilize itself, it's much better than the first iteration, 
    BUT it still can't stabilize the robot, nor to compensate for a high diviation from 
    the stable position
    - when the Kd is high the robot seems closer to stabilizeing itself, BUT it's a lot jurkey
    and this heats the motors and the buck converter that stabilizes the volteg for the motor driver
    (to the point taht the buck converter shuts off). Probbably the frequent change in direction 
    requieres high amount of current without almost any meaningfull result
    - tried using a deadzone around the work point, but this worked badly for the responsiveness
    and worked good for the overheating of the motors 
- :page_facing_up: i used already derived differential equations for the mechanics of an inverted pengulum
and derived a State space model from them (by hand)
- 🧠 I am thinking that a much hidher Kd will be required but this makes the motors to overheat
and that's far from ideal. Probbably i should try:
    - filtration of the accelerometer signal 
    - use the gyro - even thou it's signal is far from ideal (correct) it may suffer less from the noise
    generated from the motors (whem jurking around the stable position)
- :page_facing_up: TODO try:
    - high Kd component and larger deadzone
        - i want to get the robot atleast to ascilate around the work point
    - filtration of the signal
    - measure the parameters of the robot and try to create a model in matlab
    - tune a PID in matlab and try it on the robot to see if the model created has anything in common with the actual robot


## 📅 May 23, 2025
- 💻 impemented moving average
    - there is improvement but even thou somethimes there are spikes and, 
    i'm thinkig I may try adding the average value as current value, this way I'll have even better filtration
- :page_facing_up: I still had large spikes in the Kd component
- 🔍 found a YouTube channel that has decent math and controll videos
    - watched the tutorials for Complex Analisis
- 🔍 found a MatLab example of an inverted pengulum controll which i can try
 💻 tryed swapping the accelerometer with the gyro
    - didn't go well, i activated a timer to periodically return the gyro in the "correct" angle valye, but how do i know it is "correct"?
    it behaved as a gyro with wrong zero and then when it synced - just jumped on the side, and this was happening every second 
    - if i want to use the gyro I must prepare the data better first
- 🧠 I am thinking that:
    - apart from the filtration i may need to secure better the controller to the body of the robot
    because this way I'm amplifying the vibrations  
    - I may also look at the filtrations that we used in the labs for System Identification
 - 🧠 How to implement sequential control? 
    - upright stabilization is acheived by following the zero degrees angle
    - and to acheive movement i can dynamicaly change the target angle so that the car moves linearly in space
