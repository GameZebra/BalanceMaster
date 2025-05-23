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
- :page_facing_up: i used already derived differential equations for the mechanics of an inverted pengulum
and derived a State space model from them (by hand)
- 🧠 I am thinking that a much hidher Kd will be required but this makes the motors to overheat
and that's far from ideal. Probbably i should try:
    - filtration of the accelerometer signal 
    - use the gyro - even thou it's signal is far from ideal (correct) it may suffer less from the noise
    generated from the motors (whem jurking around the stable position)

