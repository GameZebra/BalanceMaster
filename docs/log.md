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
    - there is improvement but even thou somethimes there are spikes and, i'm thinkig 
    - I may try adding the average value as current value, this way I'll have even better filtration
- :page_facing_up: I still had large spikes in the Kd component
- 🔍 found a YouTube channel that has decent math and controll videos
    - watched the tutorials for Complex Analisis
- 🔍 found a MatLab example of an inverted pengulum controll which i can try
- 💻 tryed swapping the accelerometer with the gyro
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


## 📅 May 27, 2025
- 🔧 fixed the controler ant the accelerometer to the body of the robot better to limit the vibrations
    - and it is working verry well
- 💻 impemented moving average filtration
    - A great stupidity actually, I'had suspicions about that but
    - this way the angle tends to zero, and is not usefull at all
- 🔍 maged to connect the osciloscope with the PC (it needs it's USB cable to work properly)
- 🔍 The command "Get Motor M0/M1 Speed" returns the speed the driver is trying to speed the motors to - So it's not practical for checking 
if the motrs are not moveing
- 🔍 The command "Get Motor M0/M1 Current" returns average motor current for the past 5ms (+/-20%)
- 🔧 I measured the current throught the motors and the driver in different working conditions (including work mode)
    - TODO: analize the results why does the controller shuts down?
- 💻 improved the encoders demo code:
    - works with both encoders on the propper channels 
    - calculates the wheels actual speed
    

## 📅 May 28, 2025
- 💻 added code for the second encoders
    - it's interesting how when the encoder variable overflows, the speed variable overflows as well and i don't have massive speed reading
    - min speed added
- 💻 implemented brake before reversing
    - almost workiing


## 📅 May 29, 2025
- 💻 implemented brake before reversing
    - fixed the brake before reversing now stops whell
- 🧠 fix the zero position with bias for the angle
    - implementation (configuration required)

## 📅 May 30, 2025
- 💻 fix the zero position with bias for the angle
    - implementation (configuration required)
- 💥 managed to fry the buck converter that was supplying energy to the motor driver
- 🔧 measured the weight of the robot (to use in matematical model)

## 📅 June 02, 2025
- 🔍 Watched state space tutorials 
- 🔧 got new Buck converter (the same as the last one)


## 📅 June 03, 2025
- 🔍 played with the examples from the tutorials
- 💻 tried to implement angle correction on startup
    - i oscilate it by hand and it finds where the delta is smallest
     unfortunately (i suppose due to the noise of the accelerometer) this way the resulted setpoint is far from the actual zero
- 💻 changed the Kd to be basesd on the previous angle and not on the error to presumably limit the efect on the error based on the adaptive function that changes the angle
- 💻 remove the Ki because of double integration!!!
- 🔍 watched more from the control bootcamp
- 🧠 I talked with my mentor and it is highly advisable to have recordings of the performance of the robot in different strategies
    - with or without Ki
    - for this i have TODO a matlab file that can record the internal states of my robot

## 📅 June 04, 2025
- 🔍 watched more from the control bootcamp
- 💻 created a function that prints the important data to the PC via UART
    - the problem is that when i try to print everything the controll is visibly slower
    - Probbably i will have to use DMA for this
    - or just up the clock speed of the MCU because right now it is at 8 MHz
- 🔍 an interesting fact is the data types with wich the data from the experiment .txt .dat .bin
    - to save the data in .txt will require to convert the data to string first which is impractical for real time aplications
    - so i'll save the data as .bin and convert it in matlab later
    - .dat is some form of hybrid between the two where the data can be different file formats
- 💻 definately the responsiveness improved with increasing the clock speed
- 🔍 a little research about DMA
- 💻 also created matlab file that gets .bin file (from the experiments) and gets and plots the data
    - I should have done this sooner

## 📅 June 05, 2025
- 🔍 how to create header files in c, how are they connected, what to declare where and how to use them
    - fun fact when creating global variables in .h files they won't be created unless used insed the main file (the compiler decedes to optimize them)
- 💻 created separet .h files for different functionalities
    - for controlling the motors
- 🧠 I talked with my mentor: if i have some unbelievable value like 3.2*10^38 degrees (for the robot angle) i can just use the previous value
    - I made it if it is larger than 80 degrees to use the previous value
- 🧠 If the biggest problem for my control is the derivative because the accelerometer becomes very spike from the motr noise
I can just use the gyroscope for it
- 🧠 And the algorythm for the space correction will stabilize small imperfections of my zero position



## TODO
- digital filtration for the derivative
- use the gyro for the derivative
- the angle correction i think should be based on the movement of the robot and not exactly on the way it wants to move

