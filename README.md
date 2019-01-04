# PID controllers
## Control a vehicle using a PID controller to adjust the steering.

---

[//]: # (Image References)

[image1]: ./support/SimulatorStartup.png "Simulator Startup Window"
[image2]: ./support/PID_simulator.png " PID Simulator"
[image3]: ./support/PID_running.png " PID Running"
[image4]: ./support/PID.gif " PID Video Clip"


## Overview
Implementation of a PID controller in C++, which manoeuvres a vehicle around the lake race track in the Udacity simulator. Using a PID controller the position of the vehicle is maintained using an error distance from an ideal road line. This error distance that is fed to the PID control is known as cross track error (CTE). The CTE is provided over WebSocket from the Udacity Term 2 Simulator. The code receives the error data and passes it into the PID control algorithm; calculates the error, then returns a steering correction factor. To support the PID steering control a simple speed control is implemented to help arbitrate the control disturbances when cornering. The results are displayed in the simulator and the car can be seen successfully navigating the track.

---

## Installation steps

To run this code the following downloads are required:

1. Make a project directory `mkdir project_udacity && cd project_udacity`
2. Clone this repository into the project_udacity directory. `git clone https://github.com/nutmas/CarND-PID-Control-Project.git`
3. Setup environment. `cd CarND-PID-Control-Project\` and launch the script to install uWebSocketIO `./install-mac.sh`. Alternatively for Ubuntu installs launch `./install-ubuntu.sh`. The environment will be installed with these scripts.
4. Download Term 2 Simulator: 
      * Download from here: [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases).
      * Place the extracted folder into the project_EKF directory. 
      * Within the folder run simulator `term2_sim`; if successful the following window should appear:
      ![alt text][image1]

---

## Other Important Dependencies

* cmake >= 3.5
* make >= 4.1 
* gcc/g++ >= 5.4

---

## Build the code

1. From the project_udacity folder change to folder created from cloning `cd CarND-PID-Control-Project`
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 

---

## Usage

After running the script, confirming the operation of the simulator and building the code the PID program is ready to be run.

1. From terminal window; change to build folder of project `cd ~/project_udacity/CarND-PID-Control-Project/build/`
2. Run PID: `./pid `
3. Message appears `Listening to port 4567` the PID is now running and waiting for connection from the simulator
4. Run the simulator `term2_sim`
5. In the simulator window, press 'Next' arrow three times to navigate to PID Controller then press 'Select'.

![alt text][image2]

6. In the PID terminal window `Connected!!!` will now appear.
7. The PID is now fully operational, the vehicle will proceed to navigate around the track, controlling its own speed and steering autonomously.

The Animation will start and the vehicle can be seen moving around the screen.

![alt text][image3]

---

## Reflection

A PID Controller is 3 separate controllers that can be in used together or in different combinations such as P, PI, PD or PID. The three types of control are **__P__**roportional **__I__**ntegral and **__D__**erivative. The tuning of these three can result in a very accurate control system.

There are a number of ways to perform tuning, all of which aim to identify a constant value to be used with each controller: 

* Manual Tuning - Trial and error
* Mathematical Calculations - Ziegler Nichols Method
* Software Algorithm while online - Twiddle

I tried these methods and found that no one method is wholly sufficient to identify the constant value in this experiment. The software method required initial stability in order to start 'tuning' each constant; this meant that the initial values could not be initiated to zero as the vehicle would be off the track before any values to control stability could be found.

The process I followed was to manually set some values to get the vehicle to a minimum level of stability:

* I initially set gain parameters: proportional Kp, I set the integral Ki and derivative to 0.0.
* Starting with Kp on the straight road, I incremented it to a value where oscillation would start in the control - once the vehicle started to the turn the oscillations would increase rapidly an cause the vehicle to eventually oversteer and leave the road.
* With proportional control as it approaches the correction value with reduce proportionally to try to converge on the target value. However if the error values are too high it may overshoot and as it tries to correct the overshoots an oscillation is generated.
* In order to correct this overshoot I steadily increased the Kd constant value. The derivative controller will predict the future value of the error and if it is going to overshoot will correct the error to decrease the rate which it is approaching the target.
* The integral component is a running sum of errors; It supports the control by ensuring that the approach to the target is not too slow. With this parameter I implemented a very small value. Even with this small value if it was left to continuously integrate, once control started to converge the integral would be a large number and cause the control to overshoot and become unstable again. 
* To counter this I limited the size the integral could grow to and if it exceeded the set amount then I would reset its value back to 0.  `PID.cpp lines 64-67`
* The second limitation I placed on it was when the error reached a small enough value I disable the integral by setting it to 0 while the error was less than this value. `PID.cpp lines 59-62`
* This resulted in the Integral component effect being quite subtle, but it can be noticed when it comes into effect on the turns when the vehicle has slowed and is far from the target while making the turn, the integral component will quickly ramp up to get back to target.
* Overall I found the higher values of P would cause the control to become unstable and the vehicle to eventually oversteer, and if I tried to counter this with too high values of D, then the response would be sluggish and the vehicle would go wide on the corners. Any high Integral values would result in a build up and the vehicle go from stable to very unstable and not able to recover.

My final parameters chosen for the PID control constants are: 

`P =0.1, I=0.002  D=3.5` 

They are hard coded into `main.cpp` line 61: `pid.Init(0.1,0.002,3.5);`

I tried the Ziegler Nichols mathematical method which involved finding an inflection point in the response curve when at steady state, and then using a table of formulae to calculate the PID constants, however it proved difficult to get a reliable curve and took a long time to calculate.

I also tried implementing twiddle as a software solution. It never converged and produced a final result, but it did partially settle with the following values:

`Kp: 1.14981 Ki: 0.431999 Kd: 2.3973`

With the software solution I still had to have a fairly stable initial set of values to allow the simulation to run so that the software could `twiddle` at the constants.

The final conclusion was that tuning with the manual method with trial and error I could achieve a good enough control result.

In order to control the speed I implemented a very simple speed limiter which would request a slight braking  if the absolute PID calculated steering request was greater than 0.4. Otherwise allow the vehicle speed request to be 0.45. This worked quite well and can often be seen being implemented by the vehicle brake lights turning on.


---

## Results

The vehicle successfully navigates around the track. 
Criteria for success:

 `No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe.` 

 The following video shows a short duration of the vehicle control stability over the straight bridge, then steering and controlling its speed around the 'S' bends.

![alt text][image4]

A video of the full lamp can be viewed by downloading: [Full Lap Video](./support/FullVideo_480.mov)



---

## License

For License information please see the [LICENSE](./LICENSE) file for details

---

