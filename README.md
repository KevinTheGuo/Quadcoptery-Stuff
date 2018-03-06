# Quadcoptery-Stuff
A repository that holds the contents of my journey to my own quadcopter flight controller

This project is very much a work-in-progress, and currently on hold for the Fall 2017 semester (classwork is ridiculous right now!)

The goal of this project is to develop a functioning and autonomous quadcopter for aerial surveying purposes, and while doing that,
gain familiarity with flight control systems, automation, sensing, and telemetry. 

I tried to develop my own flight control system but that didn't work well, so I used an open-source one online. The current design of 
the drone relies on two microcontrollers, which which keeps the quadcopter in the air (running the open-source software), and another
I programmed which functions as a 'wrapper', allowing me to develop my own radio telemetry and control system, as well as being able
to further serve as an additional level of control (with stuff like GPS input, magnetometer+barometer control).

Still working on GPS autonomy. Because of the way the sensor sends interrupts to the microcontroller, (and my lack of hardware PWM pins)
my 'wrapper' microcontroller starts sending crazy signals to the flight-control microcontroller. I'm considering switching to some sort
of Raspberry Pi for this 'wrapper'.

I used to control the quadcopter with an arduino + radio transceiver plugged into my laptop, but typing on keys started to get hard, so
I developed a hand-motion controller which could take your hand's gyroscopic position (and your cardinal orientation) to more intuitively
control the quadcopter.
