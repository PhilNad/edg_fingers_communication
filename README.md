# EDG Fingers Communication ROS Package (Publisher/C++)

## Objective
The principal objective of this program is to make available to ROS the data coming from a serial connection. Although this ROS package was developed in the context of an Arduino Uno streaming data as fast as possible to a PC, it could be used with another interface with very little change.

## Why do I need this?
One of the most useful aspects of ROS has to be its messaging interaface allowing a great quantity of programs/nodes to talk to each other through the publication/listening of message on ROS topics. In order to allow ROS nodes to access the data generated from a real-world instrument, an interface has to be running to expose this real-world data to the ROS environment. This interface is the subject of this package.

## How does it work?
### The communication interface
On linux, many Human-Machine interfaces like your keyboard or your screen (back in the days when a screen was called a terminal) are exposed to the operating system as regular files by the hardware drivers. This makes it really easy for any program or any user to use such hardware. You would simply read the file associated with the keyboard to know what the user had entered, process it and then write the result to the file associated with the screen so the user could see it. Most of the time, these "files" dont actually takes any space on the hard disk and are virtual. This interface between the hardware and the software is so simple and works so well that it's still part of Linux today. On most versions of Linux, these special files are located in the `/dev/` directory. For example, the keyboard is represented by the file `/dev/stdin` while the screen is represented by `/dev/stdout` and a device that generates random numbers is represented by `/dev/random`. By opening a terminal, you can easily test it:
```bash
[phil@phil-local ~]$ echo "Test" >> /dev/stdout
Test
[phil@phil-local ~]$ cat /dev/random
���6�c2� �C�EJ/jp�"�O�����������q�{��
```
**Ok I understand, Linux is awesome, now what?**
The serial interface used to communicate between an Arduino and the computer is often exposed by the operating system as `/dev/ttyACM0`. The OS will generate a similar name (like `/dev/ttyACM1`) if another serial connection is already present. Some other device like the Robotiq 2f-140 gripper will use `/dev/ttyUSB0` or something similar. Reading that file let you read the communication channel while writing it let you transfer data through the channel. IMPORTANT NOTE: In order for a user to be able to read/write to these interfaces, it must be added to the `dialout` linux user group:
```bash
sudo usermod -a -G dialout $USER
```
In our current setup, only a single serial connection is made (to the Arduino) and therefore the file used is `/dev/ttyACM0`. When in doubt or when working with multiple devices, try unplugging and replugging the USB cable and then run this command to see which TTY interface was recently made available (lower is more recent):
```bash
dmesg | grep tty
```

### The communication protocol
Since the Arduino board has very limited resources, the communication protocol must be as small and simple as possible in order to allow for a measurements sampling frequency as high as possible. Indeed, serial communication is very expensive in terms of computing time and generates a great number of interrupts. While processing an interrupt, the system ignores every other interrupts (like an encoder tick interrupt) and can lead to measures inaccuracies. It is paramount to reduce as much as possible the quantity of data transmitted through the serial interface. In the mean while, the more data points you are able to transfer, the more accurate your data curves can be. A compromise must be found.

As the current setup employs 2 suction-enabled fingers, for every time step 4 measurements must be transmitted (pressure and position for each finger). These values are represented by an integer which is coded on 2 bytes on the Arduino platform. The current protocol transmit a data point using this packet definition:

| Finger 1 Position | Comma | Finger 2 Position | Comma |  Finger 1 Pressure | Comma |  Finger 2 Pressure | Newline |
|:-----------------:|:-----:|:-----------------:|:-----:|:------------------:|:-----:|:------------------:|:--------|
|2 bytes            |1 byte |2 bytes            |1 byte |2 bytes             |1 byte |2 bytes             |1 byte   | 

The role of the newline at the end is to help the computer know when a packet is fully transferred. We must keep in mind that sometimes communication errors will happen and we cannot rely strictly on counting the number of bits transferred to know when a packet is fully transferred.  

The role of the commas is to verify if the data transmission was done without errors. To accept a packet as legitimate, the third, sixth and ninth bytes must be commas.

**Potential enhancement**: It should be possible to send positions as the number of encoder ticks detected since the previous packet sent, allowing use to use only 1 byte per position field (or even less), it would be a kind of differential position.

Since an integer is stored into two bytes on the Arduino, it gets (automatically) desassembled for the serial transfer and must be reassembled on the computer. To do so, the most significant byte (MSB) and the least significant byte (LSB) are combined together such as the MSB occupies bits 8-16 and the LSB occupies bits 0-7.

### The publication on ROS topics
As this program is a ROS node (with the name edg_fingers_com_node), it can create, publish and listen on ROS topics. One topic is created for each of these measures and for each of the fingers:
1) Position, as an unsigned 16 bits integer 
2) Velocity, as a signed 32 bits float
3) Pressure, as an unsigned 16 bits integer

Note that the velocity will be calculated by doing a numerical derivative of the position with respect to the time difference. The computer's chronometer being very precise and the Arduino being very consistent in terms of timing, the velocity should be relatively accurate although a bias will probably be present.

The topics used by this programs are therefore:
* "fingers/1/position"
* "fingers/1/velocity"
* "fingers/1/pressure"
* "fingers/2/position"
* "fingers/2/velocity"
* "fingers/2/pressure"

Note that this naming convention allows for the easy addition of a third finger.

The messages being published by this program are instances of messages found in the std_msgs ROS package (https://wiki.ros.org/std_msgs) which defines basics messages types that contains nothing but a "data" field.
