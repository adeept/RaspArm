# RaspArm
A robotic arm based on Raspberry Pi.

Setting up in a Raspberry Pi may take you a lot of time, and there are too many libraries needed, so we write a python program to do the most of works for you.

NOTE: This setup.py program is needed to download and install a lot of applications and libraries, and sometimes the server or internet may break down, which may lead to some problems that the setup.py could not fix. Then you must set up a raspberry pi yourself, by following the instructions of next chapter named Set Up a Raspberry Pi.

Download the program of the RaspArm.
Input the code below to download: (Note that the commands entered here must be in lowercase)
sudo git clone https://github.com/adeept/rasparm.git
Then setup：
sudo python3 rasparm/setup.py
It may take some time to finish.

Note: The moment the power is turned on, the arm will swing randomly at a small range. Please  keep your eyes away from the swing range of the arm as much as possible. After the Raspberry Pi is turned on, the servo on the arm will drive the corresponding joint to swing to the initial position successively, while the LCD screen will display the port number of the servo to be moved and the remaining time of the movement. This is a security feature that is turned on by default, and you can turn it off by changing the code.
