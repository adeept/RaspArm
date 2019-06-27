# RaspArm
A robotic arm based on Raspberry Pi.

Setting up in a Raspberry Pi may take you a lot of time, and there are too many libraries needed, so we write a python program to do the most of works for you.

NOTE: This setup.py program is needed to download and install a lot of applications and libraries, and sometimes the server or internet may break down, which may lead to some problems that the setup.py could not fix. Then you must set up a raspberry pi yourself, by following the instructions of next chapter named Set Up a Raspberry Pi.

Download the program of the PiCar-B.
Input the code below to download: (Note that the commands entered here must be in lowercase)
sudo git clone https://github.com/adeept/rasparm.git
Then setup：
sudo python3 rasparm/setup.py
It may take some time to finish.

Note: The moment the power is turned on, the arm will swing randomly at a small range. Please  keep your eyes away from the swing range of the arm as much as possible. After the Raspberry Pi is turned on, the servo on the arm will drive the corresponding joint to swing to the initial position successively, while the LCD screen will display the port number of the servo to be moved and the remaining time of the movement. This is a security feature that is turned on by default, and you can turn it off by changing the code.

The gamepad of RaspArm has four input modes: Rotation, Short-press, Long-press I and Long-press II.
Short-press: Press the button and release it quickly. At this time, the LED will turn red when pressed. When it is released, it will change back to the previous color.

Long-press I: Press and hold the button for a while and then release. Specifically, the moment the button is pressed, the button turns red. After a short period of time, the LED turns blue and dims gradually. Release the button when the blue becomes completely dark out.

Long-press II: Press and hold the button until the LED turns red - turns blue - blue dims gradually-  becomes bright light blue and then release.

After the normal boot, the default operation interface is the mode selection interface. Under the offline state, RaspArm has 4 modes to choose by default, which are:
<Rotary Encoder>
In this mode, you can control the servo with the rotary encoder on the gamepad. Rotate to adjust the PWM of the corresponding servo port; short-press to switch the servo port; long-press II to exit the mode and enter the menu interface of the selection mode.

<Movement Input>
In this mode, you can control the RaspArm with the attitude sensor on the gamepad. The rotary knob can control the PWM of port3 (the servo of the clip by default); the lateral tilt of the gamepad can control the PWM of port0; the pitch tilt can control the PWM of port1 and port2; you can choose to control the specific one by short-pressing the button; long-press II to exit the mode and enter the menu interface of the selection mode.

<Keys and Setps>
This mode allows RaspArm to automatically loop the programmed steps. Short-press to select the servo needs to be controlled; Long-press I to save the current position; In theory, you can save more than 20 million positions. Long-press II to start looping the programmed steps. After starting, long-press II to exit the loop and enter the menu interface of the selection mode.

<TimeLapse Mode>
The name of this mode comes from a kind of photography technology. Compared with the <Keys and Setps> mode, this mode can customize the time required for the movement between any two positions and the number of motion decomposition in addition to the position information. After locating, long-press I to input the time required to move to the next position, then long-press I again to enter the number of motion decomposition to move to the next position, and then input the next position, theoretically it can save more than 10 million positions. After the input is completed, long-press II to exit the loop and enter the menu interface of the selection mode.

When RaspArm is turned on, it will automatically search for known Wi-Fi. If there is one , it will automatically connect to it. You can remotely control RaspArm through the GUI program on the PC(You can download it from https://github.com/adeept/rasparm/). If RaspArm is not connected to Wi-Fi after booting, it will automatically create a Wi-Fi hotspot which you can connect through the computer. The default address of the server is 192.168.12.1, and the rest of the operation remains unchanged.

