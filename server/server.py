#!/usr/bin/env python3
# File name   : servo.py
# Description : Control Motor
# Product	 : RaspTank  
# Website	 : www.adeept.com
# E-mail	  : support@adeept.com
# Author	  : William
# Date	: 2018/12/27
from RPi import GPIO
import time
import Adafruit_PCA9685
from rpi_ws281x import *
import argparse
import i2c_lcd1602
import threading
import datetime
import socket

#MPU_6050
try:
	from mpu6050 import mpu6050
	import Kalman_filter
	import PID
	P = 5
	I = 0.01
	D = 0
	X_pid = PID.PID()
	X_pid.SetKp(P)
	X_pid.SetKd(I)
	X_pid.SetKi(D)
	Y_pid = PID.PID()
	Y_pid.SetKp(P)
	Y_pid.SetKd(I)
	Y_pid.SetKi(D)
	kalman_filter_X =  Kalman_filter.Kalman_filter(0.001,0.1)
	kalman_filter_Y =  Kalman_filter.Kalman_filter(0.001,0.1)
	sensor = mpu6050(0x68)
	mpu = 1
except:
	mpu = 0
	print("use 'sudo pip3 install adafruit-pca9685' to install mpu6050, or check the wire connection.")
	pass

#LCD Screen
#try:
screen = i2c_lcd1602.Screen(bus=1, addr=0x27, cols=16, rows=2)
screen.enable_backlight()
lcd_show = 1
#except:
	#lcd_show = 0
	#pass

#Servos
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

#Set GPIOs for button.
clk = 35
dt = 36
btn = 38

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(btn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#Interfaces
port_0_speed = -3
port_1_speed = 3
port_2_speed = 3
port_3_speed = 3

speed_base = 0.01

pwm0_min = 100
pwm0_max = 500
pwm1_min = 100
pwm1_max = 500
pwm2_min = 100
pwm2_max = 500
pwm3_min = 100
pwm3_max = 500

#Global Variables.
clkLastState = GPIO.input(clk)
butSwitch	= 0
butMax	   = 4
Tolerance	= 1.5
longPress_I  = 12
longPress_II = 77
mpu_speed	= 0.1
remote_speed = 3

pwm0 = 300
pwm1 = 300
pwm2 = 300
pwm3 = 300

old_pwm0 = pwm0
old_pwm1 = pwm1
old_pwm2 = pwm2
old_pwm3 = pwm3

loop_cunt	= 0
loop_max	 = 300
loop_mpu_max = 350
ModeSwitch   = 1
LED_set	  = 1

lcd_last_line1 = ''
lcd_last_line2 = ''
lcd_new_line1  = ''
lcd_new_line2  = ''


class LED:
	'''
	Example Code for Using LED.
	led=LED()
	led.set_color(Color(77, 0, 0), 1)
	time.sleep(1)
	led.colorWipe(Color(77, 0, 0))
	'''
	def __init__(self):
		self.LED_COUNT	   	= 16	  # Number of LED pixels.
		self.LED_PIN		= 12	  # GPIO pin connected to the pixels (18 uses PWM!).
		self.LED_FREQ_HZ	= 800000  # LED signal frequency in hertz (usually 800khz)
		self.LED_DMA		= 10	  # DMA channel to use for generating signal (try 10)
		self.LED_BRIGHTNESS = 255	 # Set to 0 for darkest and 255 for brightest
		self.LED_INVERT		= False   # True to invert the signal (when using NPN transistor level shift)
		self.LED_CHANNEL	= 0	   # set to '1' for GPIOs 13, 19, 41, 45 or 53
		parser = argparse.ArgumentParser()
		parser.add_argument('-c', '--clear', action='store_true', help='clear the display on exit')
		args = parser.parse_args()

		# Create NeoPixel object with appropriate configuration.
		self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)
		# Intialize the library (must be called once before other functions).
		self.strip.begin()


	def colorWipe(self, color):
		"""Wipe color across display a pixel at a time."""
		for i in range(self.strip.numPixels()):
			self.strip.setPixelColor(i, color)
			self.strip.show()
			#time.sleep(wait_ms/1000.0)


	def set_color(self, color, num):
		self.strip.setPixelColor(num, color)
		self.strip.show()


def rotary(speed):
	global clkLastState, clkState
	clkState = GPIO.input(clk)
	dtState = GPIO.input(dt)
	if clkState != clkLastState:
		if dtState != clkState:
			clkLastState = clkState
			return -1*speed
		else:
			clkLastState = clkState
			return 1*speed
	else:
		clkLastState = clkState
		return 0
	clkLastState = clkState


def switch(mode):
	global lcd_new_line1, lcd_new_line2, ModeSwitch
	timestamp = datetime.datetime.now()
	lastChange = timestamp
	selet_num = mode
	ModeSwitch = 6
	while 1:
		clkState = GPIO.input(clk)
		dtState = GPIO.input(dt)
		if clkState != clkLastState:
			timestamp = datetime.datetime.now()
			if (timestamp - lastChange).seconds >= 0.1:
				lastChange = timestamp
				if dtState != clkState:
					selet_num -= 1
				else:
					selet_num += 1
		if selet_num > 5:
			selet_num = 1
		elif selet_num < 1:
			selet_num = 5
		if selet_num ==  1:
			lcd_new_line1 = 'Set: <1>-2-3-4-5'
			lcd_new_line2 = '<Rotary Encoder>'
		elif selet_num == 2:
			lcd_new_line1 = 'Set: 1-<2>-3-4-5'
			lcd_new_line2 = '<Movement Input>'
		elif selet_num == 3:
			lcd_new_line1 = 'Set: 1-2-<3>-4-5'
			lcd_new_line2 = '<Keys and Setps>'
		elif selet_num == 4:
			lcd_new_line1 = 'Set: 1-2-3-<4>-5'
			lcd_new_line2 = '<TimeLapse Mode>'
		elif selet_num == 5:
			lcd_new_line1 = 'Set: 1-2-3-4-<5>'
			lcd_new_line2 = '---<Settings>---'

		buttonPress = button(mode)
		if buttonPress == 0:
			ModeSwitch = selet_num
			break
		elif buttonPress == 1 or buttonPress == 2:
			ModeSwitch = mode
			break
		time.sleep(0.001)
		pass


def button(mode_input):
	'''
	Short Press return 0
	Long Press I return 1
	Long Press II return 2
	'''
	global butSwitch, LED_set, command_input_stu, command_output_stu
	butLast = 0
	shortPress_stu   = 0
	longPress_I_stu  = 0
	longPress_II_stu = 0
	while not GPIO.input(btn):
		LED_thread = 0
		led.set_color(Color(77, 0, 0), 4)
		led.set_color(Color(77, 0, 0), 5)
		butLast += 1
		shortPress_stu = 1
		if butLast > longPress_I:
			command_output_stu = 0
			shortPress_stu  = 0
			longPress_I_stu = 1
			butLast = 0
			led_change = longPress_II
			while not GPIO.input(btn):
				led.colorWipe(Color(0, 0, led_change))
				led_change -= 1
				if led_change == 0:
					longPress_I_stu  = 0
					longPress_II_stu = 1
					led.colorWipe(Color(0, longPress_II, longPress_II))
					while not GPIO.input(btn):
						pass
		time.sleep(0.05)

	if shortPress_stu:
		led.colorWipe(Color(0, 0, 0))
		LED_set = 1
		return 0
	elif longPress_I_stu:
		led.colorWipe(Color(0, 0, 0))
		LED_set = 1
		return 1
	elif longPress_II_stu:
		led.colorWipe(Color(0, 0, 0))
		LED_set = 1
		if command_input_stu:
			pass
		else:
			switch(mode_input)
		return 2


def ctrl_range(raw, max_genout, min_genout):
	global lcd_new_line2, LED_set
	if raw > max_genout:
		raw_output = max_genout
		lcd_new_line2 = 'REACHING MAX PWM'
	elif raw < min_genout:
		raw_output = min_genout
		lcd_new_line2 = 'REACHING MIN PWM'
	else:
		raw_output = raw
	return int(raw_output)


def dove_base(port, start, end, deley, pix, goal):
	global old_pwm0, old_pwm1, old_pwm2, old_pwm3
	if start < end:
		for i in range(start, (end+pix), pix):
			if goal == 0:
				old_pwm0 = i
				if end != pwm0:
					break
			elif goal == 1:
				old_pwm1 = i
				if end != pwm1:
					break
			elif goal == 2:
				old_pwm2 = i
				if end != pwm2:
					break
			elif goal == 3:
				old_pwm3 = i
				if end != pwm3:
					break
			print(i)
			pwm.set_pwm(port, 0, i)
			time.sleep(deley)
	elif end < start:
		for i in range(start, (end-pix), -pix):
			if goal == 0:
				old_pwm0 = i
				if end != pwm0:
					break
			elif goal == 1:
				old_pwm1 = i
				if end != pwm1:
					break
			elif goal == 2:
				old_pwm2 = i
				if end != pwm2:
					break
			elif goal == 3:
				old_pwm3 = i
				if end != pwm3:
					break
			print(i)
			pwm.set_pwm(port, 0, i)
			time.sleep(deley)


def base_move_thread():
	global old_pwm0, old_pwm1, old_pwm2, old_pwm3, TL_start
	while 1:
		if old_pwm0 != pwm0:
			pwm.set_pwm(0,0,pwm0)
			old_pwm0 = pwm0
		if old_pwm1 != pwm1:
			pwm.set_pwm(1,0,pwm1)
			old_pwm1 = pwm1
		if old_pwm2 != pwm2:
			pwm.set_pwm(2,0,pwm2)
			old_pwm2 = pwm2
		if old_pwm3 != pwm3:
			pwm.set_pwm(3,0,pwm3)
			old_pwm3 = pwm3
		if command_output_stu:
			for i in range(0,len(servo_step_0)):
				if command_output_stu:
					dove_move(servo_step_0[i],servo_step_1[i],servo_step_2[i],servo_step_3[i])
				print(i)
		if TL_start:
			old_pwm0 = TL_step_0[0]
			old_pwm1 = TL_step_1[0]
			old_pwm2 = TL_step_2[0]
			old_pwm3 = TL_step_3[0]
			for i in range(1,len(TL_step_0)):
				timelapse(TL_step_0[i],TL_step_1[i],TL_step_2[i],TL_step_3[i], timelapse_time[i-1], timelapse_frames[i-1])
				print(i)
			TL_start = 0
		time.sleep(0.01)


def rotary_mode():
	global butSwitch, lcd_new_line1, lcd_new_line2, pwm0, pwm1, pwm2, pwm3

	buttonPress = button(1)
	if buttonPress == 0:
		butSwitch += 1
		if butSwitch > 3:
			butSwitch = 0
	elif buttonPress == 2:
		return 1

	if butSwitch == 0:
		pwm0 += rotary(port_0_speed)
		pwm0 = ctrl_range(pwm0, pwm0_max, pwm0_min)
		lcd_new_line2  = 'Port:0 PWM:%d'%pwm0
	elif butSwitch == 1:
		pwm1 += rotary(port_1_speed)
		pwm1 = ctrl_range(pwm1, pwm1_max, pwm1_min)
		lcd_new_line2  = 'Port:1 PWM:%d'%pwm1
	elif butSwitch == 2:
		pwm2 += rotary(port_2_speed)
		lcd_new_line2  = 'Port:2 PWM:%d'%pwm2
		pwm2 = ctrl_range(pwm2, pwm2_max, pwm2_min)
	elif butSwitch == 3:
		pwm3 += rotary(port_3_speed)
		lcd_new_line2  = 'Port:3 PWM:%d'%pwm3
		pwm3 = ctrl_range(pwm3, pwm3_max, pwm3_min)

	time.sleep(0.001)


def movement_mode():
	global pwm0, pwm1, pwm2, pwm3, butSwitch, lcd_new_line2, pwm0_mpu, pwm1_mpu, pwm2_mpu, pwm3_mpu, old_pwm0, old_pwm1, old_pwm2, old_pwm3
	if mpu:
		accelerometer_data = sensor.get_accel_data()
		X = accelerometer_data['x']
		X = kalman_filter_X.kalman(X)
		Y = accelerometer_data['y']
		Y = kalman_filter_Y.kalman(Y)
		if abs(Y) > Tolerance:
			pwm0_mpu += Y*mpu_speed
			print(Y)
			print('mpu:%f'%pwm0_mpu)
			pwm0 = ctrl_range(pwm0_mpu, pwm0_max, pwm0_min)
			print(pwm0)

		buttonPress = button(2)
		if buttonPress == 0:
			if butSwitch == 1:
				butSwitch = 2
			else:
				butSwitch = 1
			print(buttonPress)
			print(butSwitch)
		elif buttonPress == 2:
			return 1

		pwm3_mpu = old_pwm3
		pwm3_mpu += rotary(port_3_speed)
		pwm3 = ctrl_range(pwm3_mpu, pwm3_max, pwm3_min)

		if old_pwm3 != pwm3:
			pwm.set_pwm(3, 0, pwm3)
			lcd_new_line2  = 'Port:3 PWM:%d'%pwm3
			return 0

		if abs(X) > Tolerance:
			if butSwitch == 1:
				pwm1_mpu += X*mpu_speed
				pwm1 = ctrl_range(pwm1_mpu, pwm1_max, pwm1_min)
				lcd_new_line2  = 'Port:1 PWM:%d'%pwm1
				return 0
			elif butSwitch == 2:
				pwm2_mpu += X*mpu_speed
				pwm2 = ctrl_range(pwm2_mpu, pwm2_max, pwm2_min)
				lcd_new_line2  = 'Port:2 PWM:%d'%pwm2
				return 0

		time.sleep(0.001)
	else:
		switch(2)
		pass


def lcdled_threading():
	global lcd_last_line1, lcd_last_line2, LED_set
	if lcd_show:
		screen.enable_backlight()
	timestamp = datetime.datetime.now()
	lastChange = timestamp
	old_lcd_new_line1 = ''

	while 1:
		if lcd_show:
			if lcd_last_line1 != lcd_new_line1 or lcd_last_line2 != lcd_new_line2:
				timestamp = datetime.datetime.now()
				if (timestamp - lastChange).seconds >= 0.01:
					screen.display_data(lcd_new_line1, lcd_new_line2)
					lastChange = timestamp
					lcd_last_line1 = lcd_new_line1
					lcd_last_line2 = lcd_new_line2
					pass
				continue
				pass

		if LED_set:
			led.colorWipe(Color(0, 0, 0))
			if butSwitch == 0:
				led.set_color(Color(0, 77, 13), 0)
			elif butSwitch == 1:
				led.set_color(Color(0, 77, 13), 1)
			elif butSwitch == 2:
				led.set_color(Color(0, 77, 13), 2)
			elif butSwitch == 3:
				led.set_color(Color(0, 77, 13), 3)

			if ModeSwitch == 1:
				led.set_color(Color(0, 12, 77), 4)
				led.set_color(Color(0, 12, 77), 5)
			elif ModeSwitch == 2:
				led.set_color(Color(12, 77, 0), 4)
				led.set_color(Color(12, 77, 0), 5)
			elif ModeSwitch == 3:
				led.set_color(Color(77, 24, 0), 4)
				led.set_color(Color(77, 24, 0), 5)
			elif ModeSwitch == 4:
				led.set_color(Color(12, 77, 0), 4)
				led.set_color(Color(77, 24, 0), 5)
			elif ModeSwitch == 5:
				led.set_color(Color(24, 77, 0), 4)
				led.set_color(Color(24, 77, 0), 5)
			elif ModeSwitch == 6:
				led.colorWipe(Color(77, 24, 0))

			LED_set = 0
			continue
		elif lcd_new_line2 == 'REACHING MAX PWM' or lcd_new_line2 == 'REACHING MIN PWM':
			led.set_color(Color(77, 7, 0), 4)
			led.set_color(Color(77, 7, 0), 5)
			LED_set = 1
			time.sleep(0.3)

		if 'Set:' in lcd_new_line1:
			if lcd_new_line1 != old_lcd_new_line1:
				led.colorWipe(Color(77, 24, 0))
				if lcd_new_line1 == 'Set: <1>-2-3-4-5':
					led.set_color(Color(0, 77, 13), 0)
					old_lcd_new_line1 = lcd_new_line1
				elif lcd_new_line1 == 'Set: 1-<2>-3-4-5':
					led.set_color(Color(0, 77, 13), 1)
					old_lcd_new_line1 = lcd_new_line1
				elif lcd_new_line1 == 'Set: 1-2-<3>-4-5':
					led.set_color(Color(0, 77, 13), 2)
					old_lcd_new_line1 = lcd_new_line1
				elif lcd_new_line1 == 'Set: 1-2-3-<4>-5':
					led.set_color(Color(0, 77, 13), 3)
					old_lcd_new_line1 = lcd_new_line1
				elif lcd_new_line1 == 'Set: 1-2-3-4-<5>':
					led.set_color(Color(0, 77, 13), 4)
					old_lcd_new_line1 = lcd_new_line1
		time.sleep(0.01)
		pass


def jt():
	global lcd_new_line1, butSwitch, pwm0_mpu, pwm1_mpu, pwm2_mpu, pwm3_mpu, last_step_0, last_step_1, last_step_2, last_step_3, command_input_stu, TL_setting
	if ModeSwitch == 1:
		lcd_new_line1  = '<Rotary Encoder>'
	elif ModeSwitch == 2:
		butSwitch = 1
		pwm0_mpu = pwm0
		pwm1_mpu = pwm1
		pwm2_mpu = pwm2
		pwm3_mpu = pwm3
		lcd_new_line1  = '<Movement Input>'
	elif ModeSwitch == 3:
		last_step_0 = pwm0
		last_step_1 = pwm1
		last_step_2 = pwm2
		last_step_3 = pwm3
		command_input_stu  = 1
		lcd_new_line1  = '<Keys and Setps>'
	elif ModeSwitch == 4:
		TL_setting = 1
		lcd_new_line1  = '<TimeLapse Mode>'
	elif ModeSwitch == 5:
		lcd_new_line1  = '---<Settings>---'


def led_on_off(time_set, color):
	led_stu = 0
	for i in range(0,int(time_set*5)):
		if led_stu == 0:
			led.colorWipe(color)
			led_stu = 1
		else:
			led.colorWipe(Color(0,0,0))
			led_stu = 0
		time.sleep(0.2)


def start_up():
	global lcd_new_line1, lcd_new_line2
	print('-<BE CAREFUL!!>-')
	lcd_new_line1 = '-<BE CAREFUL!!>-'
	lcd_new_line2 = 'Port0 acts in 3s'
	print('Port0 acts in 3s')
	led_on_off(2, Color(77, 7, 0))
	lcd_new_line2 = 'Port0 acts in 2s'
	print('Port0 acts in 2s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port0 acts in 1s'
	print('Port0 acts in 1s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port0 acting now'
	print('Port0 acting now')
	led_on_off(0.5, Color(77, 7, 0))
	pwm.set_pwm(0, 0, 300)

	lcd_new_line2 = 'Port1 acts in 3s'
	print('Port1 acts in 3s')
	led_on_off(2, Color(77, 7, 0))
	lcd_new_line2 = 'Port1 acts in 2s'
	print('Port1 acts in 2s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port1 acts in 1s'
	print('Port1 acts in 1s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port1 acting now'
	print('Port1 acting now')
	led_on_off(0.5, Color(77, 7, 0))
	pwm.set_pwm(1, 0, 300)

	lcd_new_line2 = 'Port2 acts in 3s'
	print('Port2 acts in 3s')
	led_on_off(2, Color(77, 7, 0))
	lcd_new_line2 = 'Port2 acts in 2s'
	print('Port2 acts in 2s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port2 acts in 1s'
	print('Port2 acts in 1s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port2 acting now'
	print('Port2 acting now')
	led_on_off(0.5, Color(77, 7, 0))
	pwm.set_pwm(2, 0, 300)

	lcd_new_line2 = 'Port3 acts in 3s'
	print('Port3 acts in 3s')
	led_on_off(2, Color(77, 7, 0))
	lcd_new_line2 = 'Port3 acts in 2s'
	print('Port3 acts in 2s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port3 acts in 1s'
	print('Port3 acts in 1s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port3 acting now'
	print('Port3 acting now')
	led_on_off(0.5, Color(77, 7, 0))
	pwm.set_pwm(3, 0, 300)

#<Keys and Setps> & <TimeLapse Mode>
command_input_stu  = 0
command_output_stu = 0

#to_pos_input= 100

deley_time = 0

servo_step_0 = []
servo_step_1 = []
servo_step_2 = []
servo_step_3 = []

timelapse_time   = []
timelapse_frames = []


def leg_base_move(port, goal_step, last_step, re_pos, to_pos):
	pwm.set_pwm(port, 0, int(last_step+(goal_step-last_step)*re_pos/to_pos))
	'''
	if last_step < goal_step:
		pwm.set_pwm(port, 0, int(last_step+(goal_step-last_step)*re_pos/to_pos))
	else:
		pwm.set_pwm(port, 0, int(last_step-(last_step-goal_step)*re_pos/to_pos))
	'''


def dove_move(input_0, input_1, input_2, input_3):
	global old_pwm0, old_pwm1, old_pwm2, old_pwm3, pwm0, pwm1, pwm2, pwm3
	range_input = max(abs(input_0-old_pwm0), abs(input_1-old_pwm1), abs(input_2-old_pwm2), abs(input_3-old_pwm3))+1
	for i in range(1, range_input):
		if command_output_stu:
			leg_base_move(0, input_0, old_pwm0, i, range_input)
			leg_base_move(1, input_1, old_pwm1, i, range_input)
			leg_base_move(2, input_2, old_pwm2, i, range_input)
			leg_base_move(3, input_3, old_pwm3, i, range_input)
			time.sleep(deley_time)
		else:
			pwm0 = int(old_pwm0+(input_0-old_pwm0)*i/range_input)
			pwm1 = int(old_pwm1+(input_1-old_pwm1)*i/range_input)
			pwm2 = int(old_pwm2+(input_2-old_pwm2)*i/range_input)
			pwm3 = int(old_pwm3+(input_3-old_pwm3)*i/range_input)
			break

	old_pwm0 = input_0
	old_pwm1 = input_1
	old_pwm2 = input_2
	old_pwm3 = input_3


def command_input():
	global pwm0, pwm1, pwm2, pwm3, butSwitch, lcd_new_line2, command_input_stu, command_output_stu
	if command_input_stu:
		buttonPress = button(3)
		if buttonPress == 0:
			butSwitch += 1
			if butSwitch > 3:
				butSwitch = 0
		elif buttonPress == 1:
			servo_step_0.append(pwm0)
			servo_step_1.append(pwm1)
			servo_step_2.append(pwm2)
			servo_step_3.append(pwm3)
			print(servo_step_0)
			print(servo_step_1)
			print(servo_step_2)
			print(servo_step_3)
		elif buttonPress == 2:
			command_input_stu  = 0
			command_output_stu = 1
			return 0

		if butSwitch == 0:
			pwm0 += rotary(port_0_speed)
			pwm0 = ctrl_range(pwm0, pwm0_max, pwm0_min)
			lcd_new_line2  = 'Port:0 PWM:%d'%pwm0
		elif butSwitch == 1:
			pwm1 += rotary(port_1_speed)
			pwm1 = ctrl_range(pwm1, pwm1_max, pwm1_min)
			lcd_new_line2  = 'Port:1 PWM:%d'%pwm1
		elif butSwitch == 2:
			pwm2 += rotary(port_2_speed)
			lcd_new_line2  = 'Port:2 PWM:%d'%pwm2
			pwm2 = ctrl_range(pwm2, pwm2_max, pwm2_min)
		elif butSwitch == 3:
			pwm3 += rotary(port_3_speed)
			lcd_new_line2  = 'Port:3 PWM:%d'%pwm3
			pwm3 = ctrl_range(pwm3, pwm3_max, pwm3_min)

		time.sleep(0.001)


def command_output():
	global command_output_stu, command_input_stu
	if command_output_stu:
		buttonPress = button(3)
		if buttonPress == 0:
			pass
		elif buttonPress == 1:
			command_input_stu  = 1
			command_output_stu = 0
			return 1
		elif buttonPress == 2:
			command_input_stu  = 1
			command_output_stu = 0
			return 1


###########<Time_Lapse>###########
TL_setting = 0
TL_start = 0

TL_step_0 = []
TL_step_1 = []
TL_step_2 = []
TL_step_3 = []

timelapse_time   = []
timelapse_frames = []

def button_TL():
	'''
	Short Press return 0
	Long Press I return 1
	Long Press II return 2
	'''
	global butSwitch, LED_set, command_input_stu, command_output_stu, TL_start
	butLast = 0
	shortPress_stu   = 0
	longPress_I_stu  = 0
	longPress_II_stu = 0
	while not GPIO.input(btn):
		LED_thread = 0
		led.set_color(Color(77, 0, 0), 4)
		led.set_color(Color(77, 0, 0), 5)
		butLast += 1
		shortPress_stu = 1
		TL_start = 0
		if butLast > longPress_I:
			TL_start = 0
			shortPress_stu  = 0
			longPress_I_stu = 1
			butLast = 0
			led_change = longPress_II
			while not GPIO.input(btn):
				led.colorWipe(Color(0, 0, led_change))
				led_change -= 1
				if led_change == 0:
					longPress_I_stu  = 0
					longPress_II_stu = 1
					led.colorWipe(Color(0, longPress_II, longPress_II))
					while not GPIO.input(btn):
						pass
		time.sleep(0.05)

	if shortPress_stu:
		led.colorWipe(Color(0, 0, 0))
		LED_set = 1
		return 0
	elif longPress_I_stu:
		led.colorWipe(Color(0, 0, 0))
		LED_set = 1
		return 1
	elif longPress_II_stu:
		led.colorWipe(Color(0, 0, 0))
		LED_set = 1
		return 2


def position_input():
	global lcd_new_line2, pwm0, pwm1, pwm2, pwm3, butSwitch, TL_setting, TL_start
	lcd_new_line2 = 'Pos Input:1'
	while 1:
		buttonPress = button_TL()
		if buttonPress == 0:
			butSwitch += 1
			if butSwitch > 3:
				butSwitch = 0
		elif buttonPress == 1:
			TL_step_0.append(pwm0)
			TL_step_1.append(pwm1)
			TL_step_2.append(pwm2)
			TL_step_3.append(pwm3)
			lcd_new_line2 = 'Pos Setted:%d'%len(TL_step_0)
			break
		elif buttonPress == 2:
			TL_step_0.append(pwm0)
			TL_step_1.append(pwm1)
			TL_step_2.append(pwm2)
			TL_step_3.append(pwm3)
			lcd_new_line2 = 'Pos Setted:%d'%len(TL_step_0)
			TL_setting = 0
			TL_start = 1
			break

		if butSwitch == 0:
			pwm0 += rotary(port_0_speed)
			pwm0 = ctrl_range(pwm0, pwm0_max, pwm0_min)
			lcd_new_line2  = 'Pos:%d Por0 %d'%(len(TL_step_0),pwm0)
		elif butSwitch == 1:
			pwm1 += rotary(port_1_speed)
			pwm1 = ctrl_range(pwm1, pwm1_max, pwm1_min)
			lcd_new_line2  = 'Pos:%d Por1 %d'%(len(TL_step_0),pwm1)
		elif butSwitch == 2:
			pwm2 += rotary(port_2_speed)
			lcd_new_line2  = 'Pos:%d Por2 %d'%(len(TL_step_0),pwm2)
			pwm2 = ctrl_range(pwm2, pwm2_max, pwm2_min)
		elif butSwitch == 3:
			pwm3 += rotary(port_3_speed)
			lcd_new_line2  = 'Pos:%d Por3 %d'%(len(TL_step_0),pwm3)
			pwm3 = ctrl_range(pwm3, pwm3_max, pwm3_min)

		time.sleep(0.001)
		pass


def time_input():
	global lcd_new_line2
	time_to_set = 0
	while 1:
		buttonPress = button_TL()
		if buttonPress == 0:
			timelapse_time.append(time_to_set)
			break
		elif buttonPress == 1:
			timelapse_time.append(time_to_set)
			break
		elif buttonPress == 2:
			timelapse_time.append(time_to_set)
			break

		time_to_set += rotary(1)
		if time_to_set < 0:
			time_to_set = 0
		lcd_new_line2 = '%ds Time Set'%time_to_set
		print('%ds Time Set'%time_to_set)
		time.sleep(0.005)
		pass


def frame_input():
	global lcd_new_line2
	frames_to_set = 0
	while 1:
		buttonPress = button_TL()
		if buttonPress == 0:
			timelapse_frames.append(frames_to_set)
			break
		elif buttonPress == 1:
			timelapse_frames.append(frames_to_set)
			break
		elif buttonPress == 2:
			timelapse_frames.append(frames_to_set)
			break

		frames_to_set += rotary(1)
		if frames_to_set < 0:
			frames_to_set = 0
		lcd_new_line2 = '%ds Frames Set'%frames_to_set
		time.sleep(0.005)
		pass


def break_TL():
	global TL_start
	buttonPress = button_TL()
	print(TL_start)
	if buttonPress == 0:
		return 1
	elif buttonPress == 1:
		TL_start = 0
		switch(4)
		return 1
	elif buttonPress == 2:
		TL_start = 0
		switch(4)
		return 1

	if TL_start == 0:
		switch(4)


def timelapse(input_0, input_1, input_2, input_3, total_time, pix):
	global old_pwm0, old_pwm1, old_pwm2, old_pwm3
	for i in range(1, pix+1):
		leg_base_move(0, input_0, old_pwm0, i, pix+1)
		leg_base_move(1, input_1, old_pwm1, i, pix+1)
		leg_base_move(2, input_2, old_pwm2, i, pix+1)
		leg_base_move(3, input_3, old_pwm3, i, pix+1)
		try:
			time.sleep(total_time/pix)
		except:
			pass
	old_pwm0 = input_0
	old_pwm1 = input_1
	old_pwm2 = input_2
	old_pwm3 = input_3

##################################

led=LED()

show_threading=threading.Thread(target=lcdled_threading) #Define a thread for LCD
show_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
show_threading.start()									 #Thread starts

base_move_threading=threading.Thread(target=base_move_thread) #Define a thread for LCD
base_move_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
base_move_threading.start()									 #Thread starts

def remote():
	global pwm0, pwm1, pwm2, pwm3, remote_speed
	print('xxx')
	HOST = ''
	PORT = 10223							  #Define port serial 
	BUFSIZ = 1024							 #Define buffer size
	ADDR = (HOST, PORT)

	'''
	try:
		s =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		s.connect(("1.1.1.1",80))
		ipaddr_check=s.getsockname()[0]
		s.close()
		print(ipaddr_check)
	except:
		ap_threading=threading.Thread(target=ap_thread)   #Define a thread for data receiving
		ap_threading.setDaemon(True)						  #'True' means it is a front thread,it would close when the mainloop() closes
		ap_threading.start()								  #Thread starts

		LED.colorWipe(Color(0,16,50))
		time.sleep(1)
		LED.colorWipe(Color(0,16,100))
		time.sleep(1)
		LED.colorWipe(Color(0,16,150))
		time.sleep(1)
		LED.colorWipe(Color(0,16,200))
		time.sleep(1)
		LED.colorWipe(Color(0,16,255))
		time.sleep(1)
		LED.colorWipe(Color(35,255,35))
	'''

	#try:
	tcpSerSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	tcpSerSock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
	tcpSerSock.bind(ADDR)
	tcpSerSock.listen(5)					  #Start server,waiting for client
	print('waiting for connection...')
	tcpCliSock, addr = tcpSerSock.accept()
	print('...connected from :', addr)

	while 1:
		data = ''
		data = str(tcpCliSock.recv(BUFSIZ).decode())
		if not data:
			continue
		elif '0+' in data:
			pwm0 += remote_speed
		elif '0-' in data:
			pwm0 -= remote_speed
		elif '1+' in data:
			pwm1 += remote_speed
		elif '1-' in data:
			pwm1 -= remote_speed
		elif '2+' in data:
			pwm2 += remote_speed
		elif '2-' in data:
			pwm2 -= remote_speed
		elif '3+' in data:
			pwm3 += remote_speed
		elif '3-' in data:
			pwm3 -= remote_speed
	#except:
		#pass


remote_threading=threading.Thread(target=remote) #Define a thread for LCD
remote_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
remote_threading.start()									 #Thread starts

#try:
	#start_up()
switch(1)

while 1:
	jt()
	while ModeSwitch == 1:
		if rotary_mode():
			break
	jt()
	while ModeSwitch == 2:
		if movement_mode():
			break
	jt()
	while ModeSwitch == 3:
		command_input()
		if command_output():
			break
	jt()
	while ModeSwitch == 4:
		while TL_setting:
			position_input()
			if TL_setting:
				time_input()
				frame_input()
			else:
				print('Setted')
				break
			print(TL_step_0)
			print(TL_step_1)
			print(TL_step_2)
			print(TL_step_3)
			print(timelapse_time)
			print(timelapse_frames)
			pass
		time.sleep(2)
		if break_TL():
			break
		pass
#except:
	#pwm = Adafruit_PCA9685.PCA9685()
	#pwm.set_pwm_freq(50)
	#led.colorWipe(Color(0,0,0))

