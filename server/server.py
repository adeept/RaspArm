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
import threading
import i2c_lcd1602
import datetime
import Adafruit_PCA9685

from rpi_ws281x import *
import argparse
import switch

import psutil

import socket

from mpu6050 import mpu6050
import Kalman_filter
kalman_filter_X =  Kalman_filter.Kalman_filter(0.001,0.1)
kalman_filter_Y =  Kalman_filter.Kalman_filter(0.001,0.1)
sensor = mpu6050(0x68)
mpu = 1
mpu_speed	= 1
Tolerance	= 1.5
auto_speed  = 2


clk = 19
dt = 16
btn = 20

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(btn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

number = 0
old_number = number
btn_input = 0
setting_select = 1
old_setting_select = setting_select

#PWM control
PWM_0 = 300
PWM_1 = 300
PWM_2 = 300
PWM_3 = 300

PWM_0_OLD = PWM_0
PWM_1_OLD = PWM_1
PWM_2_OLD = PWM_2
PWM_3_OLD = PWM_3

PWM_0_MIN = 100
PWM_1_MIN = 100
PWM_2_MIN = 100
PWM_3_MIN = 100

PWM_0_MAX = 500
PWM_1_MAX = 500
PWM_2_MAX = 500
PWM_3_MAX = 500

PWM_select = 0
old_PWM_select = PWM_select
PWM_set = 0

PWM_0_speed = 3
PWM_1_speed = 3
PWM_2_speed = 3
PWM_3_speed = 3

rotary_result = 0

lcd_last_line1 = ''
lcd_last_line2 = ''
lcd_new_line1  = ''
lcd_new_line2  = ''

#Modes
'''
0 Switch
1 <Rotary Encoder>
2 <Movement Input>
3 <Keys and Setps>
4 <TimeLapse Mode>
5 ---<Settings>---
'''
modeStates = 0

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
timestamp = datetime.datetime.now()
lastChange = timestamp
screen = i2c_lcd1602.Screen(bus=1, addr=0x27, cols=16, rows=2)
screen.enable_backlight()

DIR_step_0 = []
DIR_step_1 = []
DIR_step_2 = []
DIR_step_3 = []

mission_pwm0 = 300
mission_pwm1 = 300
mission_pwm2 = 300
mission_pwm3 = 300

timelapse_time   = []
timelapse_frames = []

time_set = 1
frame_set = 1

mission_deley_time = 0

socket_speed = 1


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
led = LED()


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
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port0 acts in 3s')
	led_on_off(2, Color(77, 7, 0))
	lcd_new_line2 = 'Port0 acts in 2s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port0 acts in 2s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port0 acts in 1s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port0 acts in 1s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port0 acting now'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port0 acting now')
	led_on_off(0.5, Color(77, 7, 0))
	pwm.set_pwm(0, 0, 300)

	lcd_new_line2 = 'Port1 acts in 3s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port1 acts in 3s')
	led_on_off(2, Color(77, 7, 0))
	lcd_new_line2 = 'Port1 acts in 2s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port1 acts in 2s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port1 acts in 1s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port1 acts in 1s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port1 acting now'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port1 acting now')
	led_on_off(0.5, Color(77, 7, 0))
	pwm.set_pwm(1, 0, 300)

	lcd_new_line2 = 'Port2 acts in 3s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port2 acts in 3s')
	led_on_off(2, Color(77, 7, 0))
	lcd_new_line2 = 'Port2 acts in 2s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port2 acts in 2s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port2 acts in 1s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port2 acts in 1s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port2 acting now'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port2 acting now')
	led_on_off(0.5, Color(77, 7, 0))
	pwm.set_pwm(2, 0, 300)

	lcd_new_line2 = 'Port3 acts in 3s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port3 acts in 3s')
	led_on_off(2, Color(77, 7, 0))
	lcd_new_line2 = 'Port3 acts in 2s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port3 acts in 2s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port3 acts in 1s'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port3 acts in 1s')
	led_on_off(1, Color(77, 7, 0))
	lcd_new_line2 = 'Port3 acting now'
	screen.display_data(lcd_new_line1, lcd_new_line2)
	print('Port3 acting now')
	led_on_off(0.5, Color(77, 7, 0))
	pwm.set_pwm(3, 0, 300)
start_up()


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


def LCD_change():
	global lcd_new_line1, lcd_new_line2
	if old_setting_select != setting_select:
		led.colorWipe(Color(0, 0, 0))
	if old_PWM_select != PWM_select:
		led.colorWipe(Color(0, 0, 0))
	if modeStates == 0:
		if setting_select == 1:
			lcd_new_line1  = 'Set: <1>-2-3-4-5'
			lcd_new_line2  = '<Rotary Encoder>'
			led.set_color(Color(0, 77, 13), 0)
		elif setting_select == 2:
			lcd_new_line1  = 'Set: 1-<2>-3-4-5'
			lcd_new_line2  = '<Movement Input>'
			led.set_color(Color(0, 77, 13), 1)
		elif setting_select == 3:
			lcd_new_line1  = 'Set: 1-2-<3>-4-5'
			lcd_new_line2  = '<Keys and Setps>'
			led.set_color(Color(0, 77, 13), 2)
		elif setting_select == 4:
			lcd_new_line1  = 'Set: 1-2-3-<4>-5'
			lcd_new_line2  = '<TimeLapse Mode>'
			led.set_color(Color(0, 77, 13), 3)
		elif setting_select == 5:
			lcd_new_line1  = 'Set: 1-2-3-4-<5>'
			lcd_new_line2  = '---<Settings>---'
			led.set_color(Color(0, 77, 13), 4)
	elif modeStates == 1:
		lcd_new_line1  = '<Rotary Encoder>'
		if PWM_select == 0:
			lcd_new_line2  = 'Port:0 PWM:%d'%PWM_0
			led.set_color(Color(0, 77, 13), 0)
		elif PWM_select == 1:
			lcd_new_line2  = 'Port:1 PWM:%d'%PWM_1
			led.set_color(Color(0, 77, 13), 1)
		elif PWM_select == 2:
			lcd_new_line2  = 'Port:2 PWM:%d'%PWM_2
			led.set_color(Color(0, 77, 13), 2)
		elif PWM_select == 3:
			lcd_new_line2  = 'Port:3 PWM:%d'%PWM_3
			led.set_color(Color(0, 77, 13), 3)
	elif modeStates == 2:
		lcd_new_line1  = '<Movement Input>'
		if PWM_select == 0:
			lcd_new_line2  = 'Port:0 PWM:%d'%PWM_0
			led.set_color(Color(0, 77, 13), 0)
		elif PWM_select == 1:
			lcd_new_line2  = 'Port:1 PWM:%d'%PWM_1
			led.set_color(Color(0, 77, 13), 1)
		elif PWM_select == 2:
			lcd_new_line2  = 'Port:2 PWM:%d'%PWM_2
			led.set_color(Color(0, 77, 13), 2)
		elif PWM_select == 3:
			lcd_new_line2  = 'Port:3 PWM:%d'%PWM_3
			led.set_color(Color(0, 77, 13), 3)
	elif modeStates == 3:
		lcd_new_line1  = '<Keys and Steps>'
		if PWM_set == 1:
			if PWM_select == 0:
				lcd_new_line2  = 'Port:0 PWM:%d'%PWM_0
				led.set_color(Color(0, 77, 13), 0)
			elif PWM_select == 1:
				lcd_new_line2  = 'Port:1 PWM:%d'%PWM_1
				led.set_color(Color(0, 77, 13), 1)
			elif PWM_select == 2:
				lcd_new_line2  = 'Port:2 PWM:%d'%PWM_2
				led.set_color(Color(0, 77, 13), 2)
			elif PWM_select == 3:
				lcd_new_line2  = 'Port:3 PWM:%d'%PWM_3
				led.set_color(Color(0, 77, 13), 3)
		elif PWM_set == 2:
			lcd_new_line1 = 'PWM0:%dPWM1:%d'%(mission_pwm0,mission_pwm1)
			lcd_new_line2 = 'PWM2:%dPWM3:%d'%(mission_pwm2,mission_pwm3)
			led.colorWipe(Color(0, 77, 13))
	elif modeStates == 4:
		lcd_new_line1  = '<TimeLapse Mode>'
		if PWM_set == 1:
			if PWM_select == 0:
				lcd_new_line2  = 'Port:0 PWM:%d'%PWM_0
				led.set_color(Color(0, 77, 13), 0)
			elif PWM_select == 1:
				lcd_new_line2  = 'Port:1 PWM:%d'%PWM_1
				led.set_color(Color(0, 77, 13), 1)
			elif PWM_select == 2:
				lcd_new_line2  = 'Port:2 PWM:%d'%PWM_2
				led.set_color(Color(0, 77, 13), 2)
			elif PWM_select == 3:
				lcd_new_line2  = 'Port:3 PWM:%d'%PWM_3
				led.set_color(Color(0, 77, 13), 3)
		elif PWM_set == 2:
			lcd_new_line2  = 'Time Input%d'%time_set
		elif PWM_set == 3:
			lcd_new_line2  = 'FrameInput%d'%frame_set
		elif PWM_set == 4:
			lcd_new_line1 = 'PWM0:%dPWM1:%d'%(mission_pwm0,mission_pwm1)
			lcd_new_line2 = 'PWM2:%dPWM3:%d'%(mission_pwm2,mission_pwm3)
			led.colorWipe(Color(0, 77, 13))
	LCD_screen.resume()


class LCD_ctrl(threading.Thread):
	def __init__(self, *args, **kwargs):
		super(LCD_ctrl, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.set()
		self.__running = threading.Event()
		self.__running.set()

	def run(self):
		global lcd_last_line1, lcd_last_line2, lastChange
		while self.__running.isSet():
			self.__flag.wait()	  # 为True时立即返回, 为False时阻塞直到内部的标识位为True后返回
			timestamp = datetime.datetime.now()
			if lcd_last_line1 != lcd_new_line1 or lcd_last_line2 != lcd_new_line2:
				if (timestamp - lastChange).seconds >= 0.01:
					screen.display_data(lcd_new_line1, lcd_new_line2)
					lastChange = timestamp
					lcd_last_line1 = lcd_new_line1
					lcd_last_line2 = lcd_new_line2
				else:
					time.sleep(0.01)
					screen.display_data(lcd_new_line1, lcd_new_line2)
					lastChange = timestamp
					lcd_last_line1 = lcd_new_line1
					lcd_last_line2 = lcd_new_line2
			LCD_screen.pause()

	def pause(self):
		self.__flag.clear()	 # 设置为False, 让线程阻塞

	def resume(self):
		self.__flag.set()	# 设置为True, 让线程停止阻塞

	def stop(self):
		self.__flag.set()	   # 将线程从暂停状态恢复, 如何已经暂停的话
		self.__running.clear()		# 设置为False  


def rotary_encoder_Mode():
	global PWM_0_OLD, PWM_1_OLD, PWM_2_OLD, PWM_3_OLD
	if PWM_select == 0:
		pwm.set_pwm(0, 0, PWM_0)
	elif PWM_select == 1:
		pwm.set_pwm(1, 0, PWM_1)
	elif PWM_select == 2:
		pwm.set_pwm(2, 0, PWM_2)
	elif PWM_select == 3:
		pwm.set_pwm(3, 0, PWM_3)
	PWM_0_OLD = PWM_0
	PWM_1_OLD = PWM_1
	PWM_2_OLD = PWM_2
	PWM_3_OLD = PWM_3


def movement_Mode():
	global PWM_0, PWM_1, PWM_2, PWM_3, PWM_0_OLD, PWM_1_OLD, PWM_2_OLD, PWM_3_OLD
	accelerometer_data = sensor.get_accel_data()
	X = accelerometer_data['x']
	X = kalman_filter_X.kalman(X)
	Y = accelerometer_data['y']
	Y = kalman_filter_Y.kalman(Y)
	if abs(Y) > Tolerance:
		print(Y)
		PWM_0 += Y*mpu_speed
		print(PWM_0)
		PWM_0 = ctrl_range(PWM_0, PWM_0_MAX, PWM_0_MIN)
		pwm.set_pwm(0, 0, PWM_0)
		PWM_0_OLD = PWM_0

	if PWM_3_OLD != PWM_3:
		pwm.set_pwm(3, 0, PWM_3)
		PWM_3_OLD = PWM_3

	if abs(X) > Tolerance:
		if PWM_select == 1:
			PWM_1 += X*mpu_speed
			PWM_1 = ctrl_range(PWM_1, PWM_1_MAX, PWM_1_MIN)
			pwm.set_pwm(1, 0, PWM_1)
			PWM_1_OLD = PWM_1
		elif PWM_select == 2:
			PWM_2 += X*mpu_speed
			PWM_2 = ctrl_range(PWM_2, PWM_2_MAX, PWM_2_MIN)
			pwm.set_pwm(2, 0, PWM_2)
			PWM_2_OLD = PWM_2
	time.sleep(0.1)


def button_input_change(args):
	global modeStates, PWM_select, PWM_set, setting_select, btn_input, old_PWM_select, old_setting_select, time_set, frame_set
	old_PWM_select = PWM_select
	old_setting_select = setting_select
	if modeStates == 1:
		if args == 1:
			#PWM change
			PWM_select += 1
			if PWM_select == 4:
				PWM_select = 0
		elif args == 2:
			#Nothing
			pass
		elif args == 3:
			#to switch
			modeStates = 0
	elif modeStates == 2:
		if args == 1:
			#PWM change
			if PWM_select == 2:
				PWM_select = 1
			elif PWM_select == 1:
				PWM_select = 2
			else:
				PWM_select = 1
		elif args == 2:
			#Nothing
			pass
		elif args == 3:
			#to switch
			modeStates = 0


	elif modeStates == 3:
		if args == 1:
			#PWM change
			PWM_select += 1
			if PWM_select == 4:
				PWM_select = 0
		elif args == 2:
			#PWM setting mode on
			if PWM_set == 1:
				DIR_step_0.append(PWM_0)
				DIR_step_1.append(PWM_1)
				DIR_step_2.append(PWM_2)
				DIR_step_3.append(PWM_3)
				print(DIR_step_0)
			else:
				pass
		elif args == 3:
			#to switch
			if PWM_set == 1:
				PWM_set = 2
			elif PWM_set == 2:
				PWM_set = 0
				modeStates = 0


	elif modeStates == 4:
		if args == 1:
			#PWM change
			PWM_select += 1
			if PWM_select == 4:
				PWM_select = 0
		elif args == 2:
			#PWM setting mode on
			if PWM_set == 1:
				DIR_step_0.append(PWM_0)
				DIR_step_1.append(PWM_1)
				DIR_step_2.append(PWM_2)
				DIR_step_3.append(PWM_3)
				print(DIR_step_0)
				PWM_set = 2
			elif PWM_set == 2:
				timelapse_time.append(time_set)
				time_set = 1
				PWM_set = 3
			elif PWM_set == 3:
				timelapse_frames.append(frame_set)
				frame_set = 1
				PWM_set = 1
		elif args == 3:
			#to switch
			if PWM_set == 1:
				PWM_set = 4
			elif PWM_set == 4:
				modeStates = 0
	elif modeStates == 5:
		modeStates = 0


	elif modeStates == 0:
		if args == 1:
			if modeStates == 2:
				PWM_select = 2
			modeStates = setting_select
	btn_input = 0


def rotary_input_change():
	global PWM_0, PWM_1, PWM_2, PWM_3, setting_select, rotary_result, mission_deley_time, time_set, frame_set
	if modeStates == 1:
		if PWM_select == 0:
			if rotary_result == 1:
				PWM_0 += PWM_0_speed
				PWM_0 = ctrl_range(PWM_0, PWM_0_MAX, PWM_0_MIN)
			elif rotary_result == -1:
				PWM_0 -= PWM_0_speed
				PWM_0 = ctrl_range(PWM_0, PWM_0_MAX, PWM_0_MIN)
		elif PWM_select == 1:
			if rotary_result == 1:
				PWM_1 += PWM_1_speed
				PWM_1 = ctrl_range(PWM_1, PWM_1_MAX, PWM_1_MIN)
			elif rotary_result == -1:
				PWM_1 -= PWM_1_speed
				PWM_1 = ctrl_range(PWM_1, PWM_1_MAX, PWM_1_MIN)
		elif PWM_select == 2:
			if rotary_result == 1:
				PWM_2 += PWM_2_speed
				PWM_2 = ctrl_range(PWM_2, PWM_2_MAX, PWM_2_MIN)
			elif rotary_result == -1:
				PWM_2 -= PWM_2_speed
				PWM_2 = ctrl_range(PWM_2, PWM_2_MAX, PWM_2_MIN)
		elif PWM_select == 3:
			if rotary_result == 1:
				PWM_3 += PWM_3_speed
				PWM_3 = ctrl_range(PWM_3, PWM_3_MAX, PWM_3_MIN)
			elif rotary_result == -1:
				PWM_3 -= PWM_3_speed
				PWM_3 = ctrl_range(PWM_3, PWM_3_MAX, PWM_3_MIN)
	elif modeStates == 2:
		if rotary_result == 1:
			PWM_3 += PWM_3_speed
			PWM_3 = ctrl_range(PWM_3, PWM_3_MAX, PWM_3_MIN)
		elif rotary_result == -1:
			PWM_3 -= PWM_3_speed
			PWM_3 = ctrl_range(PWM_3, PWM_3_MAX, PWM_3_MIN)


	elif modeStates == 3:
		if PWM_set == 1:
			if PWM_select == 0:
				if rotary_result == 1:
					PWM_0 += PWM_0_speed
					PWM_0 = ctrl_range(PWM_0, PWM_0_MAX, PWM_0_MIN)
				elif rotary_result == -1:
					PWM_0 -= PWM_0_speed
					PWM_0 = ctrl_range(PWM_0, PWM_0_MAX, PWM_0_MIN)
			elif PWM_select == 1:
				if rotary_result == 1:
					PWM_1 += PWM_1_speed
					PWM_1 = ctrl_range(PWM_1, PWM_1_MAX, PWM_1_MIN)
				elif rotary_result == -1:
					PWM_1 -= PWM_1_speed
					PWM_1 = ctrl_range(PWM_1, PWM_1_MAX, PWM_1_MIN)
			elif PWM_select == 2:
				if rotary_result == 1:
					PWM_2 += PWM_2_speed
					PWM_2 = ctrl_range(PWM_2, PWM_2_MAX, PWM_2_MIN)
				elif rotary_result == -1:
					PWM_2 -= PWM_2_speed
					PWM_2 = ctrl_range(PWM_2, PWM_2_MAX, PWM_2_MIN)
			elif PWM_select == 3:
				if rotary_result == 1:
					PWM_3 += PWM_3_speed
					PWM_3 = ctrl_range(PWM_3, PWM_3_MAX, PWM_3_MIN)
				elif rotary_result == -1:
					PWM_3 -= PWM_3_speed
					PWM_3 = ctrl_range(PWM_3, PWM_3_MAX, PWM_3_MIN)
		elif PWM_set == 2:
			if rotary_result == 1:
				mission_deley_time += 0.05
			elif rotary_result == -1:
				if mission_deley_time <= 0.05:
					mission_deley_time = 0
				else:
					mission_deley_time -= 0.05


	elif modeStates == 4:
		if PWM_set == 1:
			if PWM_select == 0:
				if rotary_result == 1:
					PWM_0 += PWM_0_speed
					PWM_0 = ctrl_range(PWM_0, PWM_0_MAX, PWM_0_MIN)
				elif rotary_result == -1:
					PWM_0 -= PWM_0_speed
					PWM_0 = ctrl_range(PWM_0, PWM_0_MAX, PWM_0_MIN)
			elif PWM_select == 1:
				if rotary_result == 1:
					PWM_1 += PWM_1_speed
					PWM_1 = ctrl_range(PWM_1, PWM_1_MAX, PWM_1_MIN)
				elif rotary_result == -1:
					PWM_1 -= PWM_1_speed
					PWM_1 = ctrl_range(PWM_1, PWM_1_MAX, PWM_1_MIN)
			elif PWM_select == 2:
				if rotary_result == 1:
					PWM_2 += PWM_2_speed
					PWM_2 = ctrl_range(PWM_2, PWM_2_MAX, PWM_2_MIN)
				elif rotary_result == -1:
					PWM_2 -= PWM_2_speed
					PWM_2 = ctrl_range(PWM_2, PWM_2_MAX, PWM_2_MIN)
			elif PWM_select == 3:
				if rotary_result == 1:
					PWM_3 += PWM_3_speed
					PWM_3 = ctrl_range(PWM_3, PWM_3_MAX, PWM_3_MIN)
				elif rotary_result == -1:
					PWM_3 -= PWM_3_speed
					PWM_3 = ctrl_range(PWM_3, PWM_3_MAX, PWM_3_MIN)
		elif PWM_set == 2:
			#time_input
			if rotary_result == 1:
				time_set += 1
			elif rotary_result == -1:
				if time_set <= 2:
					time_set = 1
				else:
					time_set -= 1
			pass
		elif PWM_set == 3:
			#frame_input
			if rotary_result == 1:
				frame_set += 1
			elif rotary_result == -1:
				if frame_set <= 2:
					frame_set = 1
				else:
					frame_set -= 1
			pass
		elif PWM_set == 4:
			#_input
			pass
	elif modeStates == 5:
		#Nothing
		pass
	elif modeStates == 0:
		if rotary_result == 1:
			setting_select += 1
		elif rotary_result == -1:
			setting_select -= 1
		if setting_select == 6:
			setting_select = 1
		if setting_select == 0:
			setting_select = 5
		print(setting_select)
		time.sleep(0.2)
	rotary_result = 0


def keys_and_steps():
	global PWM_set, DIR_step_0, DIR_step_1, DIR_step_2, DIR_step_3
	DIR_step_0 = []
	DIR_step_1 = []
	DIR_step_2 = []
	DIR_step_3 = []
	PWM_set = 1

	def leg_base_move(port, goal_step, last_step, re_pos, to_pos):
		pwm_now_input = int(last_step+(goal_step-last_step)*re_pos/to_pos)
		pwm.set_pwm(port, 0, pwm_now_input)
		return pwm_now_input

	def dove_servo(input_0, input_1, input_2, input_3):
		global PWM_0_OLD, PWM_1_OLD, PWM_2_OLD, PWM_3_OLD, mission_pwm0, mission_pwm1, mission_pwm2, mission_pwm3
		range_input = max(abs(input_0-PWM_0_OLD), abs(input_1-PWM_1_OLD), abs(input_2-PWM_2_OLD), abs(input_3-PWM_3_OLD))+1
		for i in range(1, range_input, auto_speed):
			rotary_input_change()
			button_input_change(btn_input)
			if modeStates == 3:
				mission_pwm0 = leg_base_move(0, input_0, PWM_0_OLD, i, range_input)
				mission_pwm1 = leg_base_move(1, input_1, PWM_1_OLD, i, range_input)
				mission_pwm2 = leg_base_move(2, input_2, PWM_2_OLD, i, range_input)
				mission_pwm3 = leg_base_move(3, input_3, PWM_3_OLD, i, range_input)
				if btn_input == 0:
					LCD_change()
				time.sleep(mission_deley_time)
			else:
				PWM_0_OLD = mission_pwm0
				PWM_1_OLD = mission_pwm1
				PWM_2_OLD = mission_pwm2
				PWM_3_OLD = mission_pwm3
				break
		PWM_0_OLD = mission_pwm0
		PWM_1_OLD = mission_pwm1
		PWM_2_OLD = mission_pwm2
		PWM_3_OLD = mission_pwm3


	def kas_command_input():
		for i in range(0,len(DIR_step_0)):
			if modeStates == 3:
				dove_servo(DIR_step_0[i],DIR_step_1[i],DIR_step_2[i],DIR_step_3[i])
				print('>>>>>>%d<<<<<<'%i)
			else:
				break
		dove_servo(DIR_step_0[0],DIR_step_1[0],DIR_step_2[0],DIR_step_3[0])

	while PWM_set == 1:
		rotary_input_change()
		button_input_change(btn_input)
		rotary_encoder_Mode()
		LCD_change()
	while PWM_set == 2:
		kas_command_input()
		pass


def time_lapse():
	global PWM_set, DIR_step_0, DIR_step_1, DIR_step_2, DIR_step_3
	DIR_step_0 = []
	DIR_step_1 = []
	DIR_step_2 = []
	DIR_step_3 = []
	PWM_set = 1

	def leg_base_move(port, goal_step, last_step, re_pos, to_pos):
		pwm_now_input = int(last_step+(goal_step-last_step)*re_pos/to_pos)
		pwm.set_pwm(port, 0, pwm_now_input)
		return pwm_now_input

	def doveTL_servo(input_0, input_1, input_2, input_3):
		global PWM_0_OLD, PWM_1_OLD, PWM_2_OLD, PWM_3_OLD, mission_pwm0, mission_pwm1, mission_pwm2, mission_pwm3
		range_input = max(abs(input_0-PWM_0_OLD), abs(input_1-PWM_1_OLD), abs(input_2-PWM_2_OLD), abs(input_3-PWM_3_OLD))+1
		for i in range(1, range_input, auto_speed):
			rotary_input_change()
			button_input_change(btn_input)
			if modeStates == 4:
				mission_pwm0 = leg_base_move(0, input_0, PWM_0_OLD, i, range_input)
				mission_pwm1 = leg_base_move(1, input_1, PWM_1_OLD, i, range_input)
				mission_pwm2 = leg_base_move(2, input_2, PWM_2_OLD, i, range_input)
				mission_pwm3 = leg_base_move(3, input_3, PWM_3_OLD, i, range_input)
				if btn_input == 0:
					LCD_change()
				time.sleep(mission_deley_time)
			else:
				PWM_0_OLD = mission_pwm0
				PWM_1_OLD = mission_pwm1
				PWM_2_OLD = mission_pwm2
				PWM_3_OLD = mission_pwm3
				break
		PWM_0_OLD = mission_pwm0
		PWM_1_OLD = mission_pwm1
		PWM_2_OLD = mission_pwm2
		PWM_3_OLD = mission_pwm3

	def timeLapse(input_0, input_1, input_2, input_3, total_time, pix):
		global PWM_0_OLD, PWM_1_OLD, PWM_2_OLD, PWM_3_OLD, mission_pwm0, mission_pwm1, mission_pwm2, mission_pwm3
		range_input = max(abs(input_0-PWM_0_OLD), abs(input_1-PWM_1_OLD), abs(input_2-PWM_2_OLD), abs(input_3-PWM_3_OLD))+1
		for i in range(1, pix+1):
			rotary_input_change()
			button_input_change(btn_input)
			if modeStates == 4:
				mission_pwm0 = leg_base_move(0, input_0, PWM_0_OLD, i, pix+1)
				mission_pwm1 = leg_base_move(1, input_1, PWM_1_OLD, i, pix+1)
				mission_pwm2 = leg_base_move(2, input_2, PWM_2_OLD, i, pix+1)
				mission_pwm3 = leg_base_move(3, input_3, PWM_3_OLD, i, pix+1)
				if btn_input == 0:
					LCD_change()
				time.sleep(round((total_time/pix),2))
			else:
				PWM_0_OLD = mission_pwm0
				PWM_1_OLD = mission_pwm1
				PWM_2_OLD = mission_pwm2
				PWM_3_OLD = mission_pwm3
				break
		PWM_0_OLD = mission_pwm0
		PWM_1_OLD = mission_pwm1
		PWM_2_OLD = mission_pwm2
		PWM_3_OLD = mission_pwm3

	def tla_command_input():
		global PWM_0_OLD, PWM_1_OLD, PWM_2_OLD, PWM_3_OLD
		PWM_0_OLD = DIR_step_0[0]
		PWM_1_OLD = DIR_step_1[0]
		PWM_2_OLD = DIR_step_2[0]
		PWM_3_OLD = DIR_step_3[0]
		for i in range(0,len(DIR_step_0)):
			if modeStates == 4:
				if i == 0:
					timeLapse(DIR_step_0[i], DIR_step_1[i], DIR_step_2[i], DIR_step_3[i],3, 3)
				else:
					timeLapse(DIR_step_0[i], DIR_step_1[i], DIR_step_2[i], DIR_step_3[i], timelapse_time[i-1], timelapse_frames[i-1])
			else:
				break
		if modeStates == 4:
			doveTL_servo(DIR_step_0[0],DIR_step_1[0],DIR_step_2[0],DIR_step_3[0])

	while modeStates ==4:
		while PWM_set == 1:
			rotary_input_change()
			button_input_change(btn_input)
			rotary_encoder_Mode()
			LCD_change()
			pass
		while PWM_set == 2:  #Time Input
			rotary_input_change()
			button_input_change(btn_input)
			LCD_change()
			time.sleep(0.1)
			pass
		while PWM_set == 3:  #Frame Input
			rotary_input_change()
			button_input_change(btn_input)
			LCD_change()
			time.sleep(0.1)
			pass
		while PWM_set == 4 and modeStates == 4:  #Run
			tla_command_input()
			LCD_change()
			pass



class Output_ctrl(threading.Thread):
	def __init__(self, *args, **kwargs):
		super(Output_ctrl, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.set()
		self.__running = threading.Event()
		self.__running.set()

	def run(self):
		global btn_input, PWM_select
		while self.__running.isSet():
			self.__flag.wait()
			button_input_change(btn_input)
			rotary_input_change()
			LCD_change()

			if modeStates == 1:
				rotary_encoder_Mode()
			elif modeStates == 2:
				PWM_select = 1
				while modeStates == 2:
					rotary_input_change()
					button_input_change(btn_input)
					LCD_change()
					movement_Mode()
				LCD_change()
			elif modeStates == 3:
				keys_and_steps()
				LCD_change()
			elif modeStates == 4:
				time_lapse()

			if btn_input == 1:
				print('>>>1<<<')
			elif btn_input == 2:
				print('>>>2<<<')
			elif btn_input == 3:
				print('>>>3<<<')
			btn_input = 0

			#print('PWM_select:%d'%PWM_select)
			#print('MOD_select:%d'%modeStates)
			output_move.pause()


	def pause(self):
		self.__flag.clear()

	def resume(self):
		self.__flag.set()

	def stop(self):
		self.__flag.set()
		self.__running.clear()


LCD_screen = LCD_ctrl()
LCD_screen.start()
LCD_screen.pause()

output_move = Output_ctrl()
output_move.start()
output_move.pause()

clkLastState = GPIO.input(clk)
def rotary_input(delay_time):
	global clkLastState, old_number, number
	if GPIO.wait_for_edge(clk, GPIO.FALLING):
		clkState = GPIO.input(clk)
		dtState = GPIO.input(dt)
		btnState = GPIO.input(btn)
		if clkState != clkLastState:
			if dtState != clkState:
				number -= 1
				return -1
			else:
				number += 1
				return 1
		clkLastState = clkState
		if number != old_number:
			old_number = number
			time.sleep(delay_time)


#>>> Threading for Button <<<
def button_Thread():
	global btn_input
	while 1:
		while not GPIO.input(btn):
			btn_press_time = 0
			btn_press = 1
			led.colorWipe(Color(77, 0, 0))
			while not GPIO.input(btn):
				btn_press_time += 1
				if btn_press_time > 5:
					btn_press = 2
					led.colorWipe(Color(77, 77, 0))
					while not GPIO.input(btn):
						btn_press_time += 1
						if btn_press_time > 15:
							btn_press = 3
							led.colorWipe(Color(0, 77, 77))
							while not GPIO.input(btn):
								time.sleep(0.2)
								pass
						time.sleep(0.2)
						pass
				time.sleep(0.2)
				pass
			led.colorWipe(Color(0, 0, 0))
			btn_input = btn_press
			output_move.resume()
			time.sleep(0.2)
		time.sleep(0.2)



btn_threading=threading.Thread(target=button_Thread) #Define a thread for LCD
btn_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
btn_threading.start()									 #Thread starts


def server_setup():
	global PWM_0, PWM_1, PWM_2, PWM_3, socket_speed, PWM_0_OLD, PWM_1_OLD, PWM_2_OLD, PWM_3_OLD
	HOST = ''
	PORT = 10223							  #Define port serial 
	BUFSIZ = 1024							 #Define buffer size
	ADDR = (HOST, PORT)

	def ap_thread():
		os.system("sudo create_ap wlan0 eth0 AdeeptCar 12345678")


	def get_cpu_tempfunc():
		""" Return CPU temperature """
		result = 0
		mypath = "/sys/class/thermal/thermal_zone0/temp"
		with open(mypath, 'r') as mytmpfile:
			for line in mytmpfile:
				result = line

		result = float(result)/1000
		result = round(result, 1)
		return str(result)


	def get_gpu_tempfunc():
		""" Return GPU temperature as a character string"""
		res = os.popen('/opt/vc/bin/vcgencmd measure_temp').readline()
		return res.replace("temp=", "")


	def get_cpu_use():
		""" Return CPU usage using psutil"""
		cpu_cent = psutil.cpu_percent()
		return str(cpu_cent)


	def get_ram_info():
		""" Return RAM usage using psutil """
		ram_cent = psutil.virtual_memory()[2]
		return str(ram_cent)


	def get_swap_info():
		""" Return swap memory  usage using psutil """
		swap_cent = psutil.swap_memory()[3]
		return str(swap_cent)


	def info_get():
		global cpu_t,cpu_u,gpu_t,ram_info
		while 1:
			cpu_t = get_cpu_tempfunc()
			cpu_u = get_cpu_use()
			ram_info = get_ram_info()
			time.sleep(3)

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

	tcpSerSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	tcpSerSock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
	tcpSerSock.bind(ADDR)
	tcpSerSock.listen(5)					  #Start server,waiting for client
	print('waiting for connection...')
	tcpCliSock, addr = tcpSerSock.accept()
	print('...connected from :', addr)
	switch.switchSetup()

	def info_send_client():
		SERVER_IP = addr[0]
		SERVER_PORT = 2256   #Define port serial 
		SERVER_ADDR = (SERVER_IP, SERVER_PORT)
		Info_Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #Set connection value for socket
		Info_Socket.connect(SERVER_ADDR)
		print(SERVER_ADDR)
		while 1:
			Info_Socket.send((get_cpu_tempfunc()+' '+get_cpu_use()+' '+get_ram_info()).encode())
			time.sleep(1)

	info_threading=threading.Thread(target=info_send_client)	#Define a thread for FPV and OpenCV
	info_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
	info_threading.start()									 #Thread starts
	while True: 
		data = ''
		data = str(tcpCliSock.recv(BUFSIZ).decode())
		if not data:
			continue
		elif '0+' == data:
			PWM_select = 0
			PWM_0 += socket_speed
			pwm.set_pwm(0,0,PWM_0)
			PWM_0_OLD = PWM_0
		elif '0-' == data:
			PWM_select = 0
			PWM_0 -= socket_speed
			pwm.set_pwm(0,0,PWM_0)
			PWM_0_OLD = PWM_0
		elif '1+' == data:
			PWM_select = 1
			PWM_1 += socket_speed
			pwm.set_pwm(1,0,PWM_1)
			PWM_1_OLD = PWM_1
		elif '1-' == data:
			PWM_select = 1
			PWM_1 -= socket_speed
			pwm.set_pwm(1,0,PWM_1)
			PWM_1_OLD = PWM_1
		elif '2+' == data:
			PWM_select = 2
			PWM_2 += socket_speed
			pwm.set_pwm(2,0,PWM_2)
			PWM_2_OLD = PWM_2
		elif '2-' == data:
			PWM_select = 2
			PWM_2 -= socket_speed
			pwm.set_pwm(2,0,PWM_2)
			PWM_2_OLD = PWM_2
		elif '3+' == data:
			PWM_select = 3
			PWM_3 += socket_speed
			pwm.set_pwm(3,0,PWM_3)
			PWM_3_OLD = PWM_3
		elif '3-' == data:
			PWM_select = 3
			PWM_3 -= socket_speed
			pwm.set_pwm(3,0,PWM_3)
			PWM_3_OLD = PWM_3
		elif 'Switch_1_on' == data:
			switch.switch(1,1)
		elif 'Switch_2_on' == data:
			switch.switch(2,1)
		elif 'Switch_3_on' == data:
			switch.switch(3,1)

		elif 'Switch_1_off' == data:
			switch.switch(1,0)
		elif 'Switch_2_off' == data:
			switch.switch(2,0)
		elif 'Switch_3_off' == data:
			switch.switch(3,0)

		elif 'wsB' in data:
			try:
				set_B=data.split()
				socket_speed = int(set_B[1])
			except:
				pass
		LCD_change()
		print(data)


socket_threading=threading.Thread(target=server_setup) #Define a thread for LCD
socket_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
socket_threading.start()									 #Thread starts


#>>> Main_loop Rotary  <<<
while 1:
	rotary_result = rotary_input(0.015)
	if rotary_result == 1:
		output_move.resume()
	elif rotary_result == -1:
		output_move.resume()
	pass