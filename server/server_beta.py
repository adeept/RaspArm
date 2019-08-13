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

#MPU_6050

from mpu6050 import mpu6050
import Kalman_filter
kalman_filter_X =  Kalman_filter.Kalman_filter(0.001,0.1)
kalman_filter_Y =  Kalman_filter.Kalman_filter(0.001,0.1)
sensor = mpu6050(0x68)
mpu = 1
mpu_speed	= 1
Tolerance	= 1.5


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
PWM_set = 0

PWM_0_speed = 1
PWM_1_speed = 1
PWM_2_speed = 1
PWM_3_speed = 1

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
	if modeStates == 0:
		if setting_select == 1:
			lcd_new_line1  = 'Set: <1>-2-3-4-5'
			lcd_new_line2  = '<Rotary Encoder>'
		elif setting_select == 2:
			lcd_new_line1  = 'Set: 1-<2>-3-4-5'
			lcd_new_line2  = '<Movement Input>'
		elif setting_select == 3:
			lcd_new_line1  = 'Set: 1-2-<3>-4-5'
			lcd_new_line2  = '<Keys and Setps>'
		elif setting_select == 4:
			lcd_new_line1  = 'Set: 1-2-3-<4>-5'
			lcd_new_line2  = '<TimeLapse Mode>'
		elif setting_select == 5:
			lcd_new_line1  = 'Set: 1-2-3-4-<5>'
			lcd_new_line2  = '---<Settings>---'
	elif modeStates == 1:
		lcd_new_line1  = '<Rotary Encoder>'
		if PWM_select == 0:
			lcd_new_line2  = 'Port:0 PWM:%d'%PWM_0
		elif PWM_select == 1:
			lcd_new_line2  = 'Port:1 PWM:%d'%PWM_1
		elif PWM_select == 2:
			lcd_new_line2  = 'Port:2 PWM:%d'%PWM_2
		elif PWM_select == 3:
			lcd_new_line2  = 'Port:3 PWM:%d'%PWM_3
	elif modeStates == 2:
		lcd_new_line1  = '<Movement Input>'
		if PWM_select == 0:
			lcd_new_line2  = 'Port:0 PWM:%d'%PWM_0
		elif PWM_select == 1:
			lcd_new_line2  = 'Port:1 PWM:%d'%PWM_1
		elif PWM_select == 2:
			lcd_new_line2  = 'Port:2 PWM:%d'%PWM_2
		elif PWM_select == 3:
			lcd_new_line2  = 'Port:3 PWM:%d'%PWM_3
	elif modeStates == 3:
		lcd_new_line1  = '<Keys and Setps>'
		if PWM_select == 0:
			lcd_new_line2  = 'Port:0 PWM:%d'%PWM_0
		elif PWM_select == 1:
			lcd_new_line2  = 'Port:1 PWM:%d'%PWM_1
		elif PWM_select == 2:
			lcd_new_line2  = 'Port:2 PWM:%d'%PWM_2
		elif PWM_select == 3:
			lcd_new_line2  = 'Port:3 PWM:%d'%PWM_3
	elif modeStates == 4:
		lcd_new_line1  = '<TimeLapse Mode>'
		if PWM_select == 0:
			lcd_new_line2  = 'Port:0 PWM:%d'%PWM_0
		elif PWM_select == 1:
			lcd_new_line2  = 'Port:1 PWM:%d'%PWM_1
		elif PWM_select == 2:
			lcd_new_line2  = 'Port:2 PWM:%d'%PWM_2
		elif PWM_select == 3:
			lcd_new_line2  = 'Port:3 PWM:%d'%PWM_3
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
			print('LCD')
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
			continue

	def pause(self):
		self.__flag.clear()	 # 设置为False, 让线程阻塞

	def resume(self):
		self.__flag.set()	# 设置为True, 让线程停止阻塞

	def stop(self):
		self.__flag.set()	   # 将线程从暂停状态恢复, 如何已经暂停的话
		self.__running.clear()		# 设置为False  


def rotary_encoder_Mode():
	pwm.set_pwm(PWM_select, 0, PWM_0)


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
			PWM_2 = ctrl_range(PWM_2, PWM_2_MAX, PWM_2_MIX)
			pwm.set_pwm(2, 0, PWM_2)
			PWM_2_OLD = PWM_2
	time.sleep(0.1)


def button_input_change(args):
	global modeStates, PWM_select, PWM_set, setting_select, btn_input
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
			PWM_set = 1
		elif args == 3:
			#to switch
			modeStates = 0
	elif modeStates == 4:
		if args == 1:
			#PWM change
			PWM_select += 1
			if PWM_select == 4:
				PWM_select = 0
		elif args == 2:
			#PWM setting mode on
			PWM_set = 1
		elif args == 3:
			#to switch
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
	global PWM_0, PWM_1, PWM_2, PWM_3, setting_select, rotary_result
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
	elif modeStates == 4:
		if PWM_set == 0:
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
		elif PWM_set == 1:
			#time_input
			pass
		elif PWM_set == 2:
			#steps_input
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


class Output_ctrl(threading.Thread):
	def __init__(self, *args, **kwargs):
		super(Output_ctrl, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.set()
		self.__running = threading.Event()
		self.__running.set()

	def run(self):
		global btn_input
		while self.__running.isSet():
			self.__flag.wait()
			button_input_change(btn_input)
			rotary_input_change()
			LCD_change()

			if modeStates == 1:
				rotary_encoder_Mode()
			elif modeStates == 2:
				while modeStates == 2:
					rotary_input_change()
					button_input_change(btn_input)
					LCD_change()
					movement_Mode()
				LCD_change()

			if btn_input == 1:
				print('>>>1<<<')
			elif btn_input == 2:
				print('>>>2<<<')
			elif btn_input == 3:
				print('>>>3<<<')
			btn_input = 0

			print('PWM_select:%d'%PWM_select)
			print('MOD_select:%d'%modeStates)
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
			while not GPIO.input(btn):
				btn_press_time += 1
				if btn_press_time > 5:
					btn_press = 2
					while not GPIO.input(btn):
						btn_press_time += 1
						if btn_press_time > 15:
							btn_press = 3
							while not GPIO.input(btn):
								time.sleep(0.2)
								pass
						time.sleep(0.2)
						pass
				time.sleep(0.2)
				pass
			btn_input = btn_press
			output_move.resume()
			time.sleep(0.2)
		time.sleep(0.2)



btn_threading=threading.Thread(target=button_Thread) #Define a thread for LCD
btn_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
btn_threading.start()									 #Thread starts



#>>> Main_loop Rotary  <<<
while 1:
	rotary_result = rotary_input(0.015)
	if rotary_result == 1:
		output_move.resume()
	elif rotary_result == -1:
		output_move.resume()
	pass
