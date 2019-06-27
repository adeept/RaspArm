#!/usr/bin/python
# -*- coding: UTF-8 -*-
# File name   : client.py
# Description : client  
# Website	 : www.adeept.com
# E-mail	  : support@adeept.com
# Author	  : William
# Date		: 2018/08/22
# 

from socket import *
import sys
import time
import threading as thread
import tkinter as tk

color_bg='#000000'		#Set background color
color_text='#E1F5FE'	  #Set text color
color_btn='#0277BD'	   #Set button color
color_line='#01579B'	  #Set line color
color_can='#212121'	   #Set canvas color
color_oval='#2196F3'	  #Set oval color
target_color='#FF6D00'
speed = 1
stat = 0
ip_stu=1

Switch_3 = 0
Switch_2 = 0
Switch_1 = 0


def replace_num(initial,new_num):   #Call this function to replace data in '.txt' file
	newline=""
	str_num=str(new_num)
	with open("ip.txt","r") as f:
		for line in f.readlines():
			if(line.find(initial) == 0):
				line = initial+"%s" %(str_num)
			newline += line
	with open("ip.txt","w") as f:
		f.writelines(newline)	#Call this function to replace data in '.txt' file


def num_import(initial):			#Call this function to import data from '.txt' file
	with open("ip.txt") as f:
		for line in f.readlines():
			if(line.find(initial) == 0):
				r=line
	begin=len(list(initial))
	snum=r[begin:]
	n=snum
	return n	


def connection_thread():
	global funcMode, Switch_3, Switch_2, Switch_1, SmoothMode
	while 1:
		car_info = (tcpClicSock.recv(BUFSIZ)).decode()
		if not car_info:
			continue


def Info_receive():
	global CPU_TEP,CPU_USE,RAM_USE
	HOST = ''
	INFO_PORT = 2256							#Define port serial 
	ADDR = (HOST, INFO_PORT)
	InfoSock = socket(AF_INET, SOCK_STREAM)
	InfoSock.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
	InfoSock.bind(ADDR)
	InfoSock.listen(5)					  #Start server,waiting for client
	InfoSock, addr = InfoSock.accept()
	print('Info connected')
	while 1:
		try:
			info_data = ''
			info_data = str(InfoSock.recv(BUFSIZ).decode())
			info_get = info_data.split()
			CPU_TEP,CPU_USE,RAM_USE= info_get
			#print('cpu_tem:%s\ncpu_use:%s\nram_use:%s'%(CPU_TEP,CPU_USE,RAM_USE))
			CPU_TEP_lab.config(text='CPU Temp: %s℃'%CPU_TEP)
			CPU_USE_lab.config(text='CPU Usage: %s'%CPU_USE)
			RAM_lab.config(text='RAM Usage: %s'%RAM_USE)
		except:
			pass


def socket_connect():	 #Call this function to connect with the server
	global ADDR,tcpClicSock,BUFSIZ,ip_stu,ipaddr
	ip_adr=E1.get()	   #Get the IP address from Entry

	if ip_adr == '':	  #If no input IP address in Entry,import a default IP
		ip_adr=num_import('IP:')
		l_ip_4.config(text='Connecting')
		l_ip_4.config(bg='#FF8F00')
		l_ip_5.config(text='Default:%s'%ip_adr)
		pass
	
	SERVER_IP = ip_adr
	SERVER_PORT = 10223   #Define port serial 
	BUFSIZ = 1024		 #Define buffer size
	ADDR = (SERVER_IP, SERVER_PORT)
	tcpClicSock = socket(AF_INET, SOCK_STREAM) #Set connection value for socket

	for i in range (1,6): #Try 5 times if disconnected
		#try:
		if ip_stu == 1:
			print("Connecting to server @ %s:%d..." %(SERVER_IP, SERVER_PORT))
			print("Connecting")
			tcpClicSock.connect(ADDR)		#Connection with the server
		
			print("Connected")
		
			l_ip_5.config(text='IP:%s'%ip_adr)
			l_ip_4.config(text='Connected')
			l_ip_4.config(bg='#558B2F')

			replace_num('IP:',ip_adr)
			E1.config(state='disabled')	  #Disable the Entry
			Btn14.config(state='disabled')   #Disable the Entry
			
			ip_stu=0						 #'0' means connected

			connection_threading=thread.Thread(target=connection_thread)		 #Define a thread for FPV and OpenCV
			connection_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
			connection_threading.start()									 #Thread starts

			info_threading=thread.Thread(target=Info_receive)		 #Define a thread for FPV and OpenCV
			info_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
			info_threading.start()									 #Thread starts

			video_threading=thread.Thread(target=opencv_r)		 #Define a thread for FPV and OpenCV
			video_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
			video_threading.start()									 #Thread starts

			break
		else:
			print("Cannot connecting to server,try it latter!")
			l_ip_4.config(text='Try %d/5 time(s)'%i)
			l_ip_4.config(bg='#EF6C00')
			print('Try %d/5 time(s)'%i)
			ip_stu=1
			time.sleep(1)
			continue

	if ip_stu == 1:
		l_ip_4.config(text='Disconnected')
		l_ip_4.config(bg='#F44336')


def connect(event):	   #Call this function to connect with the server
	if ip_stu == 1:
		sc=thread.Thread(target=socket_connect) #Define a thread for connection
		sc.setDaemon(True)					  #'True' means it is a front thread,it would close when the mainloop() closes
		sc.start()							  #Thread starts


def call_Switch_1(event):
	if Switch_1 == 0:
		tcpClicSock.send(('Switch_1_on').encode())
	else:
		tcpClicSock.send(('Switch_1_off').encode())


def call_Switch_2(event):
	if Switch_2 == 0:
		tcpClicSock.send(('Switch_2_on').encode())
	else:
		tcpClicSock.send(('Switch_2_off').encode())


def call_Switch_3(event):
	if Switch_3 == 0:
		tcpClicSock.send(('Switch_3_on').encode())
	else:
		tcpClicSock.send(('Switch_3_off').encode())


def set_B(event):
	time.sleep(0.03)
	tcpClicSock.send(('wsB %s'%var_B.get()).encode())


def pwm0_increase(event):
	tcpClicSock.send(('0+').encode())


def pwm1_increase(event):
	tcpClicSock.send(('1+').encode())


def pwm2_increase(event):
	tcpClicSock.send(('2+').encode())


def pwm3_increase(event):
	tcpClicSock.send(('3+').encode())


def pwm0_decrease(event):
	tcpClicSock.send(('0-').encode())


def pwm1_decrease(event):
	tcpClicSock.send(('1-').encode())


def pwm2_decrease(event):
	tcpClicSock.send(('2-').encode())


def pwm3_decrease(event):
	tcpClicSock.send(('3-').encode())


def pwm_buttons(x,y):
	Btn_0_increase = tk.Button(root, width=8, text='Port0 +',fg=color_text,bg=color_btn,relief='ridge')
	Btn_0_increase.place(x=x,y=y)
	Btn_0_increase.bind('<ButtonPress-1>', pwm0_increase)
	root.bind('<KeyPress-q>', pwm0_increase) 

	Btn_0_decrease = tk.Button(root, width=8, text='Port0 -',fg=color_text,bg=color_btn,relief='ridge')
	Btn_0_decrease.place(x=x,y=y+35)
	Btn_0_decrease.bind('<ButtonPress-1>', pwm0_decrease)
	root.bind('<KeyPress-a>', pwm0_decrease) 

	Btn_1_increase = tk.Button(root, width=8, text='Port1 +',fg=color_text,bg=color_btn,relief='ridge')
	Btn_1_increase.place(x=x+70,y=105)
	Btn_1_increase.bind('<ButtonPress-1>', pwm1_increase)
	root.bind('<KeyPress-w>', pwm1_increase) 

	Btn_1_decrease = tk.Button(root, width=8, text='Port1 -',fg=color_text,bg=color_btn,relief='ridge')
	Btn_1_decrease.place(x=x+70,y=y+35)
	Btn_1_decrease.bind('<ButtonPress-1>', pwm1_decrease)
	root.bind('<KeyPress-s>', pwm1_decrease) 

	Btn_2_increase = tk.Button(root, width=8, text='Port2 +',fg=color_text,bg=color_btn,relief='ridge')
	Btn_2_increase.place(x=x+140,y=105)
	Btn_2_increase.bind('<ButtonPress-1>', pwm2_increase)
	root.bind('<KeyPress-e>', pwm2_increase) 

	Btn_2_decrease = tk.Button(root, width=8, text='Port2 -',fg=color_text,bg=color_btn,relief='ridge')
	Btn_2_decrease.place(x=x+140,y=y+35)
	Btn_2_decrease.bind('<ButtonPress-1>', pwm2_decrease)
	root.bind('<KeyPress-d>', pwm2_decrease) 

	Btn_3_increase = tk.Button(root, width=8, text='Port3 +',fg=color_text,bg=color_btn,relief='ridge')
	Btn_3_increase.place(x=x+210,y=105)
	Btn_3_increase.bind('<ButtonPress-1>', pwm3_increase)
	root.bind('<KeyPress-r>', pwm3_increase) 

	Btn_3_decrease = tk.Button(root, width=8, text='Port3 -',fg=color_text,bg=color_btn,relief='ridge')
	Btn_3_decrease.place(x=x+210,y=y+35)
	Btn_3_decrease.bind('<ButtonPress-1>', pwm3_decrease)
	root.bind('<KeyPress-f>', pwm3_decrease) 


def information_screen(x,y):
	global CPU_TEP_lab, CPU_USE_lab, RAM_lab, l_ip_4, l_ip_5
	CPU_TEP_lab=tk.Label(root,width=18,text='CPU Temp:',fg=color_text,bg='#212121')
	CPU_TEP_lab.place(x=x,y=y)						 #Define a Label and put it in position

	CPU_USE_lab=tk.Label(root,width=18,text='CPU Usage:',fg=color_text,bg='#212121')
	CPU_USE_lab.place(x=x,y=y+30)						 #Define a Label and put it in position

	RAM_lab=tk.Label(root,width=18,text='RAM Usage:',fg=color_text,bg='#212121')
	RAM_lab.place(x=x,y=y+60)						 #Define a Label and put it in position

	l_ip_4=tk.Label(root,width=18,text='Disconnected',fg=color_text,bg='#F44336')
	l_ip_4.place(x=x,y=y+95)						 #Define a Label and put it in position

	l_ip_5=tk.Label(root,width=18,text='Use default IP',fg=color_text,bg=color_btn)
	l_ip_5.place(x=x,y=y+130)						 #Define a Label and put it in position


def connent_input(x,y):
	global E1, Btn14
	E1 = tk.Entry(root,show=None,width=16,bg="#37474F",fg='#eceff1')
	E1.place(x=x+5,y=y+25)							 #Define a Entry and put it in position

	l_ip_3=tk.Label(root,width=10,text='IP Address:',fg=color_text,bg='#000000')
	l_ip_3.place(x=x,y=y)						 #Define a Label and put it in position

	Btn14= tk.Button(root, width=8,height=2, text='Connect',fg=color_text,bg=color_btn,relief='ridge')
	Btn14.place(x=x+130,y=y)						  #Define a Button and put it in position

	root.bind('<Return>', connect)
	Btn14.bind('<ButtonPress-1>', connect)


def switch_button(x,y):
	global Btn_Switch_1, Btn_Switch_2, Btn_Switch_3
	Btn_Switch_1 = tk.Button(root, width=8, text='Port 1',fg=color_text,bg=color_btn,relief='ridge')
	Btn_Switch_2 = tk.Button(root, width=8, text='Port 2',fg=color_text,bg=color_btn,relief='ridge')
	Btn_Switch_3 = tk.Button(root, width=8, text='Port 3',fg=color_text,bg=color_btn,relief='ridge')

	Btn_Switch_1.place(x=x,y=y)
	Btn_Switch_2.place(x=x+70,y=y)
	Btn_Switch_3.place(x=x+140,y=y)

	Btn_Switch_1.bind('<ButtonPress-1>', call_Switch_1)
	Btn_Switch_2.bind('<ButtonPress-1>', call_Switch_2)
	Btn_Switch_3.bind('<ButtonPress-1>', call_Switch_3)


def scale(x,y,w):
	global var_B
	var_B = tk.StringVar()
	var_B.set(0)

	Scale_B = tk.Scale(root,label=None,
	from_=1,to=25,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=1,variable=var_B,troughcolor='#448AFF',command=set_B,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_B.place(x=x,y=y)							#Define a Scale and put it in position

	canvas_cover=tk.Canvas(root,bg=color_bg,height=30,width=510,highlightthickness=0)
	canvas_cover.place(x=x,y=y+30)


def loop():					  #GUI
	global root   #The value of tcpClicSock changes in the function loop(),would also changes in global so the other functions could use it.
	while True:
		root = tk.Tk()			#Define a window named root
		root.title('Adeept RaspArm')	  #Main window title
		root.geometry('495x256')  #Main window size, middle of the English letter x.
		root.config(bg=color_bg)  #Set the background color of root window

		try:
			logo =tk.PhotoImage(file = 'logo.png')		 #Define the picture of logo,but only supports '.png' and '.gif'
			l_logo=tk.Label(root,image = logo,bg=color_bg) #Set a label to show the logo picture
			l_logo.place(x=30,y=13)						#Place the Label in a right position
		except:
			pass

		pwm_buttons(30,105)

		information_screen(330,15)

		connent_input(110,29)

		switch_button(30,195)

		scale(245,190,215)

		global stat
		if stat==0:			  # Ensure the mainloop runs only once
			root.mainloop()  # Run the mainloop()
			stat=1		   # Change the value to '1' so the mainloop() would not run again.


if __name__ == '__main__':
	loop()