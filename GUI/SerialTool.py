#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, locale
from PyQt4.QtCore import Qt, QTextCodec, SIGNAL, SLOT, QThread, pyqtSignal
from PyQt4.QtGui import QWidget, QLabel, QLineEdit, QComboBox, QPushButton, QCheckBox, QDialog, QApplication, QSlider, QVBoxLayout, QHBoxLayout, QMessageBox, QLCDNumber, QColor, QPalette, QIntValidator
import platform, struct, serial, binascii
import logging, time

logging.basicConfig(level = logging.DEBUG,
					format = '%(asctime)s %(filename)s[line:%(lineno)d] %(levelname)s %(message)s',
					datefmt = '%a, %d %b %Y %H:%M:%S')

if platform.system() == "Windows":
	from  serial.tools import list_ports
elif platform.system() == "Linux":
	import glob, os, re

code = QTextCodec.codecForName(locale.getpreferredencoding())
QTextCodec.setCodecForLocale(code)									# 设置程序能够正确读取到的本地文件的编码方式
QTextCodec.setCodecForTr(code)										# 使用设定的code编码来解析字符串方法
QTextCodec.setCodecForCStrings(code)
color_r = (0 , 3 , 8 , 15, 24, 35, 48, 63, 80, 99, 120, 143, 168, 195, 224, 255)
color_g = (0 , 3 , 8 , 15, 24, 35, 48, 63, 80, 99, 120, 143, 168, 195, 224, 255)
color_b = (0 , 3 , 8 , 15, 24, 35, 48, 63, 80, 99, 120, 143, 168, 195, 224, 255)
color_a = (0 , 3 , 8 , 15, 24, 35, 48, 63, 80, 99, 120, 143, 168, 195, 224, 255)
#		   0   1   2   3   4   5   6   7   8   9   10   11   12   13   14   15
HEADER = "55aa"
ENDER = "0d0a"

SIMPLE_INFO = 23130
WITH_SONAR_INFO = 23125
WITH_SIX_AXIS_SENSOR_INFO = 23210
ALL_INFO = 23205
SPEED_INFO = 42330
BATTERY_INFO = 42325
SONAR_INFO = 42410
SIX_AXIS_SENSOR_INFO = 42405

REQUIRE_SIMPLE_INFO = 21930
REQUIRE_SONAR_INFO = 21845
REQUIRE_SIX_AXIS_SENSOR_INFO = 21925
REQUIRE_ALL_INFO = 21850
REQUIRE_SPEED_INFO = 43690
REQUIRE_BATTERY_INFO = 43605
REQUIRE_ONLY_SONAR_INFO = 43685
REQUIRE_ONLY_SIX_AXIS_SENSOR_INFO = 43610

_ID = None

class ThreadingFindAllSerial(QThread):
	"""docstring for ThreadingFindAllSerial"""
	find = pyqtSignal()
	def __init__(self,parent): 
		super(ThreadingFindAllSerial, self).__init__(parent) 
		self.working = True
		self.parent = parent
		self.find.connect(self.changeQComboBox)

	def __del__(self): 
		self.working = False 
		self.wait()

	def findAllSerial(self):
		'''
		获取到串口列表
		'''
		if platform.system() == "Windows":
			try:
				self.parent.temp_serial = list()
				for com in list_ports.comports():
					strCom = com[0] + ": " + com[1][:-7]
					self.parent.temp_serial.append(strCom)
				self.parent.list_box_serial = self.parent.temp_serial
				self.find.emit()
			except Exception as e:
				logging.error(e)
				self.parent.logerror()
		elif platform.system() == "Linux":
			try:
				self.parent.temp_serial = list()
				self.parent.temp_serial = self.parent.findUSBtty()
				self.parent.list_box_serial = self.parent.temp_serial
				self.find.emit()
			except Exception as e:
				logging.error(e)
				self.parent.logerror()

	def changeQComboBox(self):
		if len(self.parent.list_box_serial) <= 0:
			self.parent.serial.clear()
		else:
			for i in range(0, self.parent.serial.count()):
				if self.parent.serial.itemText(i) not in self.parent.list_box_serial:
					self.parent.serial.removeItem(i)
			for item in self.parent.list_box_serial:
				if self.parent.serial.findText(item) == -1:
					self.parent.serial.addItem(item)

	def run(self): 
		while self.working == True: 
			self.findAllSerial()

class ThreadingSerialRead(QThread):
	"""docstring for ThreadingSerialRead"""
	def __init__(self,parent): 
		super(ThreadingSerialRead, self).__init__(parent) 
		self.working = True
		self.parent = parent

	def __del__(self): 
		self.working = False 
		self.wait()

	def SerialRead(self):
		'''
		线程读取串口发送的数据
		'''
		leftVel = 0.0
		rightVel = 0.0
		batteryVoltage = 0.0
		batteryPercentage = 0.0
		pitch = 0.0
		roll = 0.0
		yaw = 0.0
		temprature = 0.0
		sonar_front = 0.0
		sonar_back = 0.0
		sonar_left = 0.0
		sonar_right = 0.0
		while self.working:
			try:
				n = self.parent.Serial.l_serial.inWaiting()
				if n:
					temp = []
					self.parent.receive_data = binascii.b2a_hex(self.parent.Serial.l_serial.read(200))
					header = self.parent.receive_data[0:4]
					if header.lower() == HEADER:
						command = int(self.parent.receive_data[6:8] + self.parent.receive_data[4:6], 16)
						length = int(self.parent.receive_data[8:10], 16)
						for i in range(0, length * 2 + 10, 2):
							temp += [int(self.parent.receive_data[i: i + 2], 16)]
						ender = self.parent.receive_data[(10 + length * 2 + 2) : (10 + length * 2 + 6)]
						_ID = self.parent.receive_data[(10 + length * 2 + 6) : (10 + length * 2 + 30)]
						# print(_ID)
						if ender.lower() == ENDER:
							crc = self.parent.tohex(self.parent.GetCRC(temp, len(temp)), 8)[6:]
							if int(crc, 16) == int(self.parent.receive_data[(10 + length * 2) : (10 + length * 2 + 2)], 16):
								if command == SIMPLE_INFO:
									leftVelHex = self.parent.receive_data[16:18] + self.parent.receive_data[14:16] + self.parent.receive_data[12:14] + self.parent.receive_data[10:12]
									rightVelHex = self.parent.receive_data[24:26] + self.parent.receive_data[22:24] + self.parent.receive_data[20:22] + self.parent.receive_data[18:20]
									batteryVoltageHex = self.parent.receive_data[32:34] + self.parent.receive_data[30:32] + self.parent.receive_data[28:30] + self.parent.receive_data[26:28]
									batteryPercentageHex = self.parent.receive_data[40:42] + self.parent.receive_data[38:40] + self.parent.receive_data[36:38] + self.parent.receive_data[34:36]
									leftVel = struct.unpack('!f', leftVelHex.decode('hex'))[0]
									rightVel = struct.unpack('!f', rightVelHex.decode('hex'))[0]
									batteryVoltage = struct.unpack('!f', batteryVoltageHex.decode('hex'))[0]
									batteryPercentage = struct.unpack('!f', batteryPercentageHex.decode('hex'))[0]
								elif command == WITH_SONAR_INFO:
									leftVelHex = self.parent.receive_data[16:18] + self.parent.receive_data[14:16] + self.parent.receive_data[12:14] + self.parent.receive_data[10:12]
									rightVelHex = self.parent.receive_data[24:26] + self.parent.receive_data[22:24] + self.parent.receive_data[20:22] + self.parent.receive_data[18:20]
									batteryVoltageHex = self.parent.receive_data[32:34] + self.parent.receive_data[30:32] + self.parent.receive_data[28:30] + self.parent.receive_data[26:28]
									batteryPercentageHex = self.parent.receive_data[40:42] + self.parent.receive_data[38:40] + self.parent.receive_data[36:38] + self.parent.receive_data[34:36]
									sonar_frontHex = self.parent.receive_data[48:50] + self.parent.receive_data[46:48] + self.parent.receive_data[44:46] + self.parent.receive_data[42:44]
									sonar_backHex = self.parent.receive_data[56:58] + self.parent.receive_data[54:56] + self.parent.receive_data[52:54] + self.parent.receive_data[50:52]
									sonar_leftHex = self.parent.receive_data[64:66] + self.parent.receive_data[62:64] + self.parent.receive_data[60:62] + self.parent.receive_data[58:60]
									sonar_rightHex = self.parent.receive_data[72:74] + self.parent.receive_data[70:72] + self.parent.receive_data[68:70] + self.parent.receive_data[66:68]
									leftVel = struct.unpack('!f', leftVelHex.decode('hex'))[0]
									rightVel = struct.unpack('!f', rightVelHex.decode('hex'))[0]
									batteryVoltage = struct.unpack('!f', batteryVoltageHex.decode('hex'))[0]
									batteryPercentage = struct.unpack('!f', batteryPercentageHex.decode('hex'))[0]
									sonar_front = struct.unpack('!f', sonar_frontHex.decode('hex'))[0]
									sonar_back = struct.unpack('!f', sonar_backHex.decode('hex'))[0]
									sonar_left = struct.unpack('!f', sonar_leftHex.decode('hex'))[0]
									sonar_right = struct.unpack('!f', sonar_rightHex.decode('hex'))[0]
								elif command == WITH_SIX_AXIS_SENSOR_INFO:
									leftVelHex = self.parent.receive_data[16:18] + self.parent.receive_data[14:16] + self.parent.receive_data[12:14] + self.parent.receive_data[10:12]
									rightVelHex = self.parent.receive_data[24:26] + self.parent.receive_data[22:24] + self.parent.receive_data[20:22] + self.parent.receive_data[18:20]
									batteryVoltageHex = self.parent.receive_data[32:34] + self.parent.receive_data[30:32] + self.parent.receive_data[28:30] + self.parent.receive_data[26:28]
									batteryPercentageHex = self.parent.receive_data[40:42] + self.parent.receive_data[38:40] + self.parent.receive_data[36:38] + self.parent.receive_data[34:36]
									pitchHex = self.parent.receive_data[48:50] + self.parent.receive_data[46:48] + self.parent.receive_data[44:46] + self.parent.receive_data[42:44]
									rollHex = self.parent.receive_data[56:58] + self.parent.receive_data[54:56] + self.parent.receive_data[52:54] + self.parent.receive_data[50:52]
									yawHex = self.parent.receive_data[64:66] + self.parent.receive_data[62:64] + self.parent.receive_data[60:62] + self.parent.receive_data[58:60]
									tempratureHex = self.parent.receive_data[72:74] + self.parent.receive_data[70:72] + self.parent.receive_data[68:70] + self.parent.receive_data[66:68]
									leftVel = struct.unpack('!f', leftVelHex.decode('hex'))[0]
									rightVel = struct.unpack('!f', rightVelHex.decode('hex'))[0]
									batteryVoltage = struct.unpack('!f', batteryVoltageHex.decode('hex'))[0]
									batteryPercentage = struct.unpack('!f', batteryPercentageHex.decode('hex'))[0]
									pitch = struct.unpack('!f', pitchHex.decode('hex'))[0]
									roll = struct.unpack('!f', rollHex.decode('hex'))[0]
									yaw = struct.unpack('!f', yawHex.decode('hex'))[0]
									temprature = struct.unpack('!f', tempratureHex.decode('hex'))[0]
								elif command == ALL_INFO:
									leftVelHex = self.parent.receive_data[16:18] + self.parent.receive_data[14:16] + self.parent.receive_data[12:14] + self.parent.receive_data[10:12]
									rightVelHex = self.parent.receive_data[24:26] + self.parent.receive_data[22:24] + self.parent.receive_data[20:22] + self.parent.receive_data[18:20]
									batteryVoltageHex = self.parent.receive_data[32:34] + self.parent.receive_data[30:32] + self.parent.receive_data[28:30] + self.parent.receive_data[26:28]
									batteryPercentageHex = self.parent.receive_data[40:42] + self.parent.receive_data[38:40] + self.parent.receive_data[36:38] + self.parent.receive_data[34:36]
									pitchHex = self.parent.receive_data[48:50] + self.parent.receive_data[46:48] + self.parent.receive_data[44:46] + self.parent.receive_data[42:44]
									rollHex = self.parent.receive_data[56:58] + self.parent.receive_data[54:56] + self.parent.receive_data[52:54] + self.parent.receive_data[50:52]
									yawHex = self.parent.receive_data[64:66] + self.parent.receive_data[62:64] + self.parent.receive_data[60:62] + self.parent.receive_data[58:60]
									tempratureHex = self.parent.receive_data[72:74] + self.parent.receive_data[70:72] + self.parent.receive_data[68:70] + self.parent.receive_data[66:68]
									sonar_frontHex = self.parent.receive_data[80:82] + self.parent.receive_data[78:80] + self.parent.receive_data[76:78] + self.parent.receive_data[74:76]
									sonar_backHex = self.parent.receive_data[88:90] + self.parent.receive_data[86:88] + self.parent.receive_data[84:86] + self.parent.receive_data[82:84]
									sonar_leftHex = self.parent.receive_data[96:98] + self.parent.receive_data[94:96] + self.parent.receive_data[92:94] + self.parent.receive_data[90:92]
									sonar_rightHex = self.parent.receive_data[104:106] + self.parent.receive_data[102:104] + self.parent.receive_data[100:102] + self.parent.receive_data[98:100]
									leftVel = struct.unpack('!f', leftVelHex.decode('hex'))[0]
									rightVel = struct.unpack('!f', rightVelHex.decode('hex'))[0]
									batteryVoltage = struct.unpack('!f', batteryVoltageHex.decode('hex'))[0]
									batteryPercentage = struct.unpack('!f', batteryPercentageHex.decode('hex'))[0]
									pitch = struct.unpack('!f', pitchHex.decode('hex'))[0]
									roll = struct.unpack('!f', rollHex.decode('hex'))[0]
									yaw = struct.unpack('!f', yawHex.decode('hex'))[0]
									temprature = struct.unpack('!f', tempratureHex.decode('hex'))[0]
									sonar_front = struct.unpack('!f', sonar_frontHex.decode('hex'))[0]
									sonar_back = struct.unpack('!f', sonar_backHex.decode('hex'))[0]
									sonar_left = struct.unpack('!f', sonar_leftHex.decode('hex'))[0]
									sonar_right = struct.unpack('!f', sonar_rightHex.decode('hex'))[0]
								elif command == SPEED_INFO:
									leftVelHex = self.parent.receive_data[16:18] + self.parent.receive_data[14:16] + self.parent.receive_data[12:14] + self.parent.receive_data[10:12]
									rightVelHex = self.parent.receive_data[24:26] + self.parent.receive_data[22:24] + self.parent.receive_data[20:22] + self.parent.receive_data[18:20]
									leftVel = struct.unpack('!f', leftVelHex.decode('hex'))[0]
									rightVel = struct.unpack('!f', rightVelHex.decode('hex'))[0]
								elif command == BATTERY_INFO:
									batteryVoltageHex = self.parent.receive_data[16:18] + self.parent.receive_data[14:16] + self.parent.receive_data[12:14] + self.parent.receive_data[10:12]
									batteryPercentageHex = self.parent.receive_data[24:26] + self.parent.receive_data[22:24] + self.parent.receive_data[20:22] + self.parent.receive_data[18:20]
									batteryVoltage = struct.unpack('!f', batteryVoltageHex.decode('hex'))[0]
									batteryPercentage = struct.unpack('!f', batteryPercentageHex.decode('hex'))[0]
								elif command == SONAR_INFO:
									sonar_frontHex = self.parent.receive_data[16:18] + self.parent.receive_data[14:16] + self.parent.receive_data[12:14] + self.parent.receive_data[10:12]
									sonar_backHex = self.parent.receive_data[24:26] + self.parent.receive_data[22:24] + self.parent.receive_data[20:22] + self.parent.receive_data[18:20]
									sonar_leftHex = self.parent.receive_data[32:34] + self.parent.receive_data[30:32] + self.parent.receive_data[28:30] + self.parent.receive_data[26:28]
									sonar_rightHex = self.parent.receive_data[40:42] + self.parent.receive_data[38:40] + self.parent.receive_data[36:38] + self.parent.receive_data[34:36]
									sonar_front = struct.unpack('!f', sonar_frontHex.decode('hex'))[0]
									sonar_back = struct.unpack('!f', sonar_backHex.decode('hex'))[0]
									sonar_left = struct.unpack('!f', sonar_leftHex.decode('hex'))[0]
									sonar_right = struct.unpack('!f', sonar_rightHex.decode('hex'))[0]
								elif command == SIX_AXIS_SENSOR_INFO:
									pitchHex = self.parent.receive_data[16:18] + self.parent.receive_data[14:16] + self.parent.receive_data[12:14] + self.parent.receive_data[10:12]
									rollHex = self.parent.receive_data[24:26] + self.parent.receive_data[22:24] + self.parent.receive_data[20:22] + self.parent.receive_data[18:20]
									yawHex = self.parent.receive_data[32:34] + self.parent.receive_data[30:32] + self.parent.receive_data[28:30] + self.parent.receive_data[26:28]
									tempratureHex = self.parent.receive_data[40:42] + self.parent.receive_data[38:40] + self.parent.receive_data[36:38] + self.parent.receive_data[34:36]
									pitch = struct.unpack('!f', pitchHex.decode('hex'))[0]
									roll = struct.unpack('!f', rollHex.decode('hex'))[0]
									yaw = struct.unpack('!f', yawHex.decode('hex'))[0]
									temprature = struct.unpack('!f', tempratureHex.decode('hex'))[0]

					self.parent.leftVel.display("%.2f" %leftVel)
					self.parent.rightVel.display("%.2f" %rightVel)
					self.parent.batteryVoltage.display("%.2f" %batteryVoltage)
					self.parent.batteryPercentage.display("%.2f" %batteryPercentage)
					self.parent.pitch.display("%.2f" %pitch)
					self.parent.roll.display("%.2f" %roll)
					self.parent.yaw.display("%.2f" %yaw)
					self.parent.temprature.display("%.2f" %temprature)
					self.parent.sonarFront.display("%.2f" %sonar_front)
					self.parent.sonarBack.display("%.2f" %sonar_back)
					self.parent.sonarLeft.display("%.2f" %sonar_left)
					self.parent.sonarRight.display("%.2f" %sonar_right)

					self.parent.receive_data = ""

			except Exception as e:
				logging.error(e)
				self.parent.receive_data = ""
				self.parent.Serial.stop()
				self.parent.Serial = None

	def run(self):
		if self.working == False:
			self.working = True
		while self.working == True: 
			self.SerialRead()

	def stop(self):
		self.working = False

class ThreadingSerialSend(QThread):
	"""docstring for ThreadingSerialSend"""
	def __init__(self,parent): 
		super(ThreadingSerialSend, self).__init__(parent) 
		self.working = True
		self.parent = parent

	def __del__(self): 
		self.working = False 
		self.wait()

	def SerialSend(self):
		while self.parent.autoSend.isChecked() and self.working:
			temp = []
			left = self.parent.tohex(self.parent.leftVelValue, 32)
			right = self.parent.tohex(self.parent.rightVelValue, 32)
			command = self.parent.tohex(REQUIRE_ALL_INFO, 32)[-4:]
			length = 8
			temp = [int(HEADER[0:2], 16)] + [int(HEADER[2:], 16)] + [int(command[2:], 16)] + [int(command[0:2], 16)] + [length] + [int(right[6:], 16)] + [int(right[4:6], 16)] + [int(right[2:4], 16)] + [int(right[0:2], 16)] + [int(left[6:], 16)] + [int(left[4:6], 16)] + [int(left[2:4], 16)] + [int(left[0:2], 16)]
			data = HEADER + command[2:] + command[0:2] + self.parent.tohex(length, 32)[-2:] + right[6:] + right[4:6] + right[2:4] + right[0:2] + left[6:] + left[4:6] + left[2:4] + left[0:2] + self.parent.tohex(self.parent.GetCRC(temp, len(temp)), 8)[6:] + ENDER
			try:
				self.parent.Serial.write(data, isHex = True)
				time.sleep(0.25)
			except Exception as e:
				logging.error(e)
		else:
			pass

	def run(self): 
		if self.working == False:
			self.working = True
		while self.working == True: 
			self.SerialSend()

	def stop(self):
		self.working = False

class Serial(object):
	def __init__(self, Port = "COM6", BaudRate = "9600", ByteSize = "8", Parity = "N", Stopbits = "1"):
		'''
		初始化一些参数
		'''
		self.l_serial = None
		self.alive = False
		self.port = Port
		self.baudrate = BaudRate
		self.bytesize = ByteSize
		self.parity = Parity
		self.stopbits = Stopbits
		self.thresholdValue = 64
		self.receive_data = ""

	def start(self):
		'''
		开始，打开串口
		'''
		self.l_serial = serial.Serial()
		self.l_serial.port = self.port
		self.l_serial.baudrate = self.baudrate
		self.l_serial.bytesize = int(self.bytesize)
		self.l_serial.parity = self.parity
		self.l_serial.stopbits = float(self.stopbits)
		self.l_serial.timeout = 2
		
		try:
			self.l_serial.open()
			if self.l_serial.isOpen():
				self.alive = True
		except Exception as e:
			self.alive = False
			logging.error(e)

	def stop(self):
		'''
		结束，关闭串口
		'''
		self.alive = False
		if self.l_serial.isOpen():
			self.l_serial.close()

	def read(self):
		'''
		循环读取串口发送的数据
		'''
		receive_data = ""
		while self.alive:
			try:
				number = self.l_serial.inWaiting()
				if number:
					receive_data += self.l_serial.read(number)
				return receive_data
			except Exception as e:
				logging.error(e)

	def write(self, data, isHex = False):
		'''
		发送数据给串口设备
		'''
		if self.alive:
			if self.l_serial.isOpen():
				if isHex:
					# data = data.replace(" ", "").replace("\n", "")
					data = binascii.unhexlify(data)
				self.l_serial.write(data)

class MainWindow( QWidget ):
	"""docstring for MainWindow"""
	def __init__(self):
		super(MainWindow, self).__init__()
		self.master = None
		self.tab = QMainSerial()

		self.setFixedSize(720, 480)
		self.setWindowTitle(u'MRobot串口调试工具')
		self.setWindowModality(Qt.ApplicationModal)

		self.init()
		self.show()

	def closeEvent(self, event):
		if self.tab.Serial != None:
			if self.tab.Serial.alive:
				try:
					self.tab.Serial.stop()
				except Exception as e:
					logging.error(e)
		if self.tab.thread_read != None:
			try:
				self.tab.SerialReadSignal.emit()
			except Exception as e:
				logging.error(e)
		if self.tab.thread_send != None:
			try:
				self.tab.SerialSendSignal.emit()
			except Exception as e:
				logging.error(e)
		if self.tab.thread_findserial != None:
			try:
				self.tab.thread_findserial.exit()
			except Exception as e:
				logging.error(e)
	
	def init(self):

		self.mainLayout = QVBoxLayout()
		self.mainLayout.addWidget(self.tab)

		self.setLayout(self.mainLayout)

class QMainSerial(QDialog):
	"""docstring for QMainSerial"""
	SerialSendSignal = pyqtSignal()
	SerialReadSignal = pyqtSignal()
	def __init__(self):
		super(QMainSerial, self).__init__()
		self.Serial = None
		self.thread_read = None
		self.thread_send = None
		self.thread_findserial = None
		self.receive_data = ""
		self.leftVelValue = 0
		self.rightVelValue = 0
		self.leftVelLabel = QLabel(u'左轮速度：', self)
		self.rightVelLabel = QLabel(u'右轮速度：', self)
		self.batteryVoltageLabel = QLabel(u'电池电压：', self)
		self.batteryPercentageLabel = QLabel(u'电池电量：', self)
		self.pitchLabel = QLabel(u'翻 滚 角  ：', self)
		self.rollLabel = QLabel(u'俯 仰 角  ：', self)
		self.yawLabel = QLabel(u'航 向 角  ：', self)
		self.tempratureLabel = QLabel(u'温    度    ：', self)
		self.sonarFrontLabel = QLabel(u'前超声波：', self)
		self.sonarBackLabel = QLabel(u'后超声波：', self)
		self.sonarLeftLabel = QLabel(u'左超声波：', self)
		self.sonarRightLabel = QLabel(u'右超声波：', self)
		self.leftVel = QLCDNumber(self)
		self.leftVel.setNumDigits(8)
		self.leftVel.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.leftVel.setFocusPolicy(Qt.NoFocus)
		self.rightVel = QLCDNumber(self)
		self.rightVel.setNumDigits(8)
		self.rightVel.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.rightVel.setFocusPolicy(Qt.NoFocus)
		self.batteryVoltage = QLCDNumber(self)
		self.batteryVoltage.setNumDigits(8)
		self.batteryVoltage.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.batteryVoltage.setFocusPolicy(Qt.NoFocus)
		self.batteryPercentage = QLCDNumber(self)
		self.batteryPercentage.setNumDigits(8)
		self.batteryPercentage.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.batteryPercentage.setFocusPolicy(Qt.NoFocus)
		self.pitch = QLCDNumber(self)
		self.pitch.setNumDigits(8)
		self.pitch.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.pitch.setFocusPolicy(Qt.NoFocus)
		self.roll = QLCDNumber(self)
		self.roll.setNumDigits(8)
		self.roll.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.roll.setFocusPolicy(Qt.NoFocus)
		self.yaw = QLCDNumber(self)
		self.yaw.setNumDigits(8)
		self.yaw.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.yaw.setFocusPolicy(Qt.NoFocus)
		self.temprature = QLCDNumber(self)
		self.temprature.setNumDigits(8)
		self.temprature.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.temprature.setFocusPolicy(Qt.NoFocus)

		self.sonarFront = QLCDNumber(self)
		self.sonarFront.setNumDigits(8)
		self.sonarFront.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.sonarFront.setFocusPolicy(Qt.NoFocus)
		self.sonarBack = QLCDNumber(self)
		self.sonarBack.setNumDigits(8)
		self.sonarBack.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.sonarBack.setFocusPolicy(Qt.NoFocus)
		self.sonarLeft = QLCDNumber(self)
		self.sonarLeft.setNumDigits(8)
		self.sonarLeft.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.sonarLeft.setFocusPolicy(Qt.NoFocus)
		self.sonarRight = QLCDNumber(self)
		self.sonarRight.setNumDigits(8)
		self.sonarRight.setPalette(QPalette(QColor(color_r[9], color_g[13], color_b[8], color_a[14])))
		self.sonarRight.setFocusPolicy(Qt.NoFocus)

		self.leftVelUnit = QLabel(u'mm/s', self)
		self.rightVelUnit = QLabel(u'mm/s', self)
		self.batteryVoltageUnit = QLabel(u'mV', self)
		self.batteryPercentageUnit = QLabel(u'%', self)
		self.pitchUnit = QLabel('°', self)
		self.rollUnit = QLabel('°', self)
		self.yawUnit = QLabel('°', self)
		self.tempratureUnit = QLabel('°C', self)
		self.sonarFrontUnit = QLabel(u'm', self)
		self.sonarBackUnit = QLabel(u'm', self)
		self.sonarLeftUnit = QLabel(u'm', self)
		self.sonarRightUnit = QLabel(u'm', self)
		self.leftVel.display("%.2f" %0.0)
		self.rightVel.display("%.2f" %0.0)
		self.batteryVoltage.display("%.2f" %0.0)
		self.batteryPercentage.display("%.2f" %0.0)
		self.sonarFront.display("%.2f" %0.0)
		self.sonarBack.display("%.2f" %0.0)
		self.sonarLeft.display("%.2f" %0.0)
		self.sonarRight.display("%.2f" %0.0)
		self.pitch.display("%.2f" %0.0)
		self.roll.display("%.2f" %0.0)
		self.yaw.display("%.2f" %0.0)
		self.temprature.display("%.2f" %0.0)

		self.leftVelSendLabel = QLabel(u'左轮速度：', self)
		self.rightVelSendLabel = QLabel(u'右轮速度：', self)
		self.leftVelSet = QLineEdit(self)
		self.leftVelSet.setText(str(0))
		self.leftVelSet.setValidator(QIntValidator(-300, 300))
		self.rightVelSet = QLineEdit(self)
		self.rightVelSet.setText(str(0))
		self.rightVelSet.setValidator(QIntValidator(-300, 300))
		self.leftVelSend = QSlider(Qt.Horizontal, self)
		self.leftVelSend.setTickPosition(QSlider.TicksBelow)
		self.leftVelSend.setTickInterval(25)
		self.leftVelSend.setFocusPolicy(Qt.NoFocus)
		self.leftVelSend.setRange(-300, 300)
		self.rightVelSend = QSlider(Qt.Horizontal, self)
		self.rightVelSend.setTickPosition(QSlider.TicksBelow)
		self.rightVelSend.setTickInterval(25)
		self.rightVelSend.setFocusPolicy(Qt.NoFocus)
		self.rightVelSend.setRange(-300, 300)
		self.sendButton = QPushButton(u'发送', self)
		self.autoSend = QCheckBox(u'自动发送', self)

		self.leftVelSend.valueChanged[int].connect(self.changeLeftValue)
		self.rightVelSend.valueChanged[int].connect(self.changeRightValue)
		self.leftVelSet.textChanged[str].connect(self.changeLeftValue)
		self.rightVelSet.textChanged[str].connect(self.changeRightValue)

		self.serialLabel = QLabel(u'串口选择：', self)
		self.serial = QComboBox(self)
		self.list_box_serial = list()
		self.findAllSerial()
		self.autoSend.setEnabled(False)
		if len(self.list_box_serial) <= 0:
			self.warning()
		else:
			# self.autoSend.setEnabled(True)
			for item in self.list_box_serial:
				self.serial.addItem(item)
		self.serialBoundRateLabel = QLabel(u'波特率：', self)
		self.serialBoundRate = QComboBox(self)
		self.serialBoundRate.addItem(u"自定义")
		self.serialBoundRate.addItem(u"1382400")
		self.serialBoundRate.addItem(u"921600")
		self.serialBoundRate.addItem(u"460800")
		self.serialBoundRate.addItem(u"256000")
		self.serialBoundRate.addItem(u"230400")
		self.serialBoundRate.addItem(u"128000")
		self.serialBoundRate.addItem(u"115200")
		self.serialBoundRate.addItem(u"76800")
		self.serialBoundRate.addItem(u"57600")
		self.serialBoundRate.addItem(u"43000")
		self.serialBoundRate.addItem(u"38400")
		self.serialBoundRate.addItem(u"19200")
		self.serialBoundRate.addItem(u"14400")
		self.serialBoundRate.addItem(u"9600")
		self.serialBoundRate.addItem(u"4800")
		self.serialBoundRate.addItem(u"2400")
		self.serialBoundRate.addItem(u"1200")
		self.serialBoundRate.setCurrentIndex(7)
		self.stopBitLabel = QLabel(u'停止位：', self)
		self.stopBit = QComboBox(self)
		self.stopBit.addItem(u"1")
		self.stopBit.addItem(u"1.5")
		self.stopBit.addItem(u"2")
		self.dataBitLabel = QLabel(u'数据位：', self)
		self.dataBit = QComboBox(self)
		self.dataBit.addItem(u"8")
		self.dataBit.addItem(u"7")
		self.dataBit.addItem(u"6")
		self.dataBit.addItem(u"5")
		self.checkBitLabel = QLabel(u'校验位：', self)
		self.checkBit = QComboBox(self)
		self.checkBit.addItem(u"None")
		self.checkBit.addItem(u"Odd")
		self.checkBit.addItem(u"Even")
		self.serialOperateLabel = QLabel(u'串口操作：', self)
		self.serialOperate = QPushButton(u'打开串口', self)
		self.threading = ThreadingSerialSend(self)
		self.SerialSendSignal.connect(self.threading.stop)

		self.connect(self.sendButton, SIGNAL("clicked()"), self.SerialSend)
		self.connect(self.serialOperate, SIGNAL("clicked()"), self.openSerial)
		self.autoSend.stateChanged.connect(self.ThreadingSend, Qt.QueuedConnection)
		self.__init()

	def changeLeftValue(self, value):
		try:
			self.leftVelValue = int(value)
			self.leftVelSet.setText(str(value))
			self.leftVelSend.setValue(int(value))
		except:
			pass

	def GetCRC(self, data, length):
		crc = 0
		for i in range(0, length):
			crc = crc ^ data[i]
			for x in range(0, 8):
				if (crc & 0x01):
					crc = (crc >> 1) ^ 0x8C
				else:
					crc = crc >> 1
		return crc

	def changeRightValue(self, value):
		try:
			self.rightVelValue = int(value)
			self.rightVelSet.setText(str(value))
			self.rightVelSend.setValue(int(value))
		except:
			pass

	def openSerial(self):
		if self.serialOperate.text() == u'打开串口':
			if len(self.list_box_serial) <= 0:
				self.autoSend.setEnabled(False)
			else:
				self.autoSend.setEnabled(True)
				if self.checkBit.currentIndex() == 0:
					checkBit = "N"
				elif self.checkBit.currentIndex() == 1:
					checkBit = "O"
				elif self.checkBit.currentIndex() == 2:
					checkBit = "E"
				try:
					self.Serial = Serial(str(self.serial.currentText()), self.serialBoundRate.currentText(), 
						self.dataBit.currentText(), checkBit, self.stopBit.currentText())
					self.serialOperate.setText(u'关闭串口')
					self.serialBoundRate.setEnabled(False)
					self.serial.setEnabled(False)
					self.stopBit.setEnabled(False)
					self.dataBit.setEnabled(False)
					self.checkBit.setEnabled(False)
					self.Serial.start()
					if self.Serial.alive:
						if self.threading == None:
							self.threading = ThreadingSerialSend(self)
							self.threading.start()
						self.thread_read = ThreadingSerialRead(self)
						self.thread_read.start()
						self.SerialReadSignal.connect(self.thread_read.stop)
				except Exception as e:
					logging.error(e)
		else:
			self.autoSend.setEnabled(False)
			self.serialOperate.setText(u'打开串口')
			self.serialBoundRate.setEnabled(True)
			self.serial.setEnabled(True)
			self.stopBit.setEnabled(True)
			self.dataBit.setEnabled(True)
			self.checkBit.setEnabled(True)
			try:
				self.autoSend.setCheckState(0)
				if self.threading != None:
					self.SerialSendSignal.emit()
					self.threading.exit()
				self.threading = None
				if self.thread_read != None:
					self.SerialReadSignal.emit()
					self.thread_read.exit()
				self.thread_read = None
				if self.Serial != None:
					self.Serial.stop()
				self.Serial = None
			except Exception as e:
				logging.error(e)

	def findAllSerial(self):
		'''
		获取到串口列表
		'''
		if platform.system() == "Windows":
			try:
				self.temp_serial = list()
				for com in list_ports.comports():
					strCom = com[0] + ": " + com[1][:-7]
					self.temp_serial.append(strCom)
				self.list_box_serial = self.temp_serial

				self.thread_findserial = ThreadingFindAllSerial(self)
				self.thread_findserial.start()
			except Exception as e:
				logging.error(e)
				self.logerror()
		elif platform.system() == "Linux":
			try:
				self.temp_serial = list()
				self.temp_serial = self.findUSBtty()
				self.list_box_serial = self.temp_serial

				self.thread_findserial = ThreadingFindAllSerial(self)
				self.thread_findserial.start()
			except Exception as e:
				logging.error(e)
				self.logerror()

	def findUSBtty(self, vendor_id = None, product_id = None):
		'''
		发现串口设备
		'''
		tty_devs = list()
		for dn in glob.glob('/sys/bus/usb/devices/*') :
			try:
				vid = int(open(os.path.join(dn, "idVendor" )).read().strip(), 16)
				pid = int(open(os.path.join(dn, "idProduct")).read().strip(), 16)
				if  ((vendor_id is None) or (vid == vendor_id)) and ((product_id is None) or (pid == product_id)) :
					dns = glob.glob(os.path.join(dn, os.path.basename(dn) + "*"))
					for sdn in dns :
						for fn in glob.glob(os.path.join(sdn, "*")) :
							if  re.search(r"\/ttyUSB[0-9]+$", fn) :
								tty_devs.append(os.path.join("/dev", os.path.basename(fn)))
			except Exception as ex:
				pass
		return tty_devs

	def tohex(self, val, nbits):
		res = hex((val + (1 << nbits)) % (1 << nbits))[2:]
		return '0' * (8 - len(res)) + res

	def ThreadingSend(self):
		if self.autoSend.isChecked():
			self.sendButton.setEnabled(False)
			self.serialOperate.setEnabled(False)
			if self.threading == None:
				self.threading = ThreadingSerialSend(self)
			self.threading.start()
		else:
			self.sendButton.setEnabled(True)
			self.serialOperate.setEnabled(True)
			self.SerialSendSignal.emit()

	def SerialSend(self):
		if self.Serial:
			temp = []
			left = self.tohex(self.leftVelValue, 32)
			right = self.tohex(self.rightVelValue, 32)
			command = self.tohex(REQUIRE_ALL_INFO, 32)[-4:]
			length = 8
			temp = [int(HEADER[0:2], 16)] + [int(HEADER[2:], 16)] + [int(command[2:], 16)] + [int(command[0:2], 16)] + [length] + [int(right[6:], 16)] + [int(right[4:6], 16)] + [int(right[2:4], 16)] + [int(right[0:2], 16)] + [int(left[6:], 16)] + [int(left[4:6], 16)] + [int(left[2:4], 16)] + [int(left[0:2], 16)]
			data = HEADER + command[2:] + command[0:2] + self.tohex(length, 32)[-2:] + right[6:] + right[4:6] + right[2:4] + right[0:2] + left[6:] + left[4:6] + left[2:4] + left[0:2] + self.tohex(self.GetCRC(temp, len(temp)), 8)[6:] + ENDER
			try:
				self.Serial.write(data, isHex = True)
				time.sleep(0.25)
			except Exception as e:
				logging.error(e)
		else:
			pass

	def warning(self):
		QMessageBox.critical(self, u"错误", self.tr(u"没有发现串口!"))

	def logerror(self):
		QMessageBox.critical(self, u"错误", self.tr(u"未知错误!"))
		
	def __init(self):
		self.leftLayout = QVBoxLayout()
		self.rightLayout = QVBoxLayout()
		self.topLayout = QVBoxLayout()
		self.bottomLayout = QHBoxLayout()
		self.onTopLayout = QHBoxLayout()
		self.onMiddleLayout = QHBoxLayout()
		self.onBottomLayout = QHBoxLayout()

		self.onLeftLayout = QVBoxLayout()
		self.onRightLayout = QVBoxLayout()
		self.onLeftTopLayout = QHBoxLayout()
		self.onLeftBottomLayout = QHBoxLayout()

		self.leftOnLayout = QVBoxLayout()
		self.rightOnLayout = QVBoxLayout()
		self.leftMiddleLayout = QVBoxLayout()
		self.rightMiddleLayout = QVBoxLayout()
		self.leftBottomLayout = QVBoxLayout()
		self.rightBottomLayout = QVBoxLayout()

		self.leftOnTopLayout = QHBoxLayout()
		self.leftOnBottomLayout = QHBoxLayout()
		self.rightOnTopLayout = QHBoxLayout()
		self.rightOnBottomLayout = QHBoxLayout()
		self.leftMiddleTopLayout = QHBoxLayout()
		self.leftMiddleBottomLayout = QHBoxLayout()
		self.rightMiddleTopLayout = QHBoxLayout()
		self.rightMiddleBottomLayout = QHBoxLayout()
		self.leftBottomTopLayout = QHBoxLayout()
		self.leftBottomBottomLayout = QHBoxLayout()
		self.rightBottomTopLayout = QHBoxLayout()
		self.rightBottomBottomLayout = QHBoxLayout()
		
		self.leftLayout.addLayout(self.topLayout)
		self.leftLayout.addLayout(self.bottomLayout)

		self.topLayout.addLayout(self.onTopLayout)
		self.topLayout.addLayout(self.onMiddleLayout)
		self.topLayout.addLayout(self.onBottomLayout)
		self.topLayout.addStretch()

		self.onTopLayout.addLayout(self.leftOnLayout)
		self.onTopLayout.addLayout(self.rightOnLayout)
		self.onMiddleLayout.addLayout(self.leftMiddleLayout)
		self.onMiddleLayout.addLayout(self.rightMiddleLayout)
		self.onBottomLayout.addLayout(self.leftBottomLayout)
		self.onBottomLayout.addLayout(self.rightBottomLayout)

		self.leftOnLayout.addLayout(self.leftOnTopLayout)
		self.leftOnLayout.addLayout(self.leftOnBottomLayout)

		self.rightOnLayout.addLayout(self.rightOnTopLayout)
		self.rightOnLayout.addLayout(self.rightOnBottomLayout)

		self.leftMiddleLayout.addLayout(self.leftMiddleTopLayout)
		self.leftMiddleLayout.addLayout(self.leftMiddleBottomLayout)

		self.rightMiddleLayout.addLayout(self.rightMiddleTopLayout)
		self.rightMiddleLayout.addLayout(self.rightMiddleBottomLayout)

		self.leftBottomLayout.addLayout(self.leftBottomTopLayout)
		self.leftBottomLayout.addLayout(self.leftBottomBottomLayout)

		self.rightBottomLayout.addLayout(self.rightBottomTopLayout)
		self.rightBottomLayout.addLayout(self.rightBottomBottomLayout)

		self.leftOnTopLayout.addWidget(self.leftVelLabel)
		self.leftOnTopLayout.addWidget(self.leftVel)
		self.leftOnTopLayout.addWidget(self.leftVelUnit)

		self.leftOnBottomLayout.addWidget(self.rightVelLabel)
		self.leftOnBottomLayout.addWidget(self.rightVel)
		self.leftOnBottomLayout.addWidget(self.rightVelUnit)

		self.rightOnTopLayout.addWidget(self.batteryVoltageLabel)
		self.rightOnTopLayout.addWidget(self.batteryVoltage)
		self.rightOnTopLayout.addWidget(self.batteryVoltageUnit)
		self.rightOnBottomLayout.addWidget(self.batteryPercentageLabel)
		self.rightOnBottomLayout.addWidget(self.batteryPercentage)
		self.rightOnBottomLayout.addWidget(self.batteryPercentageUnit)

		self.leftMiddleTopLayout.addWidget(self.pitchLabel)
		self.leftMiddleTopLayout.addWidget(self.pitch)
		self.leftMiddleTopLayout.addWidget(self.pitchUnit)
		self.leftMiddleBottomLayout.addWidget(self.rollLabel)
		self.leftMiddleBottomLayout.addWidget(self.roll)
		self.leftMiddleBottomLayout.addWidget(self.rollUnit)

		self.leftBottomTopLayout.addWidget(self.sonarFrontLabel)
		self.leftBottomTopLayout.addWidget(self.sonarFront)
		self.leftBottomTopLayout.addWidget(self.sonarFrontUnit)
		self.leftBottomBottomLayout.addWidget(self.sonarLeftLabel)
		self.leftBottomBottomLayout.addWidget(self.sonarLeft)
		self.leftBottomBottomLayout.addWidget(self.sonarLeftUnit)

		self.rightBottomTopLayout.addWidget(self.sonarBackLabel)
		self.rightBottomTopLayout.addWidget(self.sonarBack)
		self.rightBottomTopLayout.addWidget(self.sonarBackUnit)
		self.rightBottomBottomLayout.addWidget(self.sonarRightLabel)
		self.rightBottomBottomLayout.addWidget(self.sonarRight)
		self.rightBottomBottomLayout.addWidget(self.sonarRightUnit)

		self.rightMiddleTopLayout.addWidget(self.yawLabel)
		self.rightMiddleTopLayout.addWidget(self.yaw)
		self.rightMiddleTopLayout.addWidget(self.yawUnit)
		self.rightMiddleBottomLayout.addWidget(self.tempratureLabel)
		self.rightMiddleBottomLayout.addWidget(self.temprature)
		self.rightMiddleBottomLayout.addWidget(self.tempratureUnit)

		self.bottomLayout.addLayout(self.onLeftLayout)
		self.bottomLayout.addLayout(self.onRightLayout)
		self.onRightLayout.addWidget(self.sendButton)
		self.onRightLayout.addWidget(self.autoSend)
		self.onLeftLayout.addLayout(self.onLeftTopLayout)
		self.onLeftLayout.addLayout(self.onLeftBottomLayout)
		self.onLeftTopLayout.addWidget(self.leftVelSendLabel)
		self.onLeftTopLayout.addWidget(self.leftVelSend)
		self.onLeftTopLayout.addWidget(self.leftVelSet)
		self.onLeftBottomLayout.addWidget(self.rightVelSendLabel)
		self.onLeftBottomLayout.addWidget(self.rightVelSend)
		self.onLeftBottomLayout.addWidget(self.rightVelSet)

		self.rightLayout.addWidget(self.serialLabel)
		self.rightLayout.addWidget(self.serial)
		self.rightLayout.addWidget(self.serialBoundRateLabel)
		self.rightLayout.addWidget(self.serialBoundRate)
		self.rightLayout.addWidget(self.stopBitLabel)
		self.rightLayout.addWidget(self.stopBit)
		self.rightLayout.addWidget(self.dataBitLabel)
		self.rightLayout.addWidget(self.dataBit)
		self.rightLayout.addWidget(self.checkBitLabel)
		self.rightLayout.addWidget(self.checkBit)
		self.rightLayout.addWidget(self.serialOperateLabel)
		self.rightLayout.addWidget(self.serialOperate)

		self.mainLayout = QHBoxLayout()
		self.mainLayout.addLayout(self.leftLayout)
		self.mainLayout.addLayout(self.rightLayout)

		self.setLayout(self.mainLayout)

def main(*args):

	app = QApplication(sys.argv)
	MWindow = MainWindow()
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()
