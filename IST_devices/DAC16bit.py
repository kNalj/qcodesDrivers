from qcodes.instrument.base import Instrument
import types
import logging
import numpy as np
import serial
import visa

import traceback
import threading
import time

from qcodes import VisaInstrument, validators as vals
from qcodes.instrument.parameter import ManualParameter
from qcodes.utils.validators import Bool, Numbers


Fullrange = 4000
Halfrange = Fullrange / 2

class IST_16(Instrument):

	def __init__(self, name, interface = 'COM9', reset=False, numdacs=16, dac_step=10,dac_delay=.1, safe_version=True,
				 polarity=['BIP', 'BIP', 'BIP', 'BIP'],
				 use_locks=False,**kwargs):
		t0 = time.time()
		super().__init__(name, **kwargs) 

		self._interface = interface

		self.Fullrange = Fullrange
		self.Halfrange = Halfrange
		self.communication_bytes = 3

		if numdacs % 4 == 0 and numdacs > 0:
			self._numdacs = int(numdacs)
		else:
			logging.error('Number of dacs needs to be multiple of 4')
		# initialize pol_num, the voltage offset due to the polarity
		self.pol_num = np.zeros(self._numdacs)
		for i in range(int(self._numdacs / 4)):
			self.set_pol_dacrack(polarity[i], np.arange(1 + i * 4, 1 + (i + 1) * 4),
								 get_all=False)

		


		# Add functions
		#self.add_function('get_all')
		#self.add_function('set_dacs_zero')
		#self.add_function('reinitialize_dacs')


		for i in range(1, numdacs + 1):
				self.add_parameter(
					'dac{}'.format(i),
					label='Dac {}'.format(i),
					unit='mV',
					get_cmd=self._gen_ch_get_func(self.do_get_dac, i),
					set_cmd=self._gen_ch_set_func(self.do_set_dac, i),
					vals=vals.Numbers(self.pol_num[i - 1]-1,
									self.pol_num[i - 1] + self.Fullrange+1),
					step=dac_step,
					delay=dac_delay,
					max_val_age=10)

		self._open_serial_connection()

	#open serial connection
	def _open_serial_connection(self):
		
		self.ser = serial.Serial()
		self.ser.port = self._interface
		self.ser.baudrate = 10000000
		self.ser.bytesize = serial.EIGHTBITS #number of bits per bytes
		self.ser.parity = serial.PARITY_ODD #set parity check: no parity
		self.ser.stopbits = serial.STOPBITS_ONE #number of stop bits
		self.ser.timeout = 1    #non-block read
		self.ser.xonxoff = False     #disable software flow control
		self.ser.rtscts = False     #disable hardware (RTS/CTS) flow control
		self.ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
		
		try: 
			self.ser.open()
		except:
			logging.warning('Error open serial port')
			print ('error open serial port')
			self.ser.close()
			self.ser.open()
			raise Exception()

		if not self.ser.isOpen():
			logging.error('Serial port not open')
			print ('serial port not open')
			raise Exception()

		logging.info('Serial port opened: ' + self.ser.portstr)

	# close serial connection
	def _close_serial_connection(self):
		'''
		Closes the serial connection

		Input:
			None

		Output:
			None
		'''
		logging.debug('Closing serial connection')
		print ('closing serial connection')
		# vpp43.close(self._vi)  # OLD
		self.ser.close()



	def reset(self):
		'''
		Resets all dacs to 0 volts

		Input:
			None

		Output:
			None
		'''
		logging.info('Resetting instrument')
		self.set_dacs_zero()
		self.get_all()

	def set_dacs_zero(self):
		for i in range(self._numdacs):
			self.do_set_dac(0,i+1)


	def reinitialize_dacs(self):
		bytetosend = 0b11100000 #111 is a re init all dacs
		message = "%c" % (bytetosend)
		reply = self._send_and_read(message.encode(), self.communication_bytes)
		return reply


	# Conversion of data
	def _mvoltage_to_bytes(self, mvoltage):
		'''
		Converts a mvoltage on a -10V to 10V scale to a 20-bit equivalent
		output is a list of three bytes

		Input:
			mvoltage (float) : a mvoltage in the 0mV-4000mV range

		Output:
			(dataH, dataL) (int, int) : The high and low value byte equivalent
		'''
		#//+10V=01111111111111111111
		#//-10V=10000000000000000000
		# logging.info('mvoltstobytes, voltage:')
		# logging.info(mvoltage)
		# if(mvoltage>0):
		# 	data = int(((float(mvoltage)/1000)+2)*((2**16-1)/2)/(self.Halfrange/1000)) #from mV to V and multiply with the resolution, divide by the 10V max
		# 	logging.info("positive, data:")
		# 	#data = bin(int(data) & 0xffffffff)
		# 	logging.info(data)
		# else:
		# 	data = int(((float(mvoltage)/1000)+2)*(2**16/2)/(self.Halfrange/1000)) #from mV to V and multiply with the resolution, divide by the 10V max
		# 	#data = data | 0b10000000000000000000
		# 	logging.info("negative, data:")
		# 	#data = bin(data)
		# 	logging.info(data)
		
		#bytevalue = int(round(mvoltage/4000.0*65535))
		#dataH = int(bytevalue/256)
		#dataL = bytevalue - dataH*256
		#return (dataH, dataL)
		bytevalue = int(round((mvoltage+self.Halfrange) / self.Fullrange * 65535))
		return bytevalue

	# def _numbers_to_mvoltages(self, numbers):
	# 	'''
	# 	Converts a list of bytes to a list containing
	# 	the corresponding mvoltages
	# 	'''
	# 	values = np.ones(self._numdacs) #initializes the values array to all ones
	#   #//calculate the bits to send to the dac out of the input values
	#   #//D= 20bit input code 
	# 	for i in range(self._numdacs):
	# 		bitValue = ((numbers[5 + 3*i]<<8) + (numbers[6 + 3*i]<<0))
	# 		if (bitValue & 0b1000000000000000): #check if the number is positive
	# 			#logging.info(i)
	# 			#logging.info('negative number bin(:')
	# 			bitValue=bitValue & 0b0111111111111111 #strip the first bit
	# 			#logging.info(bin(bitValue))
	# 			values[i]=(float(bitValue)/((2**16-1)/2))*(self.Halfrange/1000) #multiply with 2V
	# 			#logging.info('values[i]:')
	# 			#logging.info(values[i])
	# 			values[i]=values[i]*1000 # to get to mV
	# 			#logging.info('values[i]*1000:')
	# 			#logging.info(values[i])
	# 		else:
	# 			#logging.info(i)
	# 			#logging.info('positive number bin(:')
	# 			bitValue=bitValue & 0b0111111111111111 #strip the first bit
	# 			#logging.info(bin(bitValue))
	# 			values[i]=-(self.Halfrange/1000)+(float(bitValue)/(65536.0/2))*(self.Halfrange/1000) #multiply with 10V
	# 			#logging.info('values[i]:')
	# 			#logging.info(values[i])
	# 			values[i]=values[i]*1000 # to get to mV
	# 			#logging.info('values[i]*1000:')
	# 			#logging.info(values[i])
	# 		#values[i] = int(((numbers[5 + 4*i]<<16) + (numbers[6 + 4*i]<<8) + (numbers[7 + 4*i]<<0)),2)-(1<<20)
	# 		#values[i] = (( 20*((numbers[5 + 4*i]<<16) + (numbers[6 + 4*i]<<8) + (numbers[7 + 4*i]<<0)) )/ 1048575.0) + 10
	# 		#logging.info('DAC: ')
	# 		#logging.info(numbers[4 + 4*i] )
	# 		#logging.info('Val: ')
	# 		#logging.info(values[i])
	# 	return values
	# 	#return numbers

	def _numbers_to_mvoltages(self, byte_mess):

		'''
		Converts a list of bytes to a list containing
		the corresponding mvoltages
		'''
		values = list(range(self._numdacs))
		for i in range(self._numdacs):
			# takes two bytes, converts it to a 16 bit int and then divides by
			# the range and adds the offset due to the polarity

			values[i] = ((byte_mess[5 + 3 * i] * 256 + byte_mess[6 + 3 * i]) /
						 65535.0 * self.Fullrange)-self.Halfrange # + self.pol_num[i]
		return values

	# Communication with device
	def do_get_dac(self, channel):
		'''
		Returns the value of the specified dac

		Input:
			channel (int) : 1 based index of the dac

		Output:
			voltage (float) : dacvalue in mV
		'''
		logging.info('Reading dac%s', channel)
		mvoltages = self._get_dacs()
		logging.info(mvoltages)
		#return mvoltages[channel - 1]
		return mvoltages[channel - 1]

	def do_set_dac(self, mvoltage, channel):
		'''
		Sets the specified dac to the specified voltage

		Input:
			mvoltage (float) : output voltage in mV
			channel (int)    : 1 based index of the dac

		Output:
			reply (string) : errormessage
		'''
		logging.info('Setting dac%s to %.04f mV', channel, mvoltage)
		mvoltage = self._mvoltage_to_bytes(mvoltage)
		#logging.info('mvoltage after m_to_bytes: ')
		#logging.info(mvoltage)
		#logging.info('bin(channel: ')
		#logging.info(bin(channel))
		mvoltage_bytes = [0,0]
		mvoltage_bytes = [mvoltage >> i & 0xff for i in (8,0)] #0xff is 255
		channel = (int(channel)-1) | 0b10000000 #100 is a write operation
		#message = "%c%c%c%c" % (channel,mvoltage_bytes[0], mvoltage_bytes[1], mvoltage_bytes[2])
		message = [channel,mvoltage_bytes[0], mvoltage_bytes[1]]
		#logging.info('bin(message: ')
		#logging.info(bin(mvoltage_bytes[0]))
		#logging.info(bin(mvoltage_bytes[1]))
		#logging.info(bin(mvoltage_bytes[2]))
		#logging.info('message: ')
		#logging.info(message)
		reply = self._send_and_read(message, self.communication_bytes)
		#logging.info('bin(reply: ')
		#logging.info(bin(reply[0]))
		#logging.info(bin(reply[1]))
		#logging.info(bin(reply[2]))
		#logging.info(bin(reply[3]))       
		return reply

	def do_set_dac_fast(self, mvoltage, channel): #added by Daniel, seems to work

		if channel>4:
			print('Error: Only channels 1-4 have fast setting.')
		else:

			logging.info('Setting dac%s to %.04f mV', channel, mvoltage)
			mvoltage = self._mvoltage_to_bytes(mvoltage)
			mvoltage_bytes = [0,0]
			mvoltage_bytes = [mvoltage >> i & 0xff for i in (8,0)]
			channel = (int(channel)-1) | 0b11000000 #110 is a write fast operation
			message = [channel,mvoltage_bytes[0], mvoltage_bytes[1]]

			reply = self._send_and_read(message, 0)

			return reply

	def do_ramp_dac(self, mvoltage, channel):  #added by Daniel, fucks it up completly right now...
		if channel>2:
			print('Error: Only channels 1-2 have ramping.')
		else:

			logging.info('Setting dac%s to %.04f mV', channel, mvoltage)
			mvoltage = self._mvoltage_to_bytes(mvoltage)
			mvoltage_bytes = [0,0]
			mvoltage_bytes = [mvoltage >> i & 0xff for i in (8,0)]
			channel = (int(channel)-1) | 0b10100000 #110 is a ramp operation
			message = [channel,mvoltage_bytes[0], mvoltage_bytes[1]]

			reply = self._send_and_read(message, self.communication_bytes)

			return reply

	def do_set_trigger(self):
		'''
		Sets the trigger; trigger is 1ms and around 4.2V

		Input:
			none
		Output:
			reply (string) : errormessage
		'''
		logging.debug('Trigger out')
		message = "%c%c%c%c" % (4, 0, 2, 6)
		reply = self._send_and_read(message.encode())
		return reply

	def get_dacs(self):
		mvoltages = self._get_dacs()
		for i in range(self._numdacs):
			print('dac{}: '.format(i+1)+str(mvoltages[i]))
		return mvoltages

	def _get_dacs(self):
		'''
		Reads from device and returns all dacvoltages in a list

		Input:
			None

		Output:
			voltages (float[]) : list containing all dacvoltages (in mV)
		'''
		logging.debug('Getting dac voltages from instrument')
	
		# first 3 bit are control, last 5 DAC number
		message = '\x40' #0b01000000 = 010 = read all dacs
		#logging.info(sys.getsizeof(message))
		reply = self._send_and_read(message.encode(), self._numdacs*self.communication_bytes+4)
		#logging.info(reply)
		mvoltages = self._numbers_to_mvoltages(reply)
		return mvoltages

	def _send_and_read(self, message, bytestoread):
		'''
		Send <message> to the device and read answer.
		Raises an error if one occurred
		Returns a list of bytes

		Input:
			message (string)    : string conform the IST_20 protocol

		Output:
			data_out_numbers (int[]) : return message
		'''
		logging.info('Sending %r', message)

		# clear input buffer
		self.ser.flushInput()
		#logging.info('Flushed input')
		#vpp43.write(self._vi, message) # OLD
		self.ser.write(message) # NEW
		#logging.info('Wrote Message')

# In stead of blocking, we could also poll, but it's a bit slower
#        print visafunc.get_navail(self.lib, self._vi)
#        if not visafunc.wait_data(self._vi, 2, 0.5):
#            logging.error('Failed to receive reply from IST_20 rack')
#            return False

		#data1 = visafunc.readn(self._vi, 2)  # OLD
		#sleep(2)
		#logging.info(self.ser.readline())
		s=0
		data1 = []
		while s < bytestoread:
			data1.append(ord(self.ser.read()))
			#logging.info(s)
			s=s+1
		#data1 = [ord(s) for s in data1]
		#data2 = np.reshape(data1,(-1,4))
		#logging.info('finished reading')
		#data2 = np.uint32(data1) #from string to 32bit
		data2 = data1
		#logging.info('converted to uint32')
		#logging.info('sendAndRead: %s', data2)

		return data2

	def set_pol_dacrack(self, flag, channels, get_all=True):
		'''
		Changes the polarity of the specified set of dacs

		Input:
			flag (string) : 'BIP', 'POS' or 'NEG'
			channel (int) : 0 based index of the rack
			get_all (boolean): if True (default) perform a get_all

		Output:
			None
		'''
		flagmap = {'NEG': -self.Fullrange, 'BIP': -self.Halfrange, 'POS': 0}
		if flag.upper() not in flagmap:
			raise KeyError('Tried to set invalid dac polarity %s', flag)

		val = flagmap[flag.upper()]
		for ch in channels:
			self.pol_num[ch - 1] = val
			# self.set_parameter_bounds('dac%d' % (i+1), val, val +
			# self.Fullrange.0)

		if get_all:
			self.get_all()

	def get_pol_dac(self, channel):
		'''
		Returns the polarity of the dac channel specified

		Input:
			channel (int) : 1 based index of the dac

		Output:
			polarity (string) : 'BIP', 'POS' or 'NEG'
		'''
		val = self.pol_num[channel - 1]

		if (val == -self.Fullrange):
			return 'NEG'
		elif (val == -self.Halfrange):
			return 'BIP'
		elif (val == 0):
			return 'POS'
		else:
			return 'Invalid polarity in memory'

	def get_numdacs(self):
		'''
		Get the number of DACS.
		'''
		return self._numdacs


	def _gen_ch_set_func(self, fun, ch):
		def set_func(val):
			return fun(val, ch)
		return set_func

	def _gen_ch_get_func(self, fun, ch):
		def get_func():
			return fun(ch)
		return get_func

	def get_all(self):
		return self.snapshot(update=True)