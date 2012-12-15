"""
Listen to serial, return most recent numeric values
Lots of help from here:
http://stackoverflow.com/questions/1093598/pyserial-how-to-read-last-line-sent-from-serial-device
"""
from threading import Thread
import time
import serial
import pprint

last_received = ''
def receiving(ser):
    global last_received
    buffer = ''
    while True:
        buffer = buffer + ser.read(ser.inWaiting())
        if '\n' in buffer:
            lines = buffer.split('\n') # Guaranteed to have at least 2 entries
            last_received = lines[-2]
            #If the Arduino sends lots of empty lines, you'll lose the
            #last filled line, so you could make the above statement conditional
            #like so: if lines[-2]: last_received = lines[-2]
            buffer = lines[-1]


class SerialData(object):
    def __init__(self, init=50):
        try:
            self.ser = ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                xonxoff=0,
                rtscts=0,
                interCharTimeout=None
            )
        except serial.serialutil.SerialException:
            #no serial connection
            self.ser = None
        else:
            Thread(target=receiving, args=(self.ser,)).start()

    def empty_value_hash(self):
      return {
          'acc_raw_x': 1.0,
          'acc_raw_y': 1.0,
          'acc_raw_z': 1.0,
          'acc_x': 1.0,
          'acc_y': 1.0,
          'acc_z': 1.0,
          'acc_gx': 1.0,
          'acc_gy': 1.0,
          'acc_gz': 1.0,

          'gyro_raw_x': 1.0,
          'gyro_raw_y': 1.0,
          'gyro_raw_z': 1.0,
          'gyro_smooth_x': 1.0,
          'gyro_smooth_y': 1.0,
          'gyro_smooth_z': 1.0,
          'gyro_x': 1.0,
          'gyro_y': 1.0,
          'gyro_z': 1.0,

          'dcm_q0': 1.0,
          'dcm_q1': 1.0,
          'dcm_q2': 1.0,
          'dcm_q3': 1.0,
          'dcm_x': 1.0,
          'dcm_y': 1.0,
          'dcm_z': 1.0
          }

    def values(self, raw_line):
        index_name = {
            0: 'acc_raw_x',
            1: 'acc_raw_y',
            2: 'acc_raw_z',
            3: 'acc_x',
            4: 'acc_y',
            5: 'acc_z',
            6: 'acc_gx',
            7: 'acc_gy',
            8: 'acc_gz',

            9:  'gyro_raw_x',
            10: 'gyro_raw_y',
            11: 'gyro_raw_z',
            12: 'gyro_smooth_x',
            13: 'gyro_smooth_y',
            14: 'gyro_smooth_z',
            15: 'gyro_x',
            16: 'gyro_y',
            17: 'gyro_z',

            18: 'dcm_q0',
            19: 'dcm_q1',
            20: 'dcm_q2',
            21: 'dcm_q3',
            22: 'dcm_x',
            23: 'dcm_y',
            24: 'dcm_z',
        }
        value_list = raw_line.strip().split('\t')
        value_hash = {}
        for index, value in enumerate(value_list):
            name = index_name[index]
            value_hash[name] = float(value)

        return value_hash
        
    def next(self):
        if not self.ser:
            return self.empty_value_hash()
        #return a float value or try a few times until we get one
        for i in range(40):
            raw_line = last_received
            try:
                return self.values(raw_line)
            except ValueError:
                #print 'bogus data',raw_line
                time.sleep(.005)
        return self.empty_value_hash()
    def write(self, value):
        self.ser.write(value + '\n')
    def __del__(self):
        if self.ser:
            self.ser.close()

if __name__=='__main__':
    s = SerialData()
    for i in range(500):
        #time.sleep(.015)
        print s.next()
