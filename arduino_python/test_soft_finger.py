import serial
from time import sleep
import sys

COM_PORT = '/dev/ttyACM0'  # 請自行修改序列埠名稱
BAUD_RATES = 9600
ser = serial.Serial(COM_PORT, BAUD_RATES)

try:
    while True:
        # 接收用戶的輸入值並轉成小寫
        choice = input('TYPE: 0 to open, 1 to flex-flip, 2 to close, other num below 255 to test').lower()

        if choice == '0':
            print('Sending command')
            ser.write(b'0\n') 
            sleep(0.5)             
        if choice == '1':
            print('Sending command')
            ser.write(b'1\n') 
            sleep(0.5)             
        elif choice == '2':
            print('Sending command')
            ser.write(b'2\n')
            sleep(0.5)
        elif choice == '3':
            print('Sending command')
            ser.write(b'3\n')
            sleep(0.5)
        elif choice == '4':
            print('Sending command')
            ser.write(b'4\n')
            sleep(0.5)        
        elif choice == 'e':
            ser.close()
            print('再見！')
            sys.exit()
        else:
            print('Test pressure')
            msg = choice+'\n'
            ser.write(str.encode(msg))

        while ser.in_waiting:
            mcu_feedback = ser.readline().decode()  
            print('Feedback：', mcu_feedback)
            
except KeyboardInterrupt:
    ser.close()
    print('再見！')
