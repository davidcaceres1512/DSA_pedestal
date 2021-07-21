import logging
import serial
import os
import datetime

fullpath='/home/soporte/Downloads/DATA_PEDESTAL/LOG_FIRMWARE'
filex=datetime.datetime.now().strftime("%d_%m_%Y-%H:%M:%S-pedestal_timestamp_changej.log")

root_logger= logging.getLogger()
root_logger.setLevel(logging.DEBUG) # or whatever

handler = logging.FileHandler(os.path.join(fullpath,filex), 'w', 'utf-8') # or whatever
handler.setFormatter(logging.Formatter('%(asctime)s %(message)s')) # or whatever
root_logger.addHandler(handler)
logging.getLogger().addHandler(logging.StreamHandler())

nucleo144Serial = serial.Serial(
    #port='COM5',
    port='/dev/ttyACM1', #verified that uc_pedestal is connected into port ttyACM1
    baudrate=1000000,
    parity='N',
    stopbits=1,
    bytesize=8
)
nucleo144Serial.isOpen()
nucleo144Serial.write(b'r')

print("iniciando log")

while 1 :
    out = ''
    #while ser.inWaiting() > 0:
    out = nucleo144Serial.read_until(b'\n').decode('utf-8').strip()
    #out= ser.read(1).hex()
    #print(out)
    logging.debug(out)
