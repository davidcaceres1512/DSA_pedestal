import logging
import serial

root_logger= logging.getLogger()
root_logger.setLevel(logging.DEBUG) # or whatever

handler = logging.FileHandler('pedestaljjnucl.log', 'w', 'utf-8') # or whatever
handler.setFormatter(logging.Formatter('%(asctime)s %(message)s')) # or whatever
root_logger.addHandler(handler)
logging.getLogger().addHandler(logging.StreamHandler())

ser = serial.Serial(
    port='COM5',
    baudrate=1000000,
    parity='N',
    stopbits=1,
    bytesize=8
)
ser.isOpen()

print("iniciando log")

while 1 :
    out = ''
    #while ser.inWaiting() > 0:
    out = ser.read_until(b'\n').decode('utf-8').strip()
    #out= ser.read(1).hex()
    print(out)
    logging.debug(out)
