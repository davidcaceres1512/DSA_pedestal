import logging
import os
import datetime
import argparse
import subprocess

fullpath='/home/soporte/Downloads/DATA_PEDESTAL/LOG_FIRMWARE'

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--verbose", help="Mostrar información de depuración", action="store_true")
parser.add_argument("-f", "--file", help="Nombre de archivo a procesar, coloca este flag es importante")
parser.add_argument("-i", "--ip", help="ip del equipo para enviarle ping, enviar este flag entre comillas")
args = parser.parse_args()

# Aquí procesamos lo que se tiene que hacer con cada argumento
if args.verbose:
    print("depuración activada!!!")

if args.file:
    print("El nombre de archivo a procesar es: "+ args.file)
    filex=datetime.datetime.now().strftime("%d_%m_%Y-%H:%M:%S-ping-"+args.file+".log")

root_logger= logging.getLogger()
root_logger.setLevel(logging.DEBUG) # or whatever

handler = logging.FileHandler(os.path.join(fullpath,filex), 'w', 'utf-8') # or whatever
handler.setFormatter(logging.Formatter('%(asctime)s %(message)s')) # or whatever
root_logger.addHandler(handler)
logging.getLogger().addHandler(logging.StreamHandler())


def ping(ip):
    #completed1 = subprocess.run(['wireshark','-i','1','-k'])
    #global cmd_wireshark
    global completed1
    if args.ip:
        ip=args.ip
        cmd_ping= 'ping '+ip
    else:
        cmd_ping='ping 10.10.10.28'
    
    #completed1 = subprocess.run(cmd_ping, shell=True,capture_output=True)
    completed1 = subprocess.Popen(cmd_ping, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    print('returncode_ping:', completed1.returncode)

ping(args.ip)
print("iniciando log")
#print(completed1.stdout)
#logging.debug(completed1.stdout)
while completed1.poll() is None:
    line = completed1.stdout.readline()
    #print("Print:" + line)
    logging.debug(line)
