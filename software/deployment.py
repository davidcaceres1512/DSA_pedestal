
import subprocess
import threading
import time

cmd_shell=[
    'gnome-terminal -x bash -c \"/usr/bin/python /home/soporte/Downloads/DSA_pedestal/software/nucleo_logger_server.py; exec bash\"',
    'gnome-terminal -x bash -c \"/usr/bin/python /home/soporte/Downloads/DSA_pedestal/software/nucleo_logger_pedestal.py; exec bash\"',
    'gnome-terminal -x bash -c \"/usr/bin/python /home/soporte/Downloads/DSA_pedestal/software/streamdecodetest.py; exec bash\"',
    'gnome-terminal -x bash -c \"/usr/bin/python /home/soporte/Downloads/DSA_pedestal/software/ping_logger.py -f pedestal -i \"10.10.10.27\"; exec bash\"',
    'gnome-terminal -x bash -c \"/usr/bin/python /home/soporte/Downloads/DSA_pedestal/software/ping_logger.py -f server -i \"10.10.10.28\"; exec bash\"'
] 

def openWireshark():
    #cmd_wireshark= 'wireshark -i 1 -k -f \"tcp port 1883\"' # put "wireshark -D" for view the interfaces enumerate list ("-i 1" is the interface eno2)
    cmd_wireshark= 'wireshark -i 2 -k' 
    completed1 = subprocess.run(cmd_wireshark, shell=True)
    print('returncode_wireshark:', completed1.returncode)

def openShell(command):
    shell = subprocess.run(command, shell=True)
    print('returncode_shell_'+command[-10]+': ', shell.returncode)


T_wireshark = threading.Thread(target=openWireshark)
T_cmdShell1 = threading.Thread(target=openShell, args=[cmd_shell[0]])
T_cmdShell2 = threading.Thread(target=openShell, args=[cmd_shell[1]])
T_cmdShell3 = threading.Thread(target=openShell, args=[cmd_shell[2]])
T_cmdShell4 = threading.Thread(target=openShell, args=[cmd_shell[3]])
T_cmdShell5 = threading.Thread(target=openShell, args=[cmd_shell[4]])

T_wireshark.start()
T_cmdShell1.start()
T_cmdShell2.start()
time.sleep(28) #timeout for the uc_pedestal to reboot and initialize
#http://127.0.0.1:8080/dev_conf/1/start/
T_cmdShell4.start()
T_cmdShell5.start()
T_cmdShell3.start()


