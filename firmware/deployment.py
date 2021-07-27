
#./arduino-cli compile -v --fqbn STM32:stm32:Nucleo_144:pnum=NUCLEO_F767ZI --build-property "build.extra_flags=-DSTM32F767xx -DHAL_UART_MODULE_ENABLED -DSERIAL_RX_BUFFER_SIZE=1024 -DSERIAL_TX_BUFFER_SIZE=1024 -DSERIAL_BUFFER_SIZE=1024 -DAREST_NUMBER_FUNCTIONS=20 -DAREST_NUMBER_VARIABLES=20 -DMQTT_MAX_PACKET_SIZE=3072 -DMQTT_SOCKET_TIMEOUT=1 -faggressive-loop-optimizations" ./Pedestal
#arduino-cli board list
#arduino-cli upload --port ttyACM1 --fqbn STM32:stm32:Nucleo_144:pnum=NUCLEO_F767ZI ./Pedestal
import subprocess
import threading
import time

print("ingrese el nombre de la carpeta que desea cargar")
firmware=input()

cmd_shell='gnome-terminal -x bash -c \"./home/soporte/Downloads/bin/arduino-cli compile -v --fqbn STM32:stm32:Nucleo_144:pnum=NUCLEO_F767ZI --build-property \"build.extra_flags=-DSTM32F767xx -DHAL_UART_MODULE_ENABLED -DSERIAL_RX_BUFFER_SIZE=1024 -DSERIAL_TX_BUFFER_SIZE=1024 -DSERIAL_BUFFER_SIZE=1024 -DAREST_NUMBER_FUNCTIONS=20 -DAREST_NUMBER_VARIABLES=20 -DMQTT_MAX_PACKET_SIZE=3072 -DMQTT_SOCKET_TIMEOUT=1 -faggressive-loop-optimizations\" ./'+firmware+'; exec bash\"'
cmd='./home/soporte/Downloads/bin/arduino-cli compile -v --fqbn STM32:stm32:Nucleo_144:pnum=NUCLEO_F767ZI --build-property \"build.extra_flags=-DSTM32F767xx -DHAL_UART_MODULE_ENABLED -DSERIAL_RX_BUFFER_SIZE=1024 -DSERIAL_TX_BUFFER_SIZE=1024 -DSERIAL_BUFFER_SIZE=1024 -DAREST_NUMBER_FUNCTIONS=20 -DAREST_NUMBER_VARIABLES=20 -DMQTT_MAX_PACKET_SIZE=3072 -DMQTT_SOCKET_TIMEOUT=1 -faggressive-loop-optimizations\" .. .. .. .. ./home/soporte/Downloads/bin/'
cmd='./home/soporte/Downloads/bin/arduino-cli compile -v --fqbn STM32:stm32:Nucleo_144:pnum=NUCLEO_F767ZI --build-property \"build.extra_flags=-DSTM32F767xx -DHAL_UART_MODULE_ENABLED -DSERIAL_RX_BUFFER_SIZE=1024 -DSERIAL_TX_BUFFER_SIZE=1024 -DSERIAL_BUFFER_SIZE=1024 -DAREST_NUMBER_FUNCTIONS=20 -DAREST_NUMBER_VARIABLES=20 -DMQTT_MAX_PACKET_SIZE=3072 -DMQTT_SOCKET_TIMEOUT=1 -faggressive-loop-optimizations\" ./'+firmware+'; exec bash'
print(cmd_shell)

def openShell(command):
    shell = subprocess.run(command, shell=True)
    
    #shell = subprocess.run(command, shell=True)
    print('returncode_shell: ', shell.returncode)



T_cmdShell1 = threading.Thread(target=openShell, args=(cmd_shell,))

T_cmdShell1.start()




