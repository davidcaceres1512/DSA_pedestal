#execution example: /usr/bin/python /home/soporte/Downloads/DSA_pedestal/software/timestamp_checker.py -f 20210811-094215 -v
import os
import argparse
import subprocess

global filex

fullpath='Downloads/DATA_PEDESTAL/PED_RT'

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--verbose", help="Mostrar información de depuración", action="store_true")
parser.add_argument("-f", "--file", help="Nombre de archivo a procesar, coloca este flag es importante")

args = parser.parse_args()

# Aquí procesamos lo que se tiene que hacer con cada argumento
if args.verbose:
    print("depuración activada!!!")

if args.file:
    print("El nombre de la carpeta a procesar es: "+ args.file)
    filex=os.path.join(fullpath,args.file,'HDF5')
else:
    print("error, no has introducido la carpeta a testear")
    print("introduce el argumento, este porgrama se cerrara en 5 segundos")
    time.sleep(5)
    os._exit(0)

def ls():
    global result
    cmd_ls='cd && cd '+filex+'/ && ls -l'
    #cmd_ls="ls -l"
    result = subprocess.run(cmd_ls, shell=True,capture_output=True, text=True)
    print('returncode_ls:', result.returncode)

ls()
print("iniciando testeo")
print("")
#print(result.stdout)#pirnt all output 

lines = result.stdout.split('\n') #lines is a list of line per line

"""for i in range(len(lines)-1): #menos uno porque el ultimo valor de la lista esta vacia. 
    if i>0:
        print(lines[i][-15:-5])"""

for i in range(len(lines)-1):  
    if i>1:
        var=lines[i-1]
        dv=int(lines[i][-15:-5])-int(var[-15:-5])#1628620000
        if dv!=1:
            print("hay un salto de timestamp de "+str(dv)+" seg. el salto sucede en los archivos: "+lines[i-1][-15:]+" ---> "+lines[i][-15:])
            if args.verbose:
                print("verbose enabled, show more info of the files")
                print(lines[i-1])
                print(lines[i])
                print("")


#varlist=lines[1]
#print(int(varlist[-15:-5]))#10 digits  of timestamp-integer, offset=5 -> ".hdf5"
#print(int(lines[1][-15:-5]))

