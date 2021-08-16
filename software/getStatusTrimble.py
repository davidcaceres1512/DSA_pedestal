import requests
import json
import base64

#*******************************************
#*******************************************
#***********variable json apirest sincrho***

#***getStatusTrimble set*************
enable = 1
#****************************

#*******************************************
#*******************************************
getStatusTrimble_url     = 'http://192.168.1.28/getStatusTrimble?params='

json_getStatusTrimble    = json.dumps({"enable": enable})

base64_getStatusTrimble = base64.urlsafe_b64encode(json_getStatusTrimble.encode('ascii'))

print(base64_getStatusTrimble)#choose the actual command

complete_url_getStatusTrimble = getStatusTrimble_url + base64_getStatusTrimble.decode('ascii')

print(complete_url_getStatusTrimble)#choose the actual command

r = requests.get(complete_url_getStatusTrimble, timeout=5)#choose the actual command
