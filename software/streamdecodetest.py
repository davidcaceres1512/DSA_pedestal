from numpy.core.fromnumeric import var
import paho.mqtt.client as mqtt
import time
import datetime
import base64
import numpy
import signal
import sys
import logging
import pandas as pd
from queue import Queue
#----------------------------
import os
import h5py
#----------------------------
q=Queue()
dataMessage = {}
#----------------------------
fullpath='/home/soporte/Downloads/DATA_PEDESTAL/PED_RT'
#wpath='Users\soporte\Downloads'
#-----------------------------

directoryx=''


directoryx=datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
os.mkdir(os.path.join(fullpath,directoryx))
os.mkdir(os.path.join(fullpath,directoryx,'LOG'))
os.mkdir(os.path.join(fullpath,directoryx,'HDF5'))

#updater = Updater(token='1506060089:AAEC18kF80Mc1lCX2T2DLHKQybOLmx56Et0', use_context=True)
#dispatcher = updater.dispatcher
#logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',level=logging.INFO)
#logging.getLogger("telegram").setLevel(logging.WARNING)
root_logger= logging.getLogger()
root_logger.setLevel(logging.DEBUG) # or whatever
handler2 = logging.FileHandler(os.path.join(fullpath,directoryx,'LOG','pedestal_mqtt.log'), 'w', 'utf-8') # or whatever
handler2.setFormatter(logging.Formatter('%(asctime)s %(message)s')) # or whatever
root_logger.addHandler(handler2)
logging.getLogger().addHandler(logging.StreamHandler())

MODE_STOP      = 0x00
MODE_SPEED     = 0x01
MODE_POSITION  = 0x02
RX_AZIMUTH     = 0x27
RX_ELEVATION   = 0x47
TX_AZIMUTH     = 0x25
TX_ELEVATION   = 0x45
NONE_AXIS      = 0x65
RX_FUNCTION    = 0x30
TX_FUNCTION    = 0x40
HEADER         = 0x7e

MIN_SPEED  = -180.0
MAX_SPEED  =  180.0

SHRT_MIN  = -32768
SHRT_MAX  =  32768
USHRT_MAX =  65535

mqtt_status=False
start_time=0
broker="192.168.1.30"
port=1883
data="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"
azposTelegram   = 0
azspeedTelegram = 0
elposTelegram   = 0
elspeedTelegram = 0
start_variable = False

#this variables its different of the function getspeedPosition
cw = 0 #count while main program for defined buffers dictionaries prev-act
cc = 0
AzPositionM=[]
AzSpeedM=[]
ElPositionM=[]
ElSpeedM=[]
RawDataM=[]
TimestampM=0


def preprocessingOutliers(varInput):
    s=pd.Series(varInput)
    threshold=s.mad()
    threshold=5
    print(threshold)
    for i in range(len(varInput)):  
      if i>0:
        dev=abs(varInput[i]-varInput[i-1])
        if dev>threshold:
        #ele_pos[i]=ele_pos[i-1]+threshold
          varInput[i]=varInput[i-10]

def preprocessingZero(varInput):
    for i in range(len(varInput)):  
      if varInput[i]>=359.5:
      #ele_pos[i]=ele_pos[i-1]+threshold
        varInput[i]=0.0

def getSpeedPosition(msg_b64):
    AzPosition=[]
    AzSpeed=[]
    ElPosition=[]
    ElSpeed=[]
    RawData=[]
    Timestamp=0
    RawData = numpy.frombuffer(base64.decodebytes(msg_b64.encode()),numpy.dtype('B'))
    print("Decoded raw data length: "+str(len(RawData)))
    #Timestamp = int(RawData[len(RawData)-4])+int(RawData[len(RawData)-3]<<8)+int(RawData[len(RawData)-2]<<16)+int(RawData[len(RawData)-1]<<24)
    Timestamp = (RawData[len(RawData)-4])|(RawData[len(RawData)-3]<<8)|(RawData[len(RawData)-2]<<16)|(RawData[len(RawData)-1]<<24)
    counter = 0
    while counter < len(RawData)-4:
      #print(counter)
      if RawData[counter]==HEADER:
        if RawData[counter+1]==RX_FUNCTION:
          if RawData[counter+2]==RX_AZIMUTH or RawData[counter+2]==RX_ELEVATION:
            #print("Found it!")
            iangle = 0.0
            ispeed = 0.0
            hadstuffing = 0
            
            if (counter+hadstuffing+4<len(RawData)-4):
              if RawData[counter+hadstuffing+4]==HEADER-1:
                hadstuffing+=1
                iangle = int(0x70|RawData[counter+hadstuffing+4]&0x0F)
              else:
                iangle = int(RawData[counter+hadstuffing+4])
            else:
              logging.debug("Warning: Index out of bounds. The packet is incomplete or corrupted.")
              break
            
            if (counter+hadstuffing+5<len(RawData)-4):
              if RawData[counter+hadstuffing+5]==HEADER-1:
                hadstuffing+=1
                iangle = iangle + int((0x70|RawData[counter+hadstuffing+5]&0x0F)<<8)
              else:
                iangle = iangle + int(RawData[counter+hadstuffing+5]<<8)
            else:
              logging.debug("Warning: Index out of bounds. The packet is incomplete or corrupted.")
              break
            
            if (counter+hadstuffing+6<len(RawData)-4):
              if RawData[counter+hadstuffing+6]==HEADER-1:
                hadstuffing+=1
                ispeed = int(0x70|RawData[counter+hadstuffing+6]&0x0F)
              else:
                ispeed = int(RawData[counter+hadstuffing+6])
            else:
              logging.debug("Warning: Index out of bounds. The packet is incomplete or corrupted.")
              break
            
            if (counter+hadstuffing+7<len(RawData)-4):
              if RawData[counter+hadstuffing+7]==HEADER-1:
                hadstuffing+=1
                ispeed = ispeed + int((0x70|RawData[counter+hadstuffing+7]&0x0F)<<8)
              else:
                ispeed = ispeed + int(RawData[counter+hadstuffing+7]<<8)
            else:
              logging.debug("Warning: Index out of bounds. The packet is incomplete or corrupted.")
              break
            
            if (counter+2<len(RawData)-4):
              if RawData[counter+2]==RX_AZIMUTH:
                AzPosition.append(iangle*360.0/USHRT_MAX)
                AzSpeed.append(((ispeed-SHRT_MIN)*(MAX_SPEED-MIN_SPEED)/(SHRT_MAX-SHRT_MIN))+MIN_SPEED)
              elif RawData[counter+2]==RX_ELEVATION:
                ElPosition.append(iangle*360.0/USHRT_MAX)
                ElSpeed.append(((ispeed-SHRT_MIN)*(MAX_SPEED-MIN_SPEED)/(SHRT_MAX-SHRT_MIN))+MIN_SPEED)
              else:
                counter+=1
                continue
            else:
              logging.debug("Warning: Index out of bounds. The packet is incomplete or corrupted.")
              break
            
            counter = counter+hadstuffing+13
          else:
            counter+=1
            continue
        else:
          counter+=1
          continue
      else:
        counter+=1
        continue
    return RawData,AzPosition,AzSpeed,ElPosition,ElSpeed,Timestamp

def start(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Will send an automatic message every second.")
    global start_variable
    start_variable = True

def stop(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Will stop sending automatic messages.")
    global start_variable
    start_variable = False

def echo(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text=update.message.text)

def caps(update, context):
    text_caps = ' '.join(context.args).upper()
    context.bot.send_message(chat_id=update.effective_chat.id, text=text_caps)

def status(update, context):
    global mqtt_status
    global start_time
    if mqtt_status:
      context.bot.send_message(chat_id=update.effective_chat.id, text="MQTT connected")
      context.bot.send_message(chat_id=update.effective_chat.id, text="Last message was received "+str((time.process_time_ns()-start_time)/1000000)+" milliseconds ago.")
    else:
      context.bot.send_message(chat_id=update.effective_chat.id, text="MQTT disconnected")

def az_pos(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Azimuth position is "+str(azposTelegram))

def az_speed(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Azimuth speed is "+str(azspeedTelegram))

def el_pos(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Elevation position is "+str(elposTelegram))

def el_speed(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Elevation speed is "+str(elspeedTelegram))

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    global mqtt_status
    mqtt_status = True
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("JRO_topic")

def on_disconnect(client, userdata, rc):
    global mqtt_status
    mqtt_status = False
    if rc != 0:
        print("Unexpected disconnection.")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    #print(msg.topic+" "+str(msg.payload))
    global start_time
    #variables (the last sample per variable) for telegram
    global azposTelegram
    global azspeedTelegram
    global elposTelegram
    global elspeedTelegram
    #----------------------
    #variables that you can modified for hdf5 files
    global RawDataM, AzPositionM, AzSpeedM, ElPositionM, ElSpeedM, TimestampM

    dataQueue = {'azpos': [], 'elpos': [], 'azvel': [], 'elvel': [], 'timestmp': None}

    start_time = time.process_time_ns()
    RawDataM,AzPositionM,AzSpeedM,ElPositionM,ElSpeedM,TimestampM = getSpeedPosition(str(msg.payload.decode("utf-8")))
    #preprocessingOutliers(ElPosition)
    #preprocessingZero(ElPosition)
    #preprocessingOutliers(AzPosition)

    dataQueue['azpos'] = AzPositionM
    dataQueue['elpos'] = ElPositionM
    dataQueue['azvel'] = AzSpeedM
    dataQueue['elvel'] = ElSpeedM
    dataQueue['timestmp'] = TimestampM

    q.put(dataQueue)

    #xx = numpy.array([[AzPositionM],[AzSpeedM],[ElPositionM],[ElSpeedM]], dtype=object)
    with numpy.printoptions(precision=3, suppress=True):
        #print("Azimuth position array:")
        #print(numpy.array(AzPosition))
        #print("Elevation position array:")
        #print(numpy.array(ElPosition))
        #print("Azimuth speed array:")
        #print(numpy.array(AzSpeed))
        #print("Elevation speed array:")
        #print(numpy.array(ElSpeed))
        #print("Timestamp is:"+str(Timestamp))
        logging.debug("Azimuth position array:")
        logging.debug(numpy.array(AzPositionM))
        logging.debug(numpy.size(numpy.array(AzPositionM)))
        logging.debug("Elevation position array:")
        logging.debug(numpy.array(ElPositionM))
        logging.debug(numpy.size(numpy.array(ElPositionM)))
        logging.debug("Azimuth speed array:")
        logging.debug(numpy.array(AzSpeedM))
        logging.debug(numpy.size(numpy.array(AzSpeedM)))
        logging.debug("Elevation speed array:")
        logging.debug(numpy.array(ElSpeedM))
        logging.debug(numpy.size(numpy.array(ElSpeedM)))
        logging.debug("Timestamp is:"+str(TimestampM))
    if(len(AzPositionM)>0 and len(AzSpeedM)>0 and len(ElPositionM)>0 and len(ElSpeedM)>0):
      azposTelegram   = AzPositionM[-1]
      azspeedTelegram = AzSpeedM[-1]
      elposTelegram   = ElPositionM[-1]
      elspeedTelegram = ElSpeedM[-1]

def on_publish(client,userdata,result):             #create function for callback
    print("data published \n")
    pass

def hdf5Write():
    global directoryx
    global dataMessage
    global cc
    epoc = int(TimestampM)
    meta = 'PE'
    epoch_time = int(epoc)
    #print(type(epoch_time))
    time_val = time.localtime(int(epoc))
    ext = ".hdf5"
    filex = "%s%4.4d%3.3d%10.4d%s" % (meta, time_val.tm_year, time_val.tm_yday, epoch_time, ext)
    #filename = os.path.join(os.sep, "C:" + os.sep, wpath ) #for windows
    filename = os.path.join(fullpath, directoryx, 'HDF5', filex)
    print(filename)

    #new function for constant samples...
    setSample = 100
    azi_pos = AzPositionM
    ele_pos = ElPositionM
    azi_vel = AzSpeedM
    ele_vel = ElSpeedM
    dynBuffAct = {'azpos': [], 'elpos': [], 'azvel': [], 'elvel': []}
    dynBuffPrev = {'azpos': [], 'elpos': [], 'azvel': [], 'elvel': []}

    if (cc == 0):
      dynBuffPrev['azpos'] = azi_pos
      dynBuffPrev['elpos'] = ele_pos
      dynBuffPrev['azvel'] = azi_vel
      dynBuffPrev['elvel'] = ele_vel
      cc = 1
      pass

    """data_azi_pos = numpy.array(azi_pos)
    data_ele_pos = numpy.array(ele_pos)
    data_azi_vel = numpy.array(azi_vel)
    data_ele_vel = numpy.array(ele_vel)"""

    #hacer un extend hasta llegar a 100 para cada campo. se podria aprovechar los condicionales al paso para hacer elguardado de datos...
    # not forgotten, len(azi_pos)=len(azi_vel) and len(ele_pos)=len(ele_vel). For that, I use only two condicionals
    if (len(azi_pos) == 100):
      if (len(ele_pos) == 100):
        pass
      else:
        pass
      pass
    else:
      pass
      #comprueba que los campos de datos esten todos a 100 para recien almacenarlo en formato hdf5
    
    if (numpy.size(azi_pos) == 100):
      pass
      if (numpy.size(ele_pos) == 100):
        pass
    else:
      pass

    with h5py.File(filename, 'w') as fp:
      #print("Escribiendo HDF5...",epoc)
      #·················· Data·....······································
      grp = fp.create_group("Data")
      dset = grp.create_dataset("azi_pos", data=numpy.array(azi_pos))
      dset = grp.create_dataset("azi_pos_len", data=numpy.array(len(azi_pos)))
      dset = grp.create_dataset("ele_pos", data=numpy.array(ele_pos))
      dset = grp.create_dataset("azi_vel", data=numpy.array(azi_vel))
      dset = grp.create_dataset("ele_vel", data=numpy.array(ele_vel))
      dset = grp.create_dataset("utc", data=numpy.array(int(epoc)))
    pass

client = mqtt.Client()
client.on_connect = on_connect
client.on_disconnect  = on_disconnect 
client.on_message = on_message
client.on_publish = on_publish

client.connect(broker, port)
client.subscribe("JRO_topic")#subscribe(topic,qos)
client.loop_start()

#start_handler = CommandHandler('start', start)
#dispatcher.add_handler(start_handler)
#stop_handler = CommandHandler('stop', stop)
#dispatcher.add_handler(stop_handler)
#echo_handler = MessageHandler(Filters.text & (~Filters.command), echo)
#dispatcher.add_handler(echo_handler)
#caps_handler = CommandHandler('caps', caps)
#dispatcher.add_handler(caps_handler)
#status_handler = CommandHandler('status', status)
#dispatcher.add_handler(status_handler)
#az_pos_handler = CommandHandler('az_pos', az_pos)
#dispatcher.add_handler(az_pos_handler)
#az_speed_handler = CommandHandler('az_speed', az_speed)
#dispatcher.add_handler(az_speed_handler)
#el_pos_handler = CommandHandler('el_pos', el_pos)
#dispatcher.add_handler(el_pos_handler)
#el_speed_handler = CommandHandler('el_speed', el_speed)
#dispatcher.add_handler(el_speed_handler)
#updater.start_polling()

publish_time = time.process_time_ns()
telegram_time = time.process_time_ns()

listData=[]

while True:
  
  """ if cw>0 and cf==0:
    dataMAct=dataMessage
    if 
    if (len(listData)==2):
      listData.pop(0)
    listData.append(dataMAct)
    pass
  cw=1"""
  
  while not q.empty():
    dataMessage = q.get()
    #print("queue: ", dataMessage)
    #logging.debug("dataMessage queue:")
    #logging.debug(dataMessage)
    if (len(listData)==2):
      listData.pop(0)
    listData.append(dataMessage)
    #print("listData index 0: ",listData[0])
    if (len(listData)==2):
      #print("listData index 1: ",listData[1])
      #hacer un extend hasta llegar a 100 para cada campo. se podria aprovechar los condicionales al paso para hacer elguardado de datos...
      # not forgotten, len(azi_pos)=len(azi_vel) and len(ele_pos)=len(ele_vel). For that, I use only two condicionals
      if (0< len(listData[0]['azpos']) < 100):
        flagConstantAzSample= False
        extendAzposVar=100-len(listData[0]['azpos'])
        listData[0]['azpos'].extend(listData[1]['azpos'][:extendAzposVar])
        listData[0]['azvel'].extend(listData[1]['azvel'][:extendAzposVar])
        del listData[1]['azpos'][:extendAzposVar] #del extendAzposVar times 
        del listData[1]['azvel'][:extendAzposVar] 
        if (len(listData[0]['azpos']) == 100 and len(listData[0]['azvel']) == 100):
          #grabar en hdf5
          flagConstantAzSample= True
          pass
      else:
        #if(len(listData[0]['azpos']) == 0):
        #  retraso de todo una data...presentara muchos problemas ya que mientras pasa el tiempo se va desincronizar la data.
        #  pass
        pass

      if (0< len(listData[0]['elpos']) < 100):
        flagConstantElSample= False
        extendElposVar=100-len(listData[0]['elpos'])
        listData[0]['elpos'].extend(listData[1]['elpos'][:extendElposVar])
        listData[0]['elvel'].extend(listData[1]['elvel'][:extendElposVar])
        del listData[1]['elpos'][:extendElposVar] #del extendAzposVar times 
        del listData[1]['elvel'][:extendElposVar] 
        if (len(listData[0]['elpos']) == 100 and len(listData[0]['elvel']) == 100):
          #grabar en hdf5
          flagConstantElSample= True
          pass
      else:
        #if(len(listData[0]['azpos']) == 0):
        #  retraso de todo una data...presentara muchos problemas ya que mientras pasa el tiempo se va desincronizar la data.
        #  pass
        pass
      
      if(flagConstantAzSample==True and flagConstantElSample == True ):
        epoc = int(listData[0]['timestmp'])
        meta = 'PE'
        epoch_time = int(epoc)
        #print(type(epoch_time))
        time_val = time.localtime(int(epoc))
        ext = ".hdf5"
        filex = "%s%4.4d%3.3d%10.4d%s" % (meta, time_val.tm_year, time_val.tm_yday, epoch_time, ext)
        #filename = os.path.join(os.sep, "C:" + os.sep, wpath ) #for windows
        filename = os.path.join(fullpath, directoryx, 'HDF5', filex)
        print(filename)
        with h5py.File(filename, 'w') as fp:
          #print("Escribiendo HDF5...",epoc)
          #·················· Data·....······································
          grp = fp.create_group("Data")
          dset = grp.create_dataset("azi_pos", data=numpy.array(listData[0]['azpos']))
          #dset = grp.create_dataset("azi_pos_len", data=numpy.array(len(azi_pos)))
          dset = grp.create_dataset("ele_pos", data=numpy.array(listData[0]['elpos']))
          dset = grp.create_dataset("azi_vel", data=numpy.array(listData[0]['azvel']))
          dset = grp.create_dataset("ele_vel", data=numpy.array(listData[0]['elvel']))
          dset = grp.create_dataset("utc",     data=numpy.array(int(epoc)))
        
        with numpy.printoptions(precision=3, suppress=True):
          logging.debug("Azimuth position array hdf5:")
          logging.debug(numpy.array(listData[0]['azpos']))
          logging.debug(numpy.size(numpy.array(listData[0]['azpos'])))
          logging.debug("Elevation position array hdf5:")
          logging.debug(numpy.array(listData[0]['elpos']))
          logging.debug(numpy.size(numpy.array(listData[0]['elpos'])))
          logging.debug("Azimuth speed array hdf5:")
          logging.debug(numpy.array(listData[0]['azvel']))
          logging.debug(numpy.size(numpy.array(listData[0]['azvel'])))
          logging.debug("Elevation speed array hdf5:")
          logging.debug(numpy.array(listData[0]['elvel']))
          logging.debug(numpy.size(numpy.array(listData[0]['elvel'])))
          logging.debug("Timestamp in hdf5 is:"+str(listData[0]['timestmp']))
        flagConstantAzSample= False
        flagConstantElSample= False




        """if (len(listData[0]['pos']) == 100):
          pass
        else:
          pass
        pass
      else:
        pass"""


  
  #dataMPrev=dataMessage

  #use hdf5Write
  #time.sleep(1)

  #if (time.process_time_ns()-publish_time)/1000000>=1000:
  # ret= client.publish("JRO_topic",data)
  # publish_time = time.process_time_ns()
