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
#----------------------------
import os
import h5py
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
azpos = 0
azspeed = 0
elpos = 0
elspeed = 0
start_variable = False

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
    context.bot.send_message(chat_id=update.effective_chat.id, text="Azimuth position is "+str(azpos))

def az_speed(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Azimuth speed is "+str(azspeed))

def el_pos(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Elevation position is "+str(elpos))

def el_speed(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Elevation speed is "+str(elspeed))

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
    global azpos
    global azspeed
    global elpos
    global elspeed
    global directoryx

    start_time = time.process_time_ns()
    RawData,AzPosition,AzSpeed,ElPosition,ElSpeed,Timestamp = getSpeedPosition(str(msg.payload.decode("utf-8")))
    #preprocessingOutliers(ElPosition)
    #preprocessingZero(ElPosition)
    #preprocessingOutliers(AzPosition)
    xx = numpy.array([[AzPosition],[AzSpeed],[ElPosition],[ElSpeed]], dtype=object)
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
        logging.debug(numpy.array(AzPosition))
        logging.debug(numpy.size(numpy.array(AzPosition)))
        logging.debug("Elevation position array:")
        logging.debug(numpy.array(ElPosition))
        logging.debug(numpy.size(numpy.array(ElPosition)))
        logging.debug("Azimuth speed array:")
        logging.debug(numpy.array(AzSpeed))
        logging.debug(numpy.size(numpy.array(AzSpeed)))
        logging.debug("Elevation speed array:")
        logging.debug(numpy.array(ElSpeed))
        logging.debug(numpy.size(numpy.array(ElSpeed)))
        logging.debug("Timestamp is:"+str(Timestamp))


    epoc= int(Timestamp)
    meta='PE'
    epoch_time = int(epoc)
    #print(type(epoch_time))
    time_val = time.localtime(int(epoc))
    ext=".hdf5"
    filex="%s%4.4d%3.3d%10.4d%s"%(meta,time_val.tm_year,time_val.tm_yday,epoch_time,ext)
    #filename = os.path.join(os.sep, "C:" + os.sep, wpath ) #for windows
    filename =os.path.join(fullpath,directoryx,'HDF5',filex)
    print(filename)

    azi_pos = AzPosition
    ele_pos = ElPosition
    azi_vel = AzSpeed
    ele_vel = ElSpeed
    #new function for constant samples...
    setSample = 100
    data_azi_pos = numpy.array(azi_pos)
    data_ele_pos = numpy.array(ele_pos)
    data_azi_vel = numpy.array(azi_vel)
    data_ele_vel = numpy.array(ele_vel)
    
    #if (data_azi_pos.size()==100):
      

    with h5py.File(filename,'w') as fp:
      #print("Escribiendo HDF5...",epoc)
      #·················· Data·....······································  
      grp = fp.create_group("Data")
      dset = grp.create_dataset("azi_pos"  , data=data_azi_pos)
      dset = grp.create_dataset("ele_pos"  , data=data_ele_pos)
      dset = grp.create_dataset("azi_vel"  , data=data_azi_vel)
      dset = grp.create_dataset("ele_vel"  , data=data_ele_vel)
      dset = grp.create_dataset("utc"      , data=numpy.array(int(epoc)))
      
    azpos = AzPosition[-1]
    azspeed = AzSpeed[-1]
    elpos = ElPosition[-1]
    elspeed = ElSpeed[-1]

def on_publish(client,userdata,result):             #create function for callback
    print("data published \n")
    pass

client = mqtt.Client()
client.on_connect = on_connect
client.on_disconnect  = on_disconnect 
client.on_message = on_message
client.on_publish = on_publish

client.connect(broker, port)
client.subscribe("JRO_topic")
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
while True:
  time.sleep(1)
  #if (time.process_time_ns()-publish_time)/1000000>=1000:
  # ret= client.publish("JRO_topic",data)
  # publish_time = time.process_time_ns()
