#ifndef Pedestal_h
#define Pedestal_h

#define CIRCULAR_BUFFER_INT_SAFE

#include <climits>
#include "Arduino.h"
#include <Crc16.h>
#include <CircularBuffer.h>

#ifndef PEDESTAL_MAX_BUFFER_SIZE
#define PEDESTAL_MAX_BUFFER_SIZE 8192
#endif

#ifndef DEBUG_MODE
#define DEBUG_MODE 0
#endif

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

#define MODE_STOP 0x00
#define MODE_SPEED 0x01
#define MODE_POSITION 0x02
#define RX_AZIMUTH 0x27
#define RX_ELEVATION 0x47
#define TX_AZIMUTH 0x25
#define TX_ELEVATION 0x45
#define NONE_AXIS 0x65
#define RX_FUNCTION 0x30
#define TX_FUNCTION 0x40
#define HEADER 0x7E

#ifndef REQ_SCAN
#define REQ_SCAN 2
#endif

#define OPMODE_SCAN REQ_SCAN

#ifndef REQ_TABLE
#define REQ_TABLE 3
#endif

#define OPMODE_TABLE REQ_TABLE

#ifndef REQ_CALIBRATION
#define REQ_CALIBRATION 6
#endif

#define OPMODE_CALIBRATION REQ_CALIBRATION

#ifndef REQ_STOP
#define REQ_STOP 12
#endif

#define OPMODE_STOP REQ_STOP

#define OPMODE_IDLE 0

#define MIN_SPEED -180.0
#define MAX_SPEED 180.0

template<typename T, size_t S>
class Pedestal
{
  public:
    
    // Simulation variables
    double simAzPosition;
    double simElPosition;
    double simAzSpeed;
    double simElSpeed;
    // Test
    uint16_t _max_uart_buffer_size; // Maximum size of the UART RX buffer to deal inside the handle() member.
    uint16_t _max_read_iterations; // Maximum iterations done inside the getLastParameter() member.
    
    Pedestal();
    Pedestal(HardwareSerial&);
    bool initialize();
    bool freeBuffer();
    bool commandTX(uint8_t, uint8_t, double*);
    bool commandTX(uint8_t, uint8_t, double*, uint8_t, HardwareSerial&);
    bool setAxisSpeed(uint8_t, double*);
    bool setAxisPosition(uint8_t, double*);
    bool handle();
    bool setBufferSize();
    bool setScanMode(uint8_t, double, double, double, double);
    bool setTableMode(uint8_t, double, float*, size_t);
    bool setStopMode();
    uint16_t getBufferIndex();
    size_t getBufferSize();
    uint8_t getOpMode();
    uint8_t getOpAxis();
    uint8_t getOpSpeed();
    double getAzSpeed();
    double getElSpeed();
    double getAzPosition();
    double getElPosition();
    uint8_t popBuffer();
    uint8_t shiftBuffer();
    bool pushBuffer(uint8_t);
  uint8_t getBufferData(uint16_t);
    ~Pedestal();
    
  private:

    // Buffer
    size_t _bufferSize;
    CircularBuffer<T,S> _buffer;
    // UART
    HardwareSerial* _serial;
    // Operation mode variables
    uint8_t _opMode;
    uint8_t _opPrev;
    bool _opChange;
    size_t _opTableSize;
    float* _opTable;
    uint16_t _opTableIndex;
    uint8_t _opAxis;
    double _opSpeed;
    double _opStep;
    double _opBottom;
    double _opTop;
    bool _opAxisOverflow;
    double _elPrev;
    double _azPrev;
    // Variables got from pedestal
    double _azSpeed;
    double _elSpeed;
    double _azPosition;
    double _elPosition;
    // Read the internal buffer and get the last positions for each axis
    bool getLastParameter();
};

template<typename T, size_t S>
Pedestal<T,S>::Pedestal()
{
  this->_serial = Serial6;
  this->_bufferSize = 0;
}

template<typename T, size_t S>
Pedestal<T,S>::Pedestal(HardwareSerial& serial)
{
  this->_serial = &serial;
  this->_bufferSize = 0;
}

template<typename T, size_t S>
bool Pedestal<T,S>::initialize()
{
  setBufferSize();
  this->_serial->begin(BAUDRATE);
  this->_serial->flush();
  double* value;
  *value = 0;
  //if(!commandTX(MODE_STOP, TX_ELEVATION, value))
    //return false;
  //if(!commandTX(MODE_STOP, TX_AZIMUTH, value))
    //return false;
  this->_opMode = OPMODE_IDLE;
  this->_opChange = false;
  this->_opPrev = OPMODE_IDLE;
  this->_opSpeed = 0;
  this->_opBottom = 0;
  this->_opTop = 0;
  this->_opAxis = 0;
  this->_opTableSize = 0;
  this->_opTableIndex = 0;
  this->_max_uart_buffer_size = 0;
  this->_max_read_iterations = 0;
  this->_opAxisOverflow = false;
  this->_elPrev = 0;
  this->_azPrev = 0;
  this->_elPosition = 0;
  this->_azPosition = 0;
  simAzPosition = 0;
  simElPosition = 0;
  simAzSpeed = 0;
  simElSpeed = 0;
  delay(10);
  if(!this->setStopMode())
  {
    Serial.println("Error setting stop mode.");
    return false;
  }
  double temp_temp = 0;
  delay(10);
  if(!this->setAxisPosition(TX_ELEVATION, (double*)(&temp_temp)))
  {
    Serial.println("Error setting elevation axis.");
    return false;
  }
  delay(10);
  if(!this->setAxisPosition(TX_AZIMUTH, (double*)(&temp_temp)))
  {
    Serial.println("Error setting azimuth axis.");
    return false;
  }
  return true;
}
    
template<typename T, size_t S>
bool Pedestal<T,S>::freeBuffer()
{
  this->_buffer.clear();
  return true;
}
    
template<typename T, size_t S>
bool Pedestal<T,S>::commandTX(uint8_t mode, uint8_t axis, double* value, uint8_t function, HardwareSerial& serialCom)
{
  Crc16 crc;
  double temp_value = 0;
  uint8_t msb_value = 0;
  uint8_t lsb_value = 0;
  uint8_t package_size; // Minimum size of the package is 11 bytes without stuffing bytes
  uint8_t skip_flag = 0; // To skip bytes on the array
  
  if (function==TX_FUNCTION) {
    package_size = 11;
    //Serial.println("TX DATA");
  } else if (function==RX_FUNCTION) {
    package_size = 13;
    //Serial.println("RX DATA");
  } else
    return false;

  crc.clearCrc();
  crc.updateCrc(function);
  crc.updateCrc(axis);
  crc.updateCrc(mode);
  
  if (mode == MODE_SPEED) // Getting the two byte speed value
  {
    *value = constrain((*value),(double)MIN_SPEED,(double)MAX_SPEED);
    temp_value = ((*value) - ((double)MIN_SPEED)) * (((double)SHRT_MAX) - ((double)SHRT_MIN)) / (((double)MAX_SPEED) - ((double)MIN_SPEED)) + ((double)SHRT_MIN); // Mapping the speed value on the required range
    short integer_value = (int)ceil(temp_value);
    msb_value = (integer_value & 0xFF00)>>8;
    lsb_value = (integer_value & 0x00FF);
    crc.updateCrc(0x00);
    crc.updateCrc(0x00);
    crc.updateCrc(lsb_value);
    crc.updateCrc(msb_value);
    skip_flag = 2; // To skip bytes on the array
    Serial.println("Speed mode selected: "+String(*value)+" -> "+String(integer_value)+" = 0x"+String(integer_value,HEX));
  }
  else if (mode == MODE_POSITION) // Getting the two byte position value
  {
    *value = fmod((*value),360.0); // limiting value between 0 and 360
    if (*value<0) // converting value to a positive number
      *value = (*value)+360.0;
    if ((axis==TX_ELEVATION)||(axis==RX_ELEVATION)) // Validating range for elevation axis
      *value = constrain(*value,0.0,180.0);
    temp_value = (*value)*((double)USHRT_MAX)/360.0;
    uint16_t integer_value = (uint16_t)floor(temp_value);
    msb_value = (integer_value & 0xFF00)>>8;
    lsb_value = (integer_value & 0x00FF);
    crc.updateCrc(lsb_value);
    crc.updateCrc(msb_value);
    crc.updateCrc(0x00);
    crc.updateCrc(0x00);
    Serial.println("Position mode selected: "+String(*value)+" -> "+String(integer_value)+" = 0x"+String(integer_value,HEX));
  }
  else if (mode == MODE_STOP) // The value for the stop mode is 0x0000
  {
    msb_value = 0x00;
    lsb_value = 0x00;
    crc.updateCrc(0x00);
    crc.updateCrc(0x00);
    crc.updateCrc(0x00);
    crc.updateCrc(0x00);
    Serial.println("Stop mode selected.");
  }
  else // Incorrect mode
    return false;
  if ((msb_value==HEADER)||(msb_value==HEADER-1)) // Validating byte stuffing on the MSB to get the buffer size
    package_size++;
  if ((lsb_value==HEADER)||(lsb_value==HEADER-1)) // Validating byte stuffing on the LSB to get the buffer size
    package_size++;
  
  uint8_t package[package_size];
  package[0] = HEADER;
  package[1] = function;
  package[2] = axis;
  package[3] = mode;
  package[4] = 0x00;
  package[5] = 0x00;
  package[6] = 0x00;
  package[7] = 0x00;
  
  if ((lsb_value==HEADER)||(lsb_value==HEADER-1))
  {
    package[4+skip_flag] = HEADER-1;
    package[5+skip_flag] = lsb_value&0x0F|0x50;
    skip_flag++;
    //Serial.println("Stuffing byte on the LSB.");
  }
  else
    package[4+skip_flag] = lsb_value;
  if ((msb_value==HEADER)||(msb_value==HEADER-1))
  {
    package[5+skip_flag] = HEADER-1;
    package[6+skip_flag] = msb_value&0x0F|0x50;
    //Serial.println("Stuffing byte on the MSB.");
  }
  else
    package[5+skip_flag] = msb_value;
  
  if (mode==MODE_POSITION)
  {
    package[package_size-5] = 0x00;
    package[package_size-4] = 0x00;
  }
  
  if (function==TX_FUNCTION) {
    package[package_size-3] = (uint8_t)((crc.getCrc()&0xFF00)>>8); //CRC
    package[package_size-2] = (uint8_t)(crc.getCrc()&0x00FF); //CRC
    package[package_size-1] = HEADER;
  } else if (function==RX_FUNCTION) {
    package[package_size-5] = 0x00; //status
    package[package_size-4] = 0x00; //status
    package[package_size-3] = (uint8_t)((crc.getCrc()&0xFF00)>>8); //CRC
    package[package_size-2] = (uint8_t)(crc.getCrc()&0x00FF); //CRC
    package[package_size-1] = HEADER;
  } else
    return false;
  Serial.println("The package is:");
  for(int i=0; i<package_size; i++)
  {
    serialCom.write(package[i]);
    
    if(i<10)
      Serial.print("package[0"+String(i)+"] = 0x");
    else
      Serial.print("package["+String(i)+"] = 0x");
    if(package[i]<16)
      Serial.print("0");
    Serial.println(package[i],HEX);
    
  }
  Serial.println();
  
  return true;
}

template<typename T, size_t S>
bool Pedestal<T,S>::commandTX(uint8_t mode, uint8_t axis, double* value)
{
  return this->commandTX(mode,axis,value,TX_FUNCTION,*this->_serial);
}

template<typename T, size_t S>
bool Pedestal<T,S>::setAxisSpeed(uint8_t axis, double* value)
{
  if(this->_opMode==OPMODE_IDLE)
    if(!this->commandTX(MODE_SPEED, axis, value))
      return false;
  else
    return false;
  if (axis==TX_ELEVATION)
    simElSpeed = *value;
  if (axis==TX_AZIMUTH)
    simAzSpeed = *value;
  return true;
}

template<typename T, size_t S>
bool Pedestal<T,S>::setAxisPosition(uint8_t axis, double* value)
{
  //if(this->_opMode==OPMODE_IDLE || this->_opMode==OPMODE_STOP)
    if(!this->commandTX(MODE_POSITION, axis, value))
    {
      Serial.println("Error sending position command.");
      return false;
    }
  /*else
  {
    Serial.println("Error sending position command.");
    return false;
  }*/
  if (axis==TX_ELEVATION)
    simElPosition = *value;
  if (axis==TX_AZIMUTH)
    simAzPosition = *value;
  return true;
}

template<typename T, size_t S>
bool Pedestal<T,S>::handle()
{
  if (!(this->getLastParameter()))
    //Serial.println("Error reading data from buffer.");
    __asm__("nop\n\t");
  //if (this->_buffer.size() >= 52) {
  if ((this->_buffer.size() >= 68)&&((this->_elPosition!=this->_elPrev)||(this->_azPosition!=this->_azPrev))) {
  // We wait at least for two consecutive samples for each axis (we could change it to fit it with the stuffing bytes)
  // we do this to fix the problem with negative speed and starting with the zero position, and also because we rely
  // on one current value and a previous one for each axis.
  // We just execute the operation mode when any of the axis has change.
  // 13 bytes each short package -> for two consecutive samples for each axis we need 52 bytes
  // 13+4 bytes each long package (with all stuffing bytes) -> for two consecutive samples for each axis we need 68 bytes
    if (this->_opChange) {
      Serial.println("Operation Mode changed.");
      //delay(10000);
      //if(!this->commandTX(MODE_SPEED, this->_opAxis, (double*)&this->_opSpeed))
        //return false;
      this->_opChange = false;
      Serial.println("Operation Mode started.");
    }
    switch (this->_opMode)
    {
      case OPMODE_SCAN:
        if (this->_opAxis == TX_AZIMUTH) {
          simAzSpeed = this->_opSpeed;
          if (((this->_azPosition < this->_azPrev)&&(this->_azSpeed >= 0))||((this->_azPosition > this->_azPrev)&&(this->_azSpeed <= 0))) {
            Serial.println("Azimuth route complete.");
            double el_send_position = this->_elPosition + this->_opStep;
            if ((el_send_position > this->_opTop) || (el_send_position < this->_opBottom))
              this->_opStep = -this->_opStep;
            el_send_position = constrain(el_send_position,this->_opBottom,this->_opTop);
            if(!this->commandTX(MODE_POSITION,TX_ELEVATION,(double*)(&el_send_position)))
              return false;
            simElPosition = el_send_position;
          }
        }
        else if (this->_opAxis == TX_ELEVATION) {
          simElSpeed = this->_opSpeed;
          if (abs(this->_elSpeed-this->_opSpeed)<=5.0) {
            if ((this->_elPosition >= 179.5)&&(this->_elSpeed >= 0)) { // Revisar
              this->_opSpeed = -(abs(this->_opSpeed));
              if(!this->commandTX(MODE_SPEED,TX_ELEVATION,(double*)(&this->_opSpeed)))
                return false;
              simElSpeed = this->_opSpeed;
            }
            else if (((this->_elPosition <= 0.05)&&(this->_elSpeed <= 0))||((this->_elPosition > this->_elPrev)&&(this->_elSpeed <= 0))) { // Revisar
              Serial.println("Elevation route complete.");
              this->_opSpeed = abs(this->_opSpeed);
              if(!this->commandTX(MODE_SPEED,TX_ELEVATION,(double*)(&this->_opSpeed)))
                return false;
              simElSpeed = this->_opSpeed;
              double az_send_position = this->_azPosition + this->_opStep;
              if ((az_send_position > this->_opTop) || (az_send_position < this->_opBottom))
                this->_opStep = -this->_opStep;
              az_send_position = constrain(az_send_position,this->_opBottom,this->_opTop);
              if(!this->commandTX(MODE_POSITION,TX_AZIMUTH,(double*)(&az_send_position)))
                return false;
              simAzPosition = az_send_position;
            }
          }
        }
        else
          Serial.println("Wrong requested axis.");
        break;
      case OPMODE_TABLE:
        if (this->_opAxis == TX_AZIMUTH) {
          simAzSpeed = this->_opSpeed;
          if (((this->_azPosition < this->_azPrev)&&(this->_azSpeed >= 0))||((this->_azPosition > this->_azPrev)&&(this->_azSpeed <= 0))) {
            Serial.println("Azimuth route complete.");
            if (this->_opTableIndex >= this->_opTableSize)
              this->_opTableIndex=0;
            double el_send_position = (double)(*(this->_opTable+this->_opTableIndex));
            if(!this->commandTX(MODE_POSITION,TX_ELEVATION,(double*)(&el_send_position)))
              return false;
            simElPosition = el_send_position;
            this->_opTableIndex++;
          }
        }
        else if (this->_opAxis == TX_ELEVATION) {
          simElSpeed = this->_opSpeed;
          if (abs(this->_elSpeed-this->_opSpeed)<=5) {
            if ((this->_elPosition >= 179.5)&&(this->_elSpeed >= 0)) { // Revisar
              this->_opSpeed = -(abs(this->_opSpeed));
              if(!this->commandTX(MODE_SPEED,TX_ELEVATION,(double*)(&this->_opSpeed)))
                return false;
              simElSpeed = this->_opSpeed;
            }
            else if (((this->_elPosition <= 0.05)&&(this->_elSpeed <= 0))||((this->_elPosition > this->_elPrev)&&(this->_elSpeed <= 0))) { // Revisar
              Serial.println("Elevation route complete.");
              this->_opSpeed = abs(this->_opSpeed);
              if(!this->commandTX(MODE_SPEED,TX_ELEVATION,(double*)(&this->_opSpeed)))
                return false;
              simElSpeed = this->_opSpeed;
              if (this->_opTableIndex >= this->_opTableSize)
                this->_opTableIndex=0; //It also has to be initialized on the function "table()"
              double az_send_position = (double)(*(this->_opTable+this->_opTableIndex));
              if(!this->commandTX(MODE_POSITION,TX_AZIMUTH,(double*)(&az_send_position)))
                return false;
              simAzPosition = az_send_position;
              this->_opTableIndex++;
            }
          }
        }
        else
          Serial.println("Wrong requested axis.");
        break;
      case OPMODE_STOP:
        break;
      case OPMODE_IDLE:
        break;
      case OPMODE_CALIBRATION:
        break;
    }
    this->_elPrev = this->_elPosition;
    this->_azPrev = this->_azPosition;
  }
  
  if (this->_serial->available()>this->_max_uart_buffer_size)
  {
    this->_max_uart_buffer_size = this->_serial->available();
  }
  while (this->_serial->available()) {
    // get the new byte:
    this->_buffer.push((uint8_t)this->_serial->read());
  }
  return true;
}

template<typename T, size_t S>
bool Pedestal<T,S>::setBufferSize()
{
  this->freeBuffer();
  this->_bufferSize = this->_buffer.capacity;
  return true;
}

template<typename T, size_t S>
bool Pedestal<T,S>::setScanMode(uint8_t opAxis, double opSpeed, double opStep, double opBottom, double opTop)
{
  opSpeed = constrain(opSpeed,MIN_SPEED,MAX_SPEED);
  double value1 = 0.0;
  double value2 = 180.0;
  if (opBottom>opTop)
    return false;
  if (opAxis!=TX_AZIMUTH && opAxis!=TX_ELEVATION)
    return false;
  this->_opBottom = opBottom;
  //if(!this->commandTX(MODE_POSITION, (opAxis&0x0F)|((~opAxis)&0b01100000), (double*)&opBottom))
    //return false;
  //delay(10);
  // For the elevation axis we have to set the position to 0° if the speed is positive
  // and to 180° if the speed is negative. For azimuth this is not necessary.
  if (opAxis==TX_ELEVATION) {
    if (opSpeed>=0) {
      //if(!this->commandTX(MODE_POSITION, opAxis, (double*)&value1)) // set to 0°
        //return false;
      simElPosition = value1;
    } else {
      //if(!this->commandTX(MODE_POSITION, opAxis, (double*)&value2)) // set to 180°
        //return false;
      simElPosition = value2;
    }
    simElSpeed = opSpeed;
    simAzPosition = opBottom;    
  }
  if (opAxis==TX_AZIMUTH) {
    //if(!this->commandTX(MODE_POSITION, opAxis, (double*)&value1)) // set to 0°
      //return false;
    simAzPosition = value1;
    simAzSpeed = opSpeed;
    simElPosition = opBottom;
  }
  //delay(10);
  
  if(!this->commandTX(MODE_SPEED, opAxis, (double*)&opSpeed))
    return false;
  
  this->_opMode = OPMODE_SCAN;
  this->_opAxis = opAxis;
  this->_opSpeed = opSpeed;
  this->_opStep = opStep;
  this->_opBottom = opBottom;
  this->_opTop = opTop;
  this->_opChange = true;
  
  return true;
}

template<typename T, size_t S>
bool Pedestal<T,S>::setTableMode(uint8_t opAxis, double opSpeed, float* opTable, size_t len)
{
  opSpeed = constrain(opSpeed,MIN_SPEED,MAX_SPEED);
  double value1 = 0.0;
  double value2 = 180.0;
  if (opAxis!=TX_AZIMUTH&&opAxis!=TX_ELEVATION)
    return false;
  Serial.println("First table value: "+String(*this->_opTable));
  this->_opTable = opTable;
  double temp_double = (double)(*this->_opTable);
  //if(!this->commandTX(MODE_POSITION, (opAxis&0x0F)|((~opAxis)&0b01100000), (double*)&temp_double))
    //return false;
  //delay(10);
  // For the elevation axis we have to set the position to 0° if the speed is positive
  // and to 180° if the speed is negative. For azimuth this is not necessary.
  if (opAxis==TX_ELEVATION) {
    if (opSpeed>=0) {
      //if(!this->commandTX(MODE_POSITION, opAxis, (double*)&value1)) // set to 0°
        //return false;
      simElPosition = value1;
    } else {
      //if(!this->commandTX(MODE_POSITION, opAxis, (double*)&value2)) // set to 180°
        //return false;
      simElPosition = value2;
    }
    simElSpeed = opSpeed;
    simAzPosition = *opTable;
  }
  if (opAxis==TX_AZIMUTH) {
    //if(!this->commandTX(MODE_POSITION, opAxis, (double*)&value1)) // set to 0°
      //return false;
    simAzPosition = value1;
    simAzSpeed = opSpeed;
    simElPosition = *opTable;
  }
  //delay(10);
  
  if(!this->commandTX(MODE_SPEED, opAxis, (double*)&opSpeed))
    return false;
  
  this->_opMode = OPMODE_TABLE;
  this->_opAxis = opAxis;
  this->_opSpeed = opSpeed;
  this->_opTable = opTable;
  this->_opTableSize = len;
  this->_opTableIndex = 1;
  this->_opChange = true;
  
  return true;
}

template<typename T, size_t S>
bool Pedestal<T,S>::setStopMode()
{
  double* value;
  *value = 0;
  delay(10);
  if(!this->commandTX(MODE_STOP, TX_ELEVATION, value))
    return false;
  delay(10);
  if(!this->commandTX(MODE_STOP, TX_AZIMUTH, value))
    return false;
  delay(10);
  this->_opMode = OPMODE_STOP;
  this->_opAxis = NONE_AXIS;
  this->_opSpeed = 0;
  simAzSpeed = 0;
  simElSpeed = 0;
  return true;
}

template<typename T, size_t S>
uint16_t Pedestal<T,S>::getBufferIndex()
{
  return this->_buffer.size()-1;
}

template<typename T, size_t S>
size_t Pedestal<T,S>::getBufferSize()
{
  //this->_bufferSize = this->_buffer.size();
  return this->_buffer.size();
}

template<typename T, size_t S>
uint8_t Pedestal<T,S>::getOpMode()
{
  return this->_opMode;
}

template<typename T, size_t S>
uint8_t Pedestal<T,S>::getOpAxis()
{
  return this->_opAxis;
}

template<typename T, size_t S>
uint8_t Pedestal<T,S>::getOpSpeed()
{
  return this->_opSpeed;
}

template<typename T, size_t S>
double Pedestal<T,S>::getAzSpeed()
{
  return this->_azSpeed;
}

template<typename T, size_t S>
double Pedestal<T,S>::getElSpeed()
{
  return this->_elSpeed;
}

template<typename T, size_t S>
double Pedestal<T,S>::getAzPosition()
{
  return this->_azPosition;
}

template<typename T, size_t S>
double Pedestal<T,S>::getElPosition()
{
  return this->_elPosition;
}

template<typename T, size_t S>
uint8_t Pedestal<T,S>::popBuffer()
{
  return this->_buffer.pop();
}

template<typename T, size_t S>
uint8_t Pedestal<T,S>::shiftBuffer()
{
  return this->_buffer.shift();
}

template<typename T, size_t S>
bool Pedestal<T,S>::pushBuffer(uint8_t value)
{
  this->_buffer.push(value);
  return true;
}

template<typename T, size_t S>
uint8_t Pedestal<T,S>::getBufferData(uint16_t index)
{
  return this->_buffer[index];
}

template<typename T, size_t S>
bool Pedestal<T,S>::getLastParameter()
{
  Crc16 crc;
  
  if (this->_buffer.size()<26) {// There has to be at least two packages on the buffer
    //Serial.println("Buffer too small to read.");
    return false;
  }
  uint16_t iterations = 0;
  uint8_t package_counter = 0; // We have to process two consecutive packages
  for (int i=13; i<=this->_buffer.size(); i++) // It will begin with the last 13 bytes of the buffer and it will go backwards after that
  {
    if (this->_buffer[this->_buffer.size()-i] == HEADER) // Validate first flag byte
    {
      if (this->_buffer[this->_buffer.size()-i+1] == RX_FUNCTION) // Validate function byte (rx or tx)
      {
        if ((this->_buffer[this->_buffer.size()-i+2] == RX_AZIMUTH)||(this->_buffer[this->_buffer.size()-i+2] == RX_ELEVATION)) // Validate axis
        {
          crc.clearCrc();
          crc.updateCrc(this->_buffer[this->_buffer.size()-i+1]); // CRC function
          crc.updateCrc(this->_buffer[this->_buffer.size()-i+2]); // CRC axis
          crc.updateCrc(this->_buffer[this->_buffer.size()-i+3]); // CRC mode
          uint8_t hadstuffing = 0; // To know if the first byte had a stuffing byte
          uint16_t temp_position = 0;
          int16_t temp_speed = 0;
          
          // We first get the position values
          if (this->_buffer[this->_buffer.size()-i+4+hadstuffing] == (HEADER-1)) // Validate stuffing byte for the first byte of position
          {
            temp_position = 0x70|(this->_buffer[this->_buffer.size()-i+5+hadstuffing]&0x0F);
            hadstuffing++;
          }
          else // No stuffing byte
            temp_position = this->_buffer[this->_buffer.size()-i+4+hadstuffing];
          if (this->_buffer[this->_buffer.size()-i+5+hadstuffing] == (HEADER-1)) // Validate stuffing byte for the second byte of position
          {
            temp_position = temp_position|((0x70|(this->_buffer[this->_buffer.size()-i+6+hadstuffing]&0x0F))<<8);
            hadstuffing++;
          }
          else // No stuffing byte
            temp_position = temp_position | (this->_buffer[this->_buffer.size()-i+5+hadstuffing]<<8);
          crc.updateCrc((uint8_t)(temp_position&0x00FF));
          crc.updateCrc((uint8_t)((temp_position&0xFF00)>>8));
          
          // After the position value we get the speed value
          if (this->_buffer[this->_buffer.size()-i+6+hadstuffing] == (HEADER-1)) // Validate stuffing byte for the first byte of speed
          {
            temp_speed = 0x70|(this->_buffer[this->_buffer.size()-i+7+hadstuffing]&0x0F);
            hadstuffing++;
          }
          else // No stuffing byte
            temp_speed = this->_buffer[this->_buffer.size()-i+6+hadstuffing];
          if (this->_buffer[this->_buffer.size()-i+7+hadstuffing] == (HEADER-1)) // Validate stuffing byte for the second byte of speed
          {
            temp_speed = temp_speed|((0x70|(this->_buffer[this->_buffer.size()-i+8+hadstuffing]&0x0F))<<8);
            hadstuffing++;
          }
          else // No stuffing byte
            temp_speed = temp_speed | (this->_buffer[this->_buffer.size()-i+7+hadstuffing]<<8);
          crc.updateCrc((uint8_t)(temp_speed&0x00FF));
          crc.updateCrc((uint8_t)((temp_speed&0xFF00)>>8));
      
      // assign speed and position to the correct axis
          if (this->_buffer[this->_buffer.size()-i+2] == RX_AZIMUTH) { // Azimuth axis
            this->_azPosition = fmod(((double)temp_position)*(360.0/((double)USHRT_MAX)),360.0);
            this->_azSpeed = ((double)temp_speed - (double)SHRT_MIN) * ((double)MAX_SPEED - (double)MIN_SPEED) / ((double)SHRT_MAX - (double)SHRT_MIN) + (double)MIN_SPEED;
          } else if (this->_buffer[this->_buffer.size()-i+2] == RX_ELEVATION) { // Elevation axis
            this->_elPosition = fmod(((double)temp_position)*(360.0/((double)USHRT_MAX)),360.0);
            this->_elSpeed = ((double)temp_speed - (double)SHRT_MIN) * ((double)MAX_SPEED - (double)MIN_SPEED) / ((double)SHRT_MAX - (double)SHRT_MIN) + (double)MIN_SPEED;
            if (this->_elPosition>270)
              this->_elPosition = this->_elPosition - 360.0;
          } else {// There is an error on the package!
            return false;
          }
          package_counter++;
          if (package_counter==2)
            return true;
          i = i + 12 + hadstuffing; // Skip right to the next package to save time. (remember that the for loop always add one at the end)
          if(this->_buffer.size()-i<=0) {
            //Serial.println("Index exceed limits.");
            return false;
          }
        }
      }
    }
    iterations++;
    if(this->_max_read_iterations < iterations) // to analize the time consume
      this->_max_read_iterations=iterations;
  }
  //Serial.println("Didn't found any pedestal data on buffer.");
  return false;
}

template<typename T, size_t S>
Pedestal<T,S>::~Pedestal()
{
  free(this->_opTable);
}

int CharArrayToFloatArray(float* floatarray, const char *buf, size_t len)
{
  uint32_t* temp;
  float* float_number;
  Serial.println("Start conversion");
  Serial.print("Buffer size is ");
  Serial.println(len);
  for (int i=0; i<(len/4); i++)
  {
    *temp = (uint32_t)((*(buf+4*i)<<24)|(*(buf+4*i+1)<<16)|(*(buf+4*i+2)<<8)|(*(buf+4*i+3)));
    Serial.println(*temp,HEX);
    memcpy(floatarray+i,(float*)temp,sizeof(temp));
    Serial.println(*(floatarray+i));
  }
  Serial.println("End conversion");
  return 1;
}

#endif
