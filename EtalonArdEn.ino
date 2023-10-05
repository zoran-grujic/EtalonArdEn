/* Zoran Grujić 2019
 * Realization of third harmonic lockin by Arduino DUE
 * Made for HeNe I2 etalon of laser frequency 
 */

#include <Arduino.h>
#include <math.h>

const char CONTROLLERNAME[] =  "Etalon lock-in";

const word pinSignalIn = A0;
const word pinDitherOut = DAC0;
const word pinFineScanOut = DAC1;

const word pinTemperature = A2;                 //temperature sensing
const word pinLaserPower = A1;                  //PD DC out

const word pinPWMOut = 7;    //(6-9)
const int maxres = 4095;
const float mu_2PI = 2.0*M_PI/1000000.0;        //save computation time, pre-calculate a constant
const int datablock = 100;
//Dither frequency and sampling
#define frequencyDither 1000000/(24*32)         //1302.08 // 1.3 kHz Dither
const int sampleTime_us = 32;                   //lock-in AD/DA period 32 us
const int sinePoints = 24;                      // frequency 1.3kHz, in steps of 32us has 24 samples
const int decimation  = 35;                     //points to skip
int deciNext=0;    
                             //
//average = alpha * raw + (1-alpha) * average;    //exponential smoothing
const float alpha = 0.002;
const float alphaM1 = 1- alpha;                  // pre-calculate to reduce future calculations
const int upfactor = 65536;                      // good to be 2^x then use shif to divide
const int alpha16 = int(upfactor * alpha);       //   65;
const int alpha16M1 = upfactor-alpha16;          // 65471; //65536 - 65
//average fast scan
const float alpha100 = 0.5;
const float alpha100M1 = 1-alpha100;

//average PD DC
const float alphaPD = 0.01;
const float alphaPDM1 = 1-alphaPD;
//average LOCK
const float alphaLOCK = 0.001;
const float alphaLOCKM1 = 1-alphaLOCK;

float averagePD=0;
float averageX=0;
float averageY=0;
float averageX2=0;
float averageY2=0;
float averageXLOCK = 0;

float DCFree=0;                                   //Average value of the PD signal


int average16=0;
int averageRem = 0;                               //use to preserve precision after division in average rutine
int temp = 0;                                     // temporary int variable 
#define pinDACselect 4                            // SPI device select pin
word DACmax=256*256-1;                            //2^16-1
word DACmin=0;
byte spi[3];                                      // buffer for SPI

#include <SPI.h>                                  //MUST be included before DueTimer, SPI library
#include <DueTimer.h>                             // Timer library to periodically trigger acquisition function


//Global variables

char operationMode[5] = "scan";                   // scan or lock
int scanStart = 0;
int scanStop = 4095;                              //2^12-1
int scanPos = 0;                                  // HV mirror position of next measurement, it is sent as 16bit value over SPI to DAC 2 Click
int fineScanPos = 2048;                           // fine scan over DAC1 out
int scanPosLast = scanPos;                        // gde je izvrseno poslednje merenje

unsigned long lastStreamTime_ms = 0;
int streamPeriod_ms = 10;                         //send data every streamPeriod_ms
unsigned long time_ms;                            //timestamp from milis()
unsigned long lockStartTime=0;                    //timestamp lock started


float mu_omega = frequencyDither * mu_2PI;        // angula frequency in us
int phaseDEG = 90;
float phaseRAD = 1.570796;
int next=0;                                       //next sample number

//PI parameters

int integralPI = 1000;                            //Integral constant 1/s
int propPI = 1000;                                //Proportional constant 
int fineHVratio = 100;                            //Ratio between OC and HV steps
int lockPointX = 0;
int lockPointPI = 0;
float integral=2048;                              //Error integral += integralPI * error * dt

unsigned int temperature=0;                       //ADC 
unsigned int laserPower = 0;//ADC

int out;
int in;
int t_us, t2_us, a0;
int i;

const byte numChars = 32;
char serialChars[numChars];                     // an array to store the received data
char tempChars[numChars];                       // temporary array for use when parsing
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
int integer2FromPC = 0;


bool ledOn = true;                              // LED state  On or OFF
int ledCount = 0;
int skipCount = 0;


int ditherData[sinePoints];                     
float ditherXData[sinePoints];
float dither3XData[sinePoints];
float ditherYData[sinePoints];
float dither3YData[sinePoints];

int ins[datablock];
int inscpy[datablock];
int ts_us[datablock];

float lockXHist[datablock];
float lockYHist[datablock];


//Butterworth filter 2nd order, 0.03 fN lowpass
//a =  [ 1.         -1.86689228  0.87521455]
//b =  [0.00208057 0.00416113 0.00208057]
float a2[]={1.,-1.86689228,0.87521455};
float b2[]={0.00208057, 0.00416113, 0.00208057};

float x[]={0,0,0};                       // storage for filter
float y[]={0,0,0};
float xY[]={0,0,0};
float yY[]={0,0,0};

float xd[]={0,0,0};
float yd[]={0,0,0};
float xYd[]={0,0,0};
float yYd[]={0,0,0};

//declarations of functions
void golock();
void serialReadLine();
void parse3Data();
void parse2Data();
void getCommand();
void DACwrite(word out);
void makeSinewave();
void lockit();

void IIRfilter2(float in, float X[], float Y[], float A[], float B[]);
float syncFilter(float in[], int next, int deltaI=12, int inLen=24);
float syncFilter(int in[], int next, int deltaI=12, int inLen=24);
void center();

// copy 
void copy(int* src, int* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
}

void setup() {
  // put your setup code here, to run once:

  //pin functions
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(pinDitherOut, OUTPUT);                // dither 1kHz 
  pinMode(pinFineScanOut, OUTPUT);              // fine scan out
  pinMode(pinDACselect, OUTPUT);                //CS select SPI pin for DAC 2 Click
  pinMode(pinPWMOut, OUTPUT);
  pinMode(13, OUTPUT);                          // LED pin
  digitalWrite(13, LOW);                        // LED off
  analogWrite(pinFineScanOut, 2048);

  pinMode(pinSignalIn, INPUT);                  // PD signal input this is THE signal
  analogRead(pinSignalIn);
  pinMode(pinTemperature, INPUT);               // temperature signal input
  analogRead(pinTemperature);
  pinMode(pinLaserPower, INPUT);                // laser power input
  analogRead(pinLaserPower);
  
 

  // read A0 pin at maximal rate  with no break
  //https://forum.arduino.cc/index.php?topic=137635.0
  ADC->ADC_MR |= 0x80;                          // these lines set free running mode on adc 7 (pin A0)
  ADC->ADC_CR=2;                                //aktivira ADC
  //enable A0, A1, A2
  //http://www.atmel.com/Images/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf
  ADC->ADC_CHER=0x80;                           //0xE0;//0x80; 
  
  SerialUSB.begin(250000);                      //Open Native USB port 
  while (!SerialUSB) ;                          //wait for the port to open
  SerialUSB.println(CONTROLLERNAME);
  SPI.begin();                                  

  SPI.begin(pinDACselect);                      //Start SPI communication
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));

  makeSinewave();                              //make sine waveform
  
  //interrupts!!!!!!!!!!!!!!!!!!
  Timer3.attachInterrupt(golock).start(sampleTime_us);
}


void loop() {

  //-----------------------------------------
  //send data to serial
  //-----------------------------------------
  //cast (unsigned long) type to handle 49 days rolover of milis() function. 2^32 ms = 49 days 17 h
  //https://www.baldengineer.com/arduino-how-do-you-reset-millis.html
  time_ms = millis();
  if ((unsigned long)(time_ms - lastStreamTime_ms) >= streamPeriod_ms) {
      lastStreamTime_ms = time_ms;

      SerialUSB.print(scanPosLast);
      SerialUSB.print(", ");
      SerialUSB.print(fineScanPos);
      SerialUSB.print(", ");
      SerialUSB.print(averageX2,5);
      //SerialUSB.print(averageX, 3);
      SerialUSB.print(", ");
      //SerialUSB.print("Y= ");
      SerialUSB.print(averageY2,5);
      //SerialUSB.print(averageY, 3);
      SerialUSB.print(", ");
      SerialUSB.println(lastStreamTime_ms);

      

      //LED blink On - Off
      if (ledCount >= 10)
      {
         
          ledCount=0;
          if(ledOn)
            digitalWrite(13, HIGH);
          else
            digitalWrite(13, LOW);
          ledOn = !ledOn;
      }
      else
        ledCount++;
          
      //ocassionally send temperature and laser power to GUI
      if(skipCount >= 50)
      {
        skipCount=0;
        //Read A1 and A2 
        //http://www.atmel.com/Images/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf
        ADC->ADC_CHER = 0xE0;                         //0x80; 
        while((ADC->ADC_ISR & 0xE0)==0);              // wait for conversion
        temperature=ADC->ADC_CDR[5];                  //A2     read data

        laserPower=ADC->ADC_CDR[6];                   //A1
        ADC->ADC_CHER = 0x80;

        SerialUSB.print("temp: ");
        SerialUSB.println(temperature);
        SerialUSB.print("power: ");
        SerialUSB.println(laserPower);
      }
      else{
        skipCount++;
      }
        


    //------------------------------
     // Check for GUI commands on Serial port
    //--------------------------------
    getCommand();

    //-------------------------------------
    // lock it
    //-------------------------------------
    lockit();

    }//end IF time to act
}//END loop()

//PI regulation
void lockit()
  {
    if(strcmp(operationMode, "lock")==0 && time_ms - lockStartTime > 50000)
    {
      //yd[0] error
      /*
      int integralPI = 1000;                            //Integral constant 1/s
      int propPI = 1000;                                //Proportional constant 
      int fineHVratio = 100;                            //Ratio between OC and HV steps
      int lockPointX = 0;
      int lockPointPI = 0;
      float integral=2048;                              //Error integral += integralPI * error * dt, current value

      */
      float error = averageX2 - lockPointPI/1000.0;
      integral +=  (error * integralPI)/1000.0;
      fineScanPos = int(integral + (error * propPI)/1000.0);

      if(scanPos<0 || scanPos>65535)
      {
        //error
        analogWrite(pinFineScanOut, 2048);
        SerialUSB.println("Error: lock out of HV range.");
        strcpy(operationMode, "scan");//stop lock, start scan
        SerialUSB.println("mode: scan");
         
        return;
      }

      if(fineScanPos<0 || fineScanPos>4095)
      {
        //error
        
        analogWrite(pinFineScanOut, 2048);
        SerialUSB.print("Error: lock out of fine range:");
        SerialUSB.print(fineScanPos);
        SerialUSB.print(" integral:");
        SerialUSB.println(integral);
        strcpy(operationMode, "scan");//stop lock, start scan
        SerialUSB.println("mode: scan");
        fineScanPos = 2048;
        integral = fineScanPos;
         
        return;
      }

      // almost out of max range! Jump towards center
      if(fineScanPos > 3500)
      {
        fineScanPos += - fineHVratio;
        integral += - fineHVratio;
        scanPos +=10;
      }

      // almost out of min range! Jump towards center
      if(fineScanPos < 500)
      {
        fineScanPos += fineHVratio;
        integral += fineHVratio;
        scanPos +=-10;
      }

      analogWrite(pinFineScanOut, fineScanPos);       // set new HR mirror position
      DACwrite(scanPos);                              // set new OC mirro position
      
    }    
  }//end lockit()

/*
 play dither signal
 calculate X and Y lock-in components
 execute every sampleTime_us, trigered interrupt by Timer3
 Sample frequency 31250 Hz
 */
void golock()
{
  
  analogWrite(pinDitherOut, ditherData[next]);

  while((ADC->ADC_ISR & 0x80)==0);        // wait for conversion
  ins[next]=ADC->ADC_CDR[7];              // read data A0

  // use exponential filter to get new mean value
  // from syncFiltered data
  averagePD = alphaPD * syncFilter(ins, next) + alphaPDM1 * averagePD;
  DCFree = ins[next] - averagePD;
  
  /*
  multiply signal by 3f reference and apply lowpass exponential filter
  in lock mode only X (in-phase) component is calculated to save time
  */  
  if(strcmp(operationMode, "lock")!=0)
  {
    //scan mode
   
    lockXHist[next] = DCFree * dither3XData[next];
    lockYHist[next] = DCFree * dither3YData[next];

    // exponential filter
    averageX = alpha * syncFilter(lockXHist, next) + alphaM1 * averageX;
    averageY = alpha * syncFilter(lockYHist, next) + alphaM1 * averageY;
  }
  else
  {
    // lock mode
    lockXHist[next] = DCFree * dither3XData[next];
    averageX = alpha * syncFilter(lockXHist, next) + alphaM1 * averageX;
  }
  
  // decimate data
  // effective sample frequency 868.055 Hz after decimation by 35
  if(deciNext>=decimation )
  {
    deciNext=0;
    //filter decimated and filtered X & Y 
    if(strcmp(operationMode, "lock")!=0)
    {
      // scan mode
      // effective cut-off frequency 26.04 Hz = 868.055 * 0.03 f_cut
      IIRfilter2(averageX, xd, yd, a2, b2);//X
      IIRfilter2(averageY, xYd, yYd, a2, b2);//Y

      averageX2=yd[0];
      averageY2=yYd[0];
    }
    else
    {
       /* lock mode */     

      IIRfilter2(averageX, xd, yd, a2, b2);         //X
      averageX2=yd[0];                              //yd[0];
    }

  }
  else
  {
    deciNext++;
  }
  

  scanPosLast = scanPos;                           // save last scan position
  next++;
  if(next >= sinePoints)
    next=0;                                       // start new dither period
  }

  void IIRfilter2(float in, float X[], float Y[], float A[], float B[])
  {
    //Butterworth filter 1. reda, 0.015 fN lowpass
    //a =  [ 1.         -1.93338023  0.9355289 ]
    //b =  [0.00053717 0.00107434 0.00053717]
    
    // shift variables
    X[2]=X[1];
    X[1]=X[0];
    X[0]= in;
    Y[2]=Y[1];
    Y[1]=Y[0];
    //filter
    Y[0]=( B[0]*X[0] + B[1]*X[1] + B[2]*X[2] - A[1]*Y[1] - A[2]*Y[2] );// A[0]=1.0 skip division by 1 to save computation time
    }

// supress dither frequency - 1st harmonic
float syncFilter(float in[], int next, int deltaI, int inLen)
{
  int deltaNext;
  //
  if((deltaNext=next-deltaI)<0)
    deltaNext = inLen + deltaNext;

  return (in[next] + in[deltaNext])/2.0;
}

// supress dither frequency - 1st harmonic
float syncFilter(int in[], int next, int deltaI, int inLen)
{
  int deltaNext;
  //
  if((deltaNext=next-deltaI)<0)
    deltaNext = inLen + deltaNext;

  return (in[next] + in[deltaNext])/2.0;
}

// read input from GUI
void serialReadLine(){
  static byte n = 0;
  char rc;
  if(SerialUSB.available()>0)
    while( (rc = SerialUSB.read())!='\n'){
      serialChars[n] = rc;
      n++;
      if (n >= numChars) {
          n = numChars - 1;
      }
    }                                     //end while
  serialChars[n] = '\0';                  // terminate the string
  n=0;
  }                                       //end void readLine(){

// split the data into its parts
void parse3Data() {                       

    char * strtokIndx;                    // this is used by strtok() as an index

    strtokIndx = strtok(tempChars," ");      // get the first part - the string
    strcpy(messageFromPC, strtokIndx);       // copy it to messageFromPC
 
    strtokIndx = strtok(NULL, " ");          // this continues where the previous call left off
    integerFromPC = atoi(strtokIndx);        // convert this part to an integer

    strtokIndx = strtok(NULL, " ");
    integer2FromPC = atoi(strtokIndx);      // convert this part to a int atof()- float

}

// split the data into its parts
void parse2Data() {                       

    char * strtokIndx;                      // this is used by strtok() as an index

    strtokIndx = strtok(tempChars," ");      // get the first part - the string
    strcpy(messageFromPC, strtokIndx);      // copy it to messageFromPC
 
    strtokIndx = strtok(NULL, " ");         // this continues where the previous call left off
    integerFromPC = atoi(strtokIndx);       // convert this part to an integer

}


void getCommand()
{
  //-----------------------------------------
  //read from serial
  //-----------------------------------------
  //commands:
  //whois? - returns controller name CONTROLLERNAME
  //

  
  if(SerialUSB.available()>0)
  {
    serialReadLine();                       //read into serialChars
    strcpy(tempChars, serialChars);         //move copy to tempChars
    parse3Data();                           //parse command and two ints from input
    
      //command scan
      if(strcmp(messageFromPC, "scan")==0){
        strcpy(operationMode, "scan");
        if(integerFromPC == integer2FromPC)
        {
          scanPos = integerFromPC;
          DACwrite(scanPos);
          integral = 2048;
          analogWrite(pinFineScanOut, integral);
          return;
        }
        else
        {
          SerialUSB.print("Error: ");
          SerialUSB.println(serialChars);   
          return;    
        }
        
      }
      //command whois?
      else if(strcmp(messageFromPC, "whois?")==0)
      {
        SerialUSB.println(CONTROLLERNAME);          //its me!
        return;
      }
      //mode command
      else if(strcmp(messageFromPC,"mode?")==0){
        SerialUSB.print("mode: ");
        SerialUSB.println(operationMode);               //report operation mode
        return;
      }
      //set phase, input is in deggree multiplied by 100
       else if(strcmp(messageFromPC,"phase")==0){
        if(integerFromPC == integer2FromPC)
        {
          phaseDEG = integerFromPC;
          phaseRAD = 2*M_PI*phaseDEG/36000;             //360*100
          makeSinewave();                               //generate new sine waves
          SerialUSB.print("Phase: ");                 
          SerialUSB.println(phaseDEG/100.0,3);          //report phase change
        }
        else
        {
          SerialUSB.print("Error: ");
          SerialUSB.println(serialChars);       
        }
        return;
        
      }
      //command moveHV
      else if(strcmp(messageFromPC,"moveHV")==0){
        if(integerFromPC == integer2FromPC)
        {
          scanPos += integerFromPC;//
        }
        else
        {
          SerialUSB.print("Error: ");
          SerialUSB.println(serialChars);       
        }
        return;
        
      }
      // command lock
      else if(strcmp(messageFromPC, "lock")==0){
        lockStartTime= micros();
        strcpy(operationMode, "lock");
        if(integerFromPC == integer2FromPC)
        {
          lockPointX = integerFromPC;
          scanPos = lockPointX;
          DACwrite(scanPos);                      //go to lock point 
          integral=2048;                          //Reset integrator
          fineScanPos = integral;
          analogWrite(pinFineScanOut, fineScanPos); //set new position for fine scan

        
          SerialUSB.print("mode: ");              //report mode and lockpoint
          SerialUSB.print(operationMode);
          SerialUSB.print(": Lok point: ");
          SerialUSB.println(integerFromPC);
          
        }
        else
        {
          SerialUSB.print("Error: ");
          SerialUSB.println(serialChars);       
        }
        return;
        
      }
      //command PI to set P & I
      else if(strcmp(messageFromPC, "PI")==0){
        integralPI = integerFromPC;
        propPI = integer2FromPC;
      }
      //command lockpoint - set offset from 0
      else if(strcmp(messageFromPC, "lockpoint")==0){
        if(integerFromPC == integer2FromPC)
        {
          lockPointPI = integerFromPC;
          SerialUSB.print("Lock point offset: ");
          SerialUSB.println(lockPointPI);
        }
        else
        {
          SerialUSB.print("Error: ");
          SerialUSB.println(serialChars);       
        }
        
      }
      //command center
      else if(strcmp(messageFromPC, "center")==0 && strcmp(operationMode, "lock")==0 ){
        center();
      }
      //command FHVratio  - set ratio between HR and OC
      else if(strcmp(messageFromPC,"FHVratio")==0){
        if(integerFromPC == integer2FromPC)
        {
          fineHVratio = integerFromPC;
        }
        else
        {
          SerialUSB.print("Error: ");
          SerialUSB.println(serialChars);       
        }
        
      }
      // command not recognised  :-(
      else{
        SerialUSB.print("Unknown command: '");
        SerialUSB.print(messageFromPC);
        SerialUSB.println("'");
        }

    }//end read from serial
  }


void center()
{
  int d;
  int deltaScan;
  d = 2048-fineScanPos;
  deltaScan = -10* int(d / fineHVratio);
  d = fineHVratio* int(deltaScan/10);
  integral -= d;
  scanPos += deltaScan;
  fineScanPos = integral;

  SerialUSB.print("lockcenter: ");
  SerialUSB.println(scanPos);

}
  
//Send new value to DAC 2 Click over SPI 
void DACwrite(word out){

  if(out > DACmax)
  {
    out=DACmax;
  }
  if(out<DACmin)
  {
    SerialUSB.print("Error: DAC out: ");
    SerialUSB.println(out);
    out=DACmin;
  }
 
 spi[0] = B00110000;                        //Write to and Update (Power Up)
 spi[1] = highByte(out);
 spi[2] = lowByte(out); 
 SPI.transfer(pinDACselect, spi, 3);        //Send 24 bit to SPI https://www.analog.com/media/en/technical-documentation/data-sheets/2601fb.pdf
   
}//end void DACwrite(word out){

// Here we make Dither and its harmonics to be used as references
void makeSinewave(){
  //generise sinus 
  for(i=0;i<sinePoints;i++)
  {
    ditherData[i] = (cos(mu_omega * i*sampleTime_us ) + 1.0 )*maxres/2.0;//ovo ide na izlaz
    ditherXData[i] = cos(mu_omega * i*sampleTime_us); //samo sin(x) NIKAKO OFFSET
    ditherYData[i] = cos(mu_omega * i*sampleTime_us + M_PI/2.0); //samo sin(x +pi/2) NIKAKO OFFSET
    dither3XData[i] = cos(3.0 * mu_omega * i*sampleTime_us + phaseRAD);// samo sin(x) NIKAKO OFFSET    
    dither3YData[i] = cos(3.0 * mu_omega * i*sampleTime_us + M_PI/2.0 + phaseRAD);// samo sin(x +pi/2) NIKAKO OFFSET
  }
}
 