//--------------------------------------------------------------------------
// Water heating control
// Author: Jonatan Rodriguez
// 
//--------------------------------------------------------------------------

//libraries//*****************************************************
//TFT.......................................
  #include <MCUFRIEND_kbv.h>
  #include <OneWire.h>
  #include <DallasTemperature.h>
//INTERRUPCIONES//..........................
  #include <avr/io.h>
  #include <avr/interrupt.h>

//MACROS//*****************************************************
//macro for detection af rasing edge
  #define RE(signal, state) (state=(state<<1)|(signal&1)&3)==1
  
// global constants and variables//*****************************************************
  //main
    int WorkMode=0; 
  //INTERRUPCIONES//..........................
    unsigned long lastmicros;
    unsigned long delta;
    int T_max=50;//
    int _100ms_tick=0;
  //ON OFF POWER MODULATION//....................
    int DutyCycle; 
  //TFT........................................
    MCUFRIEND_kbv tft;
    #define WHITE   0xFFFF
    #define BLACK   0x0000
    #define BLUE    0x001F
    int width = tft.width();
    int height = tft.height();    
  //IO PARAMETRIZATION..........................
    const int OneWirePin = 52;
    const int SsrPin = 53; 
    const int ButtonPin = 22;
    const int analogPin = A15;
    boolean Button;
    boolean SsrOn;
    int AnalogIn;
  //Temperature Control general variables..........................
    #define eMYIDLE 0  //IDLE MODE THE OUTOUT IS OFF
    #define eONOFF 1   //ON OFF hYSTERESIS CONTROL
    #define eMYPWM 2   //MANUAL "PWM" REGULATION
    #define eMYPILOOP 3//PiMode
    #define eMYAWPILOOP 4//anti windup PI
    #define eMYAWBETAPILOOP 5
    float Setpoint=30;
    float Feedback;
    float MyError;
  //ON_OFF HYSTERESYS CONTROL....................
    float Onspan=0.1 ;
    float Offspan=1;
  //Pi CONTROL
    unsigned long LastITime;
    float Beta;
    float PIAction;
    float ControlAction;
    float IntegralError;
    float ITime;
    float Kp=0.1;
    float Ki=0.01;
    float Paction;
    float Iaction;
  //ONEWIRE BUS AND SENSORS.................... 
    OneWire Onewire(OneWirePin);
    DallasTemperature Sensors(&Onewire);
    DeviceAddress WaterTempSensor =    {0x28,0xFF,0xD0,0xC6,0x91,0x15,0x03,0x54};
    DeviceAddress AmbientTempSensor ={0x28,0xFF,0x15,0xC9,0x91,0x15,0x03,0x3E};
    float WaterTemp;
    float AmbientTemp;
 
void setup(){//*****************************************************
  //inicializacion interrupcion.................
    cli();//habilitacion de interupciones
    TCCR3A=0;//
    TCCR3B=0;//configuracion  del timer
    OCR3A=1566;//valor precargado para crea una interrupcion de 100ms
    TCCR3B |=   (1<<WGM32);
    TCCR3B |=   (1<<CS32);//cpnfiguracopn prescaler en 1024
    TCCR3B |=   (0<<CS31);
    TCCR3B |=   (1<<CS30);
    TIMSK3=(1<<OCIE3A);
    sei();
  //serial monitor..................................
    Serial.begin(115200);
    while (!Serial);
  //oneWire Sensos
    Sensors.begin();
  //tft..................................
    tft.begin();
    tft.fillScreen(BLUE);
   // tft.drawRect(0, 0, width, height, WHITE);
   //IO..................................
    pinMode(SsrPin, OUTPUT);
    pinMode(ButtonPin, INPUT);
    //Pi
    IntegralError=0;
    Iaction=0;
}

void SensorsReading(){//*****************************************************
  Sensors.requestTemperatures();
  WaterTemp=Sensors.getTempC(WaterTempSensor);
  AmbientTemp=Sensors.getTempC(AmbientTempSensor);
}

void MyTft(){//*****************************************************
  //my variables
    char *sWmode;
 //code
    tft.setCursor(1, 1); 
    tft.setTextSize(2);
    tft.setTextColor(WHITE,BLACK);
    tft.println("TEMPERATURE CONTROL");
    tft.setCursor(1, 20);
    tft.setTextSize(2);
    tft.println("");
    tft.setTextColor(WHITE,BLUE);
    tft.print("   WaterTemp: ");
    tft.println(WaterTemp);
    tft.print(" AmbientTemp: ");
    tft.println(AmbientTemp);    
  //every screen definition
    switch (WorkMode){
    case eMYIDLE://----------------------------------
       sWmode=("Mode 1- CONTROL OFF");
      break;
    case eONOFF://-------------------------------
      tft.println();
      tft.print("       +Span: ");tft.println((Setpoint + Onspan));
      tft.print("    SetPoint: ");tft.println(Setpoint);
      tft.print("       -Span: ");tft.println((Setpoint - Offspan));
      tft.println();
      tft.print("      Output:  ");tft.println((SsrOn));

      sWmode=("Mode 2- Hysteresis On/Off Control");
      break;
    case eMYPWM://--------------------------
      tft.println();
      tft.print("   DutyCycle: ");
      tft.println(DutyCycle);
      sWmode=("Mode 3-  5s period PWM control");
      tft.print("      Output:  ");
      tft.println((SsrOn));
      break;
    case eMYPILOOP://--------------------------
      tft.print("    SetPoint: "); tft.println(Setpoint);
      tft.print("    error: ");tft.println(MyError);
      tft.setTextSize(1);
      tft.println();
      tft.print(" kp: ");tft.print(Kp); tft.print(" Paction: ");tft.println(Paction);
      tft.println();
      tft.print(" Integration time(s): ");tft.println(ITime);
      tft.println();
      tft.print(" ki: ");tft.print(Ki);tft.print("  Iaction: ");tft.println(Iaction);
      tft.println();
      tft.print(" ControlAction: ");tft.println(ControlAction);
      sWmode=("Mode 4-  PI control");
      break;

    case eMYAWPILOOP://--------------------------
      tft.print("    SetPoint: "); tft.println(Setpoint);
      tft.print("    error: ");tft.println(MyError);
      tft.setTextSize(1);
      tft.println();
      tft.print(" kp: ");tft.print(Kp); tft.print(" Paction: ");tft.println(Paction);
      tft.println();
      tft.print(" Integration time(s): ");tft.println(ITime);
      tft.println();
      tft.print(" ki: ");tft.print(Ki);tft.print("  Iaction: ");tft.println(Iaction);
      tft.println();
      tft.print(" PI Action: ");tft.println(PIAction);
      tft.println();
      tft.print(" Control Action: ");tft.println(ControlAction);
      sWmode=("Mode 5- Anti-Windup PI control");
      break;
      
  case eMYAWBETAPILOOP://--------------------------
      tft.print("    SetPoint: "); tft.println(Setpoint);
      tft.print("    error: ");tft.println(MyError);
      tft.setTextSize(1);
      tft.println();
      tft.print(" kp: ");tft.print(Kp); tft.print(" Paction: ");tft.println(Paction);
      tft.println();
      tft.print(" beta: ");tft.print(Beta); tft.print(" Betaaction: ");tft.println(Setpoint*Beta);
      tft.println();
      tft.print(" Integration time(s): ");tft.println(ITime);
      tft.println();
      tft.print(" ki: ");tft.print(Ki);tft.print("  Iaction: ");tft.println(Iaction);
      tft.println();
      tft.print(" PI Action: ");tft.println(PIAction);
      tft.println();
      tft.print(" Control Action: ");tft.println(ControlAction);
      sWmode=("Mode 6-  BetaWeigh Anti-Windup PI control");
      break;
    }
    tft.setCursor(1, 20);
    tft.setTextSize(1);
    tft.print(sWmode);
 }

bool OnoffControl(float Feedback) { //*****************************************************
  if (Feedback > (Setpoint + Onspan)) {return false;}
  else if ( Feedback<=(Setpoint - Offspan)){return true;}
}
void Serialplot(){//*****************************************
  Serial.print("SP ");
  Serial.print(Setpoint);
  Serial.print(" Water ");
  Serial.print(WaterTemp);
  Serial.print(" fbk ");
  Serial.print(Feedback);
  Serial.print(" ssr ");
  Serial.print(SsrOn);
  Serial.print(" +sp ");
  Serial.print((Setpoint + Onspan));
  Serial.print(" -sp ");
  Serial.println((Setpoint - Offspan));
  Serial.print("WorkMode");
  Serial.println(WorkMode); 
}

bool OnOffPowerModulation(int DutyCycle){//****************************************************
  if (_100ms_tick<=DutyCycle){return true;}
  else {return false;}
}
int MyPiControl(){//****************************************************
    Feedback=WaterTemp;
    MyError=Setpoint-Feedback;
    Paction=MyError*Kp;
    ITime=(float)(millis()-LastITime)/1000;
    IntegralError=MyError*ITime*Ki;
    Iaction=Iaction+IntegralError;
    LastITime=millis();
    PIAction=Paction+Iaction;
    ControlAction=PIAction ; 
    return ControlAction;
}
int MyAWPiControl(){//****************************************************
    Feedback=WaterTemp;
    MyError=Setpoint-Feedback;
    Paction=MyError*Kp;
    ITime=(float)(millis()-LastITime)/1000;
    IntegralError=MyError*ITime*Ki;
    Iaction=Iaction+(IntegralError+(ControlAction-PIAction))*ITime;
    LastITime=millis();
    PIAction=Paction+Iaction;
    ControlAction= AntiSaturation(0,50,PIAction); 
    return ControlAction;
}
int MyAWBetaPiControl(){//****************************************************
    Feedback=WaterTemp;
    MyError=Setpoint-Feedback;
    Paction=Kp*(Setpoint*Beta-Feedback);
    ITime=(float)(millis()-LastITime)/1000;
    IntegralError=MyError*ITime*Ki;
    Iaction=Iaction+(IntegralError+(ControlAction-PIAction))*ITime;
    LastITime=millis();
    PIAction=Paction+Iaction;
    ControlAction= AntiSaturation(0,50,PIAction); 
    return ControlAction;
}
float AntiSaturation(float vmin,float vmax,float value){
  if (value>vmax){return vmax;}
  else if (value<vmin){return vmin;}
  else{return value;}
}
void IO(){//*****************************************************
  //field inputs management
    Button = digitalRead(ButtonPin);
    AnalogIn = analogRead(analogPin);          // realizar la lectura analógica raw
  //field output management
    switch (WorkMode){
      case eMYIDLE://----------------------------------
        SsrOn = false;
        break;
      case eONOFF://-------------------------------
        SsrOn=OnoffControl(WaterTemp);
        break;
      case eMYPWM://--------------------------
        SsrOn=OnOffPowerModulation(DutyCycle);
        break;
      case eMYPILOOP:
        SsrOn=OnOffPowerModulation(MyPiControl());
         break;
      case eMYAWPILOOP:
        SsrOn=OnOffPowerModulation(MyAWPiControl());
         break;
      case eMYAWBETAPILOOP:
        SsrOn=OnOffPowerModulation(MyAWBetaPiControl());
         break;
      }
    if (SsrOn==true){digitalWrite(SsrPin, HIGH);}
    else {digitalWrite(SsrPin, LOW);}
}
ISR(TIMER3_COMPA_vect){//CYCLIC 100ms INTERRUPT TREATMENT*****************************************************
  delta=micros()-lastmicros;
  Serial.println(delta);
  lastmicros=micros();
  if (_100ms_tick<=T_max) {_100ms_tick=_100ms_tick+1;}
  else{_100ms_tick=0;}
  IO();
}
                  
void loop(){//MAIN PROGRAM*****************************************************
  SensorsReading();
  //System inputs management
    switch (WorkMode){
      case eMYIDLE://----------------------------------
        break;
      case eONOFF://-------------------------------
        Setpoint =(int) map(AnalogIn, 0, 1023, 20, 100);
        break;
      case eMYPWM://--------------------------
        DutyCycle = map(AnalogIn, 0, 1023, 0, 50);
        break;
      case eMYPILOOP://--------------------------
        Setpoint = map(AnalogIn, 0, 1023, 0, 100);
         break;
      case eMYAWPILOOP://-------------------------
        Setpoint = map(AnalogIn, 0, 1023, 0, 100);
         break;
      case eMYAWBETAPILOOP://---------------------
        Setpoint = map(AnalogIn, 0, 1023, 0, 100);
         break;
    } 
    int Button_RT;
    if(RE(digitalRead(ButtonPin), Button_RT)){
      IntegralError=0;
      Iaction=0;
      tft.fillScreen(BLUE);//limpiar pantalla
      if (WorkMode>5){WorkMode=0;}
      else {WorkMode++;}
    }    
    MyTft();
    //Serialplot();  
 }
