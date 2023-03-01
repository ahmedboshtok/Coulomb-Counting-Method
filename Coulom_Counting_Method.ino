

#include <LiquidCrystal.h>


/********************************Global variable for SOCFun************************/
float I_BATT,V_BATT;
float T_BATT,I_out;

/**********************************************************************************/



// Data wire is plugged into digital pin 2 on the Arduino
/*#define ONE_WIRE_BUS 2*/
/*************************coulom methode**************************************/
#define CAPACITY              1200//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define V_LIM                 8.4

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/**********************voltage sensor******************************************/
#define voltagepin            A1
#define ADC_SCALE             1024.0
#define VREF                  5.0

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/**************************CURRENT SENSOR*************************************/
//#define ADC_SCALE             1024.0
//#define VREF                  5.0

#define CURRENTPIN            A0
#define V_NOLOAD              2.5
#define SENSITIVITY          .185;
/*
  0.185V/A for 5A max type
  0.185V/A for 20A max type
  0.66V/A for 30A max type*/
/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/*************************************************************************************/



#define tempPin           A2

#define CURRENTPIN2       A3




double startTime, currentTime,TOC,TODC;
float SOC, DOD;
float SOC0 = 82;

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup() {
 Serial.begin(9600);
  lcd.begin(16, 2);            
  lcd.clear();    
  lcd.setCursor(0, 0); 
  lcd.print("EPS");



}

void loop() {
 
  CLAC_SOC_PERCENTAGE();



}





/**************************CURRENT SENSOR*************************************/
float getCurrent(int C_P){
  int adc = analogRead(C_P);
  /*convert the ADC value to DC curret
  */
  /*****************************/
  //convert from ADC to to voltage
  float voltage = adc * VREF / ADC_SCALE;

  float current = (voltage - V_NOLOAD) / SENSITIVITY;
if (C_P ==CURRENTPIN2 ){
  
  Serial.print("Current input :");
  Serial.println(current);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Curr_I/P =");
  lcd.setCursor(11,0);
  lcd.print(current);
  lcd.setCursor(15,0);
  lcd.print("A");
  delay(3000);
  }
  else if (C_P ==  CURRENTPIN){
    Serial.print("Current output=");
  Serial.println(current);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Curr_O/P =");
  lcd.setCursor(11,0);
  lcd.print(current);
  lcd.setCursor(15,0);
  lcd.print("A");
  delay(3000);
  }
  else {
    Serial.print("invalid pin");
  }
    
  
  return current;
}
  /*********************************************************************/
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  /*********************VOLTAGE SENSOR*********************************/



  float getVoltage(){
    float ADCvalue = analogRead(voltagepin);
    float vOUT = (ADCvalue * VREF) / ADC_SCALE;
    
    float VIN = vOUT*3;
    
    Serial.print("Input voltage = ");
    Serial.println(VIN);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Volt =");
    lcd.setCursor(7,0);
    lcd.print(VIN);
    lcd.setCursor(12,0);
    lcd.print("v");
    delay(3000);
    return VIN;
  }
  /**************************************************************************/
  ////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////
  /***************************TEMPERATURE SENSOR****************************/

  
float getTemperature(){
  float val = analogRead(tempPin);
  float Vd = 1024/4.96;
  float mv = (val / Vd)*1000;
  float K = mv /10;
  float cel = K - 273.15;
  lcd.clear();
  lcd.setCursor(0,0);

  Serial.print("TEMPRATURE = ");
  lcd.print("Temp =");
  lcd.setCursor(7,0);
  lcd.print(cel);
  lcd.setCursor(13,0);
  lcd.print(((char)223));
  lcd.setCursor(14,0);
  lcd.print("C");
  Serial.print(cel);
  Serial.print(((char)223));
  
  Serial.println();
    

    delay(3000);
    return cel;
  }
void CLAC_SOC_PERCENTAGE(void)
{
  I_BATT =  getCurrent(CURRENTPIN2);//A3
  I_out = getCurrent (CURRENTPIN);//A0
  V_BATT = getVoltage();
  //V_BATT =8.2;
  T_BATT = getTemperature();
 

  startTime= millis();
  
  if ( T_BATT >= 0 && T_BATT <= 40 && V_BATT > 6.2) {
    // ideal  state 
    // supply off and load off
    
    if (I_BATT < 0.2 && I_out < 0.2){
      
      SOC = SOC0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Idle Case");
      lcd.setCursor(0,1);
      lcd.print("SOC =");
      lcd.setCursor (6,1);
      lcd.print(SOC);
      lcd.setCursor(10,1);
      lcd.print("%");
      
    }
    /*******************CHARG**************************************/
    else if(I_BATT > I_out  )
    
    {
       currentTime = millis();
        TOC = (currentTime - startTime); 
     
          startTime = currentTime;
          
          SOC = SOC0 + (I_BATT / CAPACITY) * TOC * 100;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("charging ...");
          lcd.setCursor(0,1); 
          lcd.print("SOC =");
          lcd.setCursor(6,1);
          lcd.print(SOC);
          lcd.setCursor(10,1);
          lcd.print("%");
        /*  Serial.println("charge");
          Serial.println("SOC = ");
          Serial.println(SOC);*/
          SOC0 = SOC;
          delay (3000);
      
      
      
    }
    else if (I_out > I_BATT )
    {
      lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Discharging ...");
       
        currentTime = millis();
        TODC = (currentTime - startTime);
        SOC = SOC0 + (-I_out / CAPACITY) * TODC * 100;
        lcd.setCursor(0,1);
        lcd.print("SOC=");
        lcd.print(SOC);
        lcd.setCursor(8,1);
        lcd.print("%");
        Serial.println("Discharge");
        Serial.println("SOC = ");
        Serial.println(SOC);
        
       // DOD = 100 - SOC;

        //lcd.print("DOD = ");
        //lcd.print(DOD);
        //Serial.println("DOD= ");
        //Serial.println(DOD);

        SOC0 = SOC;
        delay (3000);
      
      
    }
    else{
      delay (3000);
    }
 
  }
  else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Cooling ...");
    lcd.setCursor(0,1);
    lcd.print("Down");
    
    delay (30000);
    
  }
  /*******************TEMPERATURE MONITORING***************/
  if ( T_BATT < 0 && T_BATT > 55) {
    
    lcd.clear();
    lcd.setCursor(0,0);
    
    lcd.print("waiting ...");
    Serial.println("zorbieh heat");
    delay (30000);
  }



  
}
