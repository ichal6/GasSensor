
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h>
#include <math.h>
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

int Vin=5;        //voltage at 5V pin of arduino
float Vout=0;     //voltage at A0 pin of arduino
float R_reference =218;    //value of known resistance
float R_sensor=0;       //value of unknown resistance
int a2d_data=0;    
float buffer=0;
float value_percent = 0;
float max_value = 89.91;
float min_value = 1.93;
float range_value = max_value - min_value;
float last_val = 30;
int sensorVal = LOW;
unsigned long prev = 0;
float actual_expenditure = 0;
int LED_PIN = 8;


float read_from_sensor()
{
  a2d_data=analogRead(A0); // READ data from analog A0 port (0 - 1023)
  
  buffer=a2d_data*Vin;
  Vout=(buffer)/1024.0; // calculate out voltage
  //Serial.println(Vout);
  buffer=Vout/(Vin-Vout); 
  ///Serial.print("buffer = "); 
  //Serial.println(buffer);
  R_sensor=R_reference*buffer; // calculate unknown R2 from Ohm rule

  return R_sensor;
}

void setup() 
{
 lcd.init();
 lcd.backlight(); //turn on backlight
 Serial.begin(9600); //start communication via UART with speed 9600 baud/sec (symbols/seconds)
 pinMode(3, INPUT_PULLUP); //configure pin 3 as an output and active built in pull up resistor
 pinMode(LED_PIN, OUTPUT); //configure built in LED on board Arduino UNO
 prev = millis();
     
}

void loop()
{

  {
    R_sensor=read_from_sensor();
    value_percent = ((R_sensor-min_value)/range_value)*100; //calculate actual fuel in percent
    // Serial.println(R_sensor);
    
    if( value_percent <= 8){
      digitalWrite(LED_PIN, HIGH);
      lcd.setCursor(3,0);
      lcd.print("LOW FUEL!!!");
    } else{
      digitalWrite(LED_PIN, LOW);
      lcd.setCursor(3,0);
      lcd.print("FUEL USE ");
      lcd.print(actual_expenditure);
    }
    
    if (sensorVal == LOW) { // LOW = don't push button
      lcd.backlight();
        
      lcd.setCursor(4,1);
      lcd.print(round(value_percent));
      lcd.print("\% FUEL");
      delay(4000);
    } else {
      lcd.noBacklight();
    }
    delay(300);
    lcd.clear();
  }
  sensorVal = digitalRead(3); //read button status
  // Serial.println(sensorVal);

  unsigned long now = millis();

  if((now - prev)/1000 >= 20){
    actual_expenditure = (last_val - R_sensor);
    Serial.print("Aktualne zuzycie: ");
    Serial.println(actual_expenditure);
    prev = now;
    last_val = R_sensor;
  }
}
