
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
float last_val = 0;
int sensorVal = LOW;
unsigned long prev = 0;
float actual_expenditure = 0;
int LED_PIN = 8;
float last_expenditure = 0;
int SIZE_TANK = 32; // size of tank in liters

float read_from_sensor()
{
  a2d_data=analogRead(A0); // READ data from analog A0 port (0 - 1023)
  
  buffer=a2d_data*Vin;
  Vout=(buffer)/1024.0; // calculate out voltage
  buffer=Vout/(Vin-Vout); 
  R_sensor=R_reference*buffer; // calculate unknown R2 from Ohm rule

  return R_sensor;
}

float calculate_percent_expenditure(float R_sensor)
{
  return ((R_sensor-min_value)/range_value)*100; //calculate actual fuel in percent
}

void calculate_fluel_expenditure()
{
  unsigned long now = millis();

  if((now - prev)/1000 >= 2){
    actual_expenditure = (last_val - value_percent);
    Serial.print("Aktualne zuzycie: ");
    Serial.println(actual_expenditure);
    prev = now;

    Serial.println(last_val);
    if(int(actual_expenditure) > 0)
    {
      last_expenditure = actual_expenditure;
      last_val = value_percent;  
    }
    if(int(actual_expenditure) < 0)
    {
      float last_detect = read_from_sensor();
      last_val = calculate_percent_expenditure(last_detect);
    }
  }
}

void setup() 
{
  lcd.init();
  lcd.backlight(); //turn on backlight
  Serial.begin(9600); //start communication via UART with speed 9600 baud/sec (symbols/seconds)
  pinMode(3, INPUT_PULLUP); //configure pin 3 as an output and active built in pull up resistor
  pinMode(LED_PIN, OUTPUT); //configure built in LED on board Arduino UNO
  prev = millis();
  float last_detect = read_from_sensor();
  last_val = calculate_percent_expenditure(last_detect);
}

void loop()
{

  {
    R_sensor=read_from_sensor();
    value_percent = calculate_percent_expenditure(R_sensor);
    
    if( value_percent <= 8){
      digitalWrite(LED_PIN, HIGH);
      lcd.setCursor(3,0);
      lcd.print("LOW FUEL!!!");
    } else{
      digitalWrite(LED_PIN, LOW);
      if(int(actual_expenditure) > 0)
      {
        lcd.setCursor(3,0);
        lcd.print("FUEL USE ");
        lcd.print((actual_expenditure*SIZE_TANK)/100, 1);
      }
      else
      {
        lcd.setCursor(0,0);
        lcd.print("L FUEL USE ");
        lcd.print((last_expenditure*SIZE_TANK)/100, 1);
      }
      lcd.print(" l");
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
  calculate_fluel_expenditure();
 
}
