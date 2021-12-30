// INCLUDES
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h>
#include <math.h>
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

//GLOBAL VALUES
float Vout=0;     //voltage at A0 pin of arduino


float value_percent = 0;
float last_val = 0;

int sensorVal = LOW;

unsigned long prev = 0;
float actual_expenditure = 0;
float last_expenditure = 0;

//COSTANT
int LED_PIN = 8;
int SIZE_TANK = 32; // size of tank in liters
float SAMPLING = 1*60; //size in seconds

float max_value = 90.3;
float min_value = 2.4;
float range_value = max_value - min_value;

float R_reference = 219;    //value of known resistance
int Vin=5;        //voltage at 5V pin of arduino

float read_from_sensor()
{
  float a2d_data=analogRead(A0); // READ data from analog A0 port (0 - 1023)
  
  float buffer=a2d_data*Vin;
  Vout=(buffer)/1024.0; // calculate out voltage
  buffer=Vout/(Vin-Vout);
  float R_sensor=R_reference*buffer; // calculate unknown Resistor from Ohm rule
  return R_sensor;
}

float calculate_percent_expenditure(float R_sensor)
{
  return ((R_sensor-min_value)/range_value)*100; //calculate actual fuel in percent
}

void calculate_fluel_expenditure()
{
  unsigned long now = millis();
  if((now - prev)/1000 >= SAMPLING){
    actual_expenditure = (last_val - value_percent);
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

void display_first_row()
{
  if( value_percent <= 8) // if fuel level is low
  {
    digitalWrite(LED_PIN, HIGH);
    lcd.setCursor(3,0);
    lcd.print("LOW FUEL!!!");
  } 
  else 
  {
    digitalWrite(LED_PIN, LOW);
    float value = 0;
    if(int(actual_expenditure) > 0) // display actual expenditure (the wehicle is moving)
    {
      lcd.setCursor(0,0);
      lcd.print("FUEL USE ");
      value = ((actual_expenditure*SIZE_TANK)/100)/(SAMPLING/3600);
      lcd.print(value, 1);
    }
    else // display last expenditure (the wehicle is stop)
    {
      lcd.setCursor(0,0);
      lcd.print("L. F. USE ");
      value = ((last_expenditure*SIZE_TANK)/100)/(SAMPLING/3600);
      lcd.print(value, 1);
    }

    lcd.print(" L");
  }
}

void display_second_row()
{
  if (sensorVal == LOW) { // LOW = button is pushing
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

void setup() 
{
  lcd.init();
  lcd.backlight(); //turn on backlight
  
  Serial.begin(9600); //start communication via UART with speed 9600 baud/sec (symbols/seconds)
  
  pinMode(3, INPUT_PULLUP); //configure pin 3 as an output and active built in pull up resistor
  pinMode(LED_PIN, OUTPUT); //configure built in LED on board Arduino UNO
  
  prev = millis(); //initialize time measure
  float last_detect = read_from_sensor();                //initialize first detect from sensor
  last_val = calculate_percent_expenditure(last_detect); // 
}

void loop()
{
  float R_sensor=read_from_sensor();
  value_percent = calculate_percent_expenditure(R_sensor);

  display_first_row();
  display_second_row();

  sensorVal = digitalRead(3); //read button status
  calculate_fluel_expenditure();
 
}
