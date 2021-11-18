
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

int Vin=5;        //voltage at 5V pin of arduino
float Vout=0;     //voltage at A0 pin of arduino
float R_reference =218;    //value of known resistance
float R_sensor=0;       //value of unknown resistance
int a2d_data=0;    
float buffer=0;
float value_percent = 0;
float max_value = 89;
float min_value = 2;
float range_value = max_value - min_value;

void setup() 
{
 lcd.init();
 lcd.backlight(); //turn on backlight
 Serial.begin(9600); //start communication via UART with speed 9600 baud/sec (symbols/seconds)
}

void loop()
{
  a2d_data=analogRead(A0); // READ data from analog A0 port (0 - 1023)
  if(a2d_data)
  {
    buffer=a2d_data*Vin;
    Vout=(buffer)/1024.0; // calculate out voltage
    //Serial.println(Vout);
    buffer=Vout/(Vin-Vout); 
    ///Serial.print("buffer = "); 
    //Serial.println(buffer);
    R_sensor=R_reference*buffer; // calculate unknown R2 from Ohm rule
    value_percent = (R_sensor/range_value)*100;
    lcd.setCursor(4,0);
    lcd.print("ohm meter");

    lcd.setCursor(0,1);
    lcd.print("R (ohm) = ");
    lcd.print(value_percent);
    Serial.println(R_sensor);
    delay(3000);
    //lcd.noBacklight();
  }
}
