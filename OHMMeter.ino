
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
int sensorVal = 0;

void setup() 
{
 lcd.init();
 lcd.backlight(); //turn on backlight
 Serial.begin(9600); //start communication via UART with speed 9600 baud/sec (symbols/seconds)
 //konfiguracja pinu3 jako wejście i aktywowanie wbudowanego rezystora podciągającego
 pinMode(3, INPUT_PULLUP);
}

void loop()
{
  sensorVal = digitalRead(3);   //odczytanie stanu przycisku i przypisanie go do zmiennej sensorVal
  a2d_data=analogRead(A0); // READ data from analog A0 port (0 - 1023)
  Serial.println(sensorVal);     //przesłanie odczytanej wartości zmiennej sensorVal do portu szeregowego
   
  if(a2d_data)
  {
    buffer=a2d_data*Vin;
    Vout=(buffer)/1024.0; // calculate out voltage
    //Serial.println(Vout);
    buffer=Vout/(Vin-Vout); 
    ///Serial.print("buffer = "); 
    //Serial.println(buffer);
    R_sensor=R_reference*buffer; // calculate unknown R2 from Ohm rule
    value_percent = ((R_sensor-min_value)/range_value)*100; //calculate actual fuel in percent
    // Należy pamiętać, że po aktywacji pull-up logika przycisku jest odwrócona
  // Jeśli przycisk jest nie naciśnięty to jego stan jest wysoki (HIGH),
  // a stan niski (LOW) kiedy jest naciśnięty. Dioda będzie zapalona kiedy przycisk
  // zostanie naciśnięty:
    if (sensorVal == LOW) {
      lcd.backlight();
      lcd.clear();
      lcd.setCursor(4,0);
      lcd.print("ohm meter");
  
      lcd.setCursor(4,1);
      lcd.print(round(value_percent));
      lcd.print("\% FUEL");
      delay(4000);
    } else {
      lcd.noBacklight();
    }
    delay(300);
    
    Serial.println(R_sensor);
  }
}
