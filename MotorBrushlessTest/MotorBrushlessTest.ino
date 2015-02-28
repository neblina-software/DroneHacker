/*
*  DroneHacker (c) 2015 Anibal Gomez
*
*  Valor en consola:
*
*    Motor Brushless (A2212/13T)
*    0 = Iniciar (beeps)
*    30 = Iniciar (beeps de inicio)
*    100 = Girar motor
*
*    Referencias:
*    http://techvalleyprojects.blogspot.mx/2012/06/arduino-control-escmotor-tutorial.html
*/

#include <Servo.h>

Servo myMotor;
String incomingString;

void setup()
{
  // Arduino pin #9
  myMotor.attach(9);
  // Debug
  Serial.begin(9600);
  Serial.println("iniciando...");
}

void loop()
{
  if(Serial.available() > 0)
  {
    char ch = Serial.read();
  
    if (ch != 10){
      Serial.print("Valor recibido: ");
      Serial.print(ch, DEC);
      Serial.print('\n');
    
      incomingString += ch;
    }
    else
    {
      Serial.println("Imprimiendo cadena: ");
      Serial.println(incomingString);
    
      int val = incomingString.toInt();
    
      Serial.println("Imprimiendo valor: ");
      Serial.println(val);
    
      if (val > -1 && val < 181)
     {
       Serial.println("Valor entre 0 y 180");
       myMotor.write(val);
     }
     else
     {
       Serial.println("Valor fuera de rango 0 y 180");
       Serial.println("Error con la entrada");
     }
      incomingString = "";
    }
  }
}

