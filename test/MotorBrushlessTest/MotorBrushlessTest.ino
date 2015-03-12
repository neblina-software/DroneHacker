/*
*  DroneHacker (c) 2015 Anibal Gomez
*
*  Valor en consola:
*
*    Probado con Motor Brushless (A2212/13T)
*    0 = Beeps
*    30 = Armar
*    54 = Girar motor
*    El valor de motores puede variar
*
*    Referencias:
*    http://techvalleyprojects.blogspot.mx/2012/06/arduino-control-escmotor-tutorial.html
*
* LICENSE
*
* This source file is subject to the new BSD license that is bundled
* with this package in the file LICENSE.txt.
*
*/

#include <Servo.h>

Servo myMotorPin5;
Servo myMotorPin7;
Servo myMotorPin9;
Servo myMotorPin11;

String incomingString;

void setup()
{
  // Pins
  myMotorPin5.attach(5);
  myMotorPin7.attach(7);
  myMotorPin9.attach(9);
  myMotorPin11.attach(11);
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
       myMotorPin5.write(val);
       myMotorPin7.write(val);
       myMotorPin9.write(val);
       myMotorPin11.write(val);
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

