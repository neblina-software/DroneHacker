/*
*  DroneHacker (c) 2015 Anibal Gomez
*
*  Valor en consola:
*
*    Probado con Motor Brushless (A2212/13T)
*    0 = Beeps
*    30 = Armar
*    43 = Girar motor
*    El valor de motores puede variar
*
* Quadcoptero
* -----------
*
*            NW      NE
*            |\     /|
*            ( \   / )
*             \ (_) / 
*              ) _ (  
*             / ( ) \ 
*            ( /   \ )
*            |/     \|
*            SW     SE
*       
* ---------------------------------
* anibalgomez@icloud.com
* ---------------------------------
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

Servo motorNW;
Servo motorNE;
Servo motorSE;
Servo motorSW;

String incomingString;

void setup()
{
  // Pins
  motorNW.attach(8);
  motorNE.attach(9);
  motorSE.attach(11);
  motorSW.attach(13);
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
       motorNW.write(val);
       motorNE.write(val);
       motorSE.write(val);
       motorSW.write(val);
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

