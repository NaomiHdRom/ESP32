#include "ESP32_IDE.h"
#include <LiquidCrystal.h>

int LM35_ADC;
float T_C, T_F; 
#define LM35     E6         //Señal LCD D7

//LCD
#define RS     S7           //Señal LCD RS     
#define EN     S8           //Señal LCD EN      
#define D4     S9           //Señal LCD D4   
#define D5     S10          //Señal LCD D5   
#define D6     S11          //Señal LCD D6
#define D7     S12          //Señal LCD D7



// Configuración pines despliegue de cristal líquido
LiquidCrystal LCD1(RS, EN, D4, D5, D6, D7); 



void setup()     
{ 
 
Serial.begin(9600);
  
  // Configuración despliegue de cristal líquido
  LCD1.begin(16,2);            // LCD de 16 caracteres (16 columnas)y 2 renglones
  LCD1.clear();                // Limpia LCD

}

void loop() //
{
  
  float temp;
 LM35_ADC = analogRead(LM35);
 T_C = LM35_ADC/10.; //Temperatura °C
 T_F = (T_C*1.8)+32; //Remperatura °F
 LCD1.setCursor(0,1); //Ubica inicio de impresión de caracteres
 LCD1.printf("%3.2f", T_C);
 LCD1.write(byte(0));
 LCD1.printf("C %3.2f", T_F);
 LCD1.write(byte(0));
 LCD1.print("F");

 delay(1000); 

  Serial.print("Temperatura CELCIUS: ");
  Serial.println(T_C);
  Serial.print("Temperatura FARENHEIT: ");
  Serial.println(T_F);
  
  ////////Código realizado por el profesor Yukihiro Minami Koyama

}
