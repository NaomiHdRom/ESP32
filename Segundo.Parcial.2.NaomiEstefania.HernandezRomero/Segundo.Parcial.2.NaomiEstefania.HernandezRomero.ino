//EXAMEN DE REPOSICION 2    Naomi Estefanía Hernández Romero

//*******************DECLARACIONES********************************
        //Librerias       
#include "ESP32_IDE.h"
#include <LiquidCrystal.h>
#include <ESP32Servo.h>
#include <Stepper.h> 

        //inputs y outputs

#define POT         E4
#define LEDC_PIN    E5           //Señal PWM  CD
#define CTL         E6           //Señal Servo
const byte HOMEled= E7;      //salida del LED
#define SEL         E10          //SEL Btn
#define ON          E11          //ON Btn
#define RS          S7           //Señal LCD RS     
#define EN          S8           //Señal LCD EN      
#define D4          S9           //Señal LCD D4   
#define D5          S10          //Señal LCD D5   
#define D6          S11          //Señal LCD D6
#define D7          S12          //Señal LCD D7

      //auxiliares para el motor CD
#define LEDC_CH_4  4            // Canal 4
#define LEDC_RES_3 3           // Resolución 
#define LEDC_FREQ1 1000        // Frecuencia 1000Hz


      //auxiliares para el DipSwitch

const byte DS0=S0;          //Dipswitch dígito Q0   Entrada del Dip
const byte DS1=S1;           //Dipswitch dígito Q1  Entrada del Dip 


      //auxiliares para el motPP
const byte O[4]={S3,S4,S5,S6};   
#define PASOS 2056 
Stepper stepper(PASOS,S3,S5,S4,S6); 
int   pps=20;                   //Pasos por segundo
int i;                          
float velPP;                      //velocidad motorPP
int   tmpp;                      //tiempo motorPP
byte cuenta;                      //contador de los pasos
unsigned char paso;                 //cada paso
int tiempo_motorpp;
const int prd[]{12,8,5,3,2,1.5};    //Establecimiento del periodo
      //auxiliares para el motCD
const byte IN1=E8;           //Motor CD IN1
const byte IN2=E9;           //Motor CD IN2
unsigned long tiempoactual; 

      //auxiliares para la rutina de seleccion
int estadoVEL=1;

      //auxiliares para el filtro
bool ONbtn=false;
#define Tfiltro 400
int tmrFiltro=0;

      //auxiliares para el potenciometro
int tiemPot;
int tpp;

      //auxiliares para la impresión en pantalla LCD
int SelVelocidad;
bool DMotor;
bool sen;


int tiempoPOT=0;

      // Configuración pines despliegue de cristal líquido
LiquidCrystal LCD1(RS, EN, D4, D5, D6, D7); 

      // Instanciamiento (creación) de un objeto de la clase Servo                               
Servo servo;    
int ang;     

//************************************CREACION DE FUNCIONES VOID*******************************
      //filtro 
bool filtro()
{
  if ( millis() - Tfiltro >= tmrFiltro )
  {
    tmrFiltro = millis();
    return(true);
  }
  else
  {
    return(false);
  }
}

      //encendido al hacer click en ONbtn
 void clkON()
 {
   if (filtro())
   {
     ONbtn=!ONbtn;
   }
 }

      //Seleccion de Velocidad segun el numero de clicks en SELbtn
void clkSEL()
{
  if (filtro())
  {
    if (estadoVEL<6)
    {
      estadoVEL++;
    }
    else
    {
      estadoVEL=1;
    }
  }
}

      //Impresion del motor, tiempo y velocidad en LCD
void fnLCD ()         
{

  if(estadoVEL>0)
  {
    if (DMotor) //Si DMotor=true......
    {
      LCD1.home ();           
      LCD1.setCursor(0,1);                                
      LCD1.print ("MOT");     
      LCD1.setCursor(13,1);                                
      LCD1.print ("TRN");        
    }
    else      //Si DMotor=false
    {
      LCD1.home ();
      LCD1.setCursor(0,1);                                
      LCD1.print ("MOTPP");
      
       LCD1.setCursor(13,1);                                
      LCD1.print ("TRN");   
    }   
  } 
  else
    {
    digitalWrite(IN1,LOW);      //INICIALIZA EL MOTOR CD EN 0
    digitalWrite(IN2,LOW);        
    }  

    switch (estadoVEL)
  {
    case (1):
    velPP=12;
    LCD1.setCursor(0,0);
    LCD1.print("tPP=");
     LCD1.setCursor(13,1);                                
      LCD1.print ("TRN");   
    if  (DMotor)
    {
  

    LCD1.setCursor(6,1);
    LCD1.print ("V=1");
    }
    else
    {
    LCD1.home ();
    LCD1.setCursor(6,1);
    LCD1.print ("Vms=12");
    }
    break;

    case (2):
    velPP=8;
    LCD1.setCursor(0,0);
    LCD1.print("tPP=");
     LCD1.setCursor(13,1);                                
      LCD1.print ("TRN");   
    if  (DMotor)
    {
    
    LCD1.setCursor(6,1);
    LCD1.print ("V=2");
    }
    else
    {
    LCD1.home ();
    LCD1.setCursor(6,1);
    LCD1.print ("Vms=8");
    }
    break;

    case (3):
    velPP=5;
    LCD1.setCursor(0,0);
    LCD1.print("tPP=");
     LCD1.setCursor(13,1);                                
      LCD1.print ("TRN");   
    if  (DMotor)
    {
    
    LCD1.setCursor(6,1);
    LCD1.print ("V=3");
    }
    else
    {
    
    LCD1.setCursor(6,1);
    LCD1.print ("Vms=5");
    }
    break;

    case (4):
    velPP=3;
    LCD1.setCursor(0,0);
    LCD1.print("tPP=");
     LCD1.setCursor(13,1);                                
      LCD1.print ("TRN");   
    if  (DMotor)
    {
    
    LCD1.setCursor(6,1);
    LCD1.print ("V=4");
    }
    else
    {
    
    LCD1.setCursor(6,1);
    LCD1.print ("Vms=3");
    }
    break;

    case (5):
    velPP=2;
    LCD1.setCursor(0,0);
    LCD1.print("tPP=");
     LCD1.setCursor(13,1);                                
      LCD1.print ("TRN");   
    if  (DMotor)
    {
   
    LCD1.setCursor(6,1);
    LCD1.print ("V=5");
    }
    else
    {
    LCD1.setCursor(6,1);
    LCD1.print ("Vms=2");
    }
    break;

    case (6):
    velPP=1.5;
    LCD1.setCursor(0,0);
    LCD1.print("tPP=");
     LCD1.setCursor(13,1);                                
      LCD1.print ("TRN");   
    if  (DMotor)
    {
   
    LCD1.setCursor(6,1);
    LCD1.print ("V=6");
    }
    else
    {
    
    LCD1.setCursor(6,1);
    LCD1.print ("Vms=1.5");
    }
    break;

    default:
    
    LCD1.clear();
    LCD1.setCursor(0,0);
    LCD1.print("tPP=");
    LCD1.setCursor(10,0);
    LCD1.print("tCD=");
  
    LCD1.setCursor(0,1);
    LCD1.print("MOT ");
    LCD1.setCursor(7,1);
    LCD1.print ("Vel=");    
    LCD1.setCursor(13,1);
    LCD1.print("SEN");

  }
}

//FUNCIÓN TIEMPO MOTOR PASO A PASO
void funcionPot()
{
  tiemPot=analogRead(POT);
//tiemPot/=4;      
  if(tiemPot<=255)
  {
    LCD1.home();
    LCD1.setCursor(0,0);
    LCD1.print("tPP=1s");
    tpp=1000;    
  }
  if(tiemPot>255&tiemPot<=511)
  {
   
    LCD1.setCursor(0,0);
    LCD1.print("tPP=2s");
    tpp=2000;
  }
  if(tiemPot>511&tiemPot<=767)
  {
    
    LCD1.setCursor(0,0);
    LCD1.print("tPP=3s");
    tpp=3000;
  } 
  if(tiemPot>767&tiemPot<=1023)
  {       
   
    LCD1.setCursor(0,0);
    LCD1.print("tPP=4s");
    tpp=4000;
  }
}

//FUNCIÓN TIEMPO MOTOR CD
void fnCDTime ()
{
  if(digitalRead(DS1)&digitalRead(DS0))   // interr0 y interr0 -> 0
  {
    LCD1.setCursor(10,0);
      LCD1.print("tCD=2s");
    tiempoactual = millis()+(tiempo_motorpp);            //velocidad del motor CD=0
    while(millis()<tiempoactual &&  ONbtn)
    {
      LCD1.home();
      
                                               
    }
  }
  if(!digitalRead(DS1)& digitalRead(DS0))
  { //  interr0 y interr 1 -> 1 
    LCD1.setCursor(10,0);
      LCD1.print("tCD=4s"); 
    tiempoactual = millis()+(tiempo_motorpp);            //velocidad del motor CD=0
    while(millis()<tiempoactual &&  ONbtn)
    {
      LCD1.home();
      
                                                 
    }                                                                      
  }
  if(digitalRead(DS1)& !digitalRead(DS0))
  { // interr1 y interr0 -> 2 
    LCD1.home();    
    LCD1.setCursor(10,0);
      LCD1.print("tCD=6s");  
    tiempoactual = millis()+(tiempo_motorpp);            //velocidad del motor CD=0
    while(millis()<tiempoactual&&  ONbtn)
    {
      LCD1.home();
      LCD1.setCursor(10,0);
      LCD1.print("tCD=6s");  
                                                 
    }                                       
  }

  if(!digitalRead(DS1)& !digitalRead(DS0))
  { // interr1 y interr1 -> 3   
    LCD1.setCursor(10,0);
      LCD1.print("tCD=8s"); 
    tiempoactual = millis()+(tiempo_motorpp);            //velocidad del motor CD=0
    while(millis()<tiempoactual&& ONbtn)
    {
      LCD1.home();
      LCD1.setCursor(10,0);
      LCD1.print("tCD=8s"); 
                                                  
    }                                                                        
  }
}

//FUNCIÓN DEL MOTOR CD
void fnCD (bool SentCD)     //Sentido y PWM
{
  if (SentCD) //Dex
  {
    digitalWrite (IN1, HIGH);
    digitalWrite (IN2, LOW);
  }
  else //lev
  {
    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);
  }

  switch (estadoVEL)
  {
    case (1):
    ledcWrite(LEDC_CH_4, 255*.5); //50%
    LCD1.setCursor(6,1);
    LCD1.print ("V%=");
    LCD1.print(50);
    
    break;
    
    case (2):
    ledcWrite(LEDC_CH_4, 255*.6); //60%
    LCD1.setCursor(6,1);
    LCD1.print ("V%=");
    LCD1.print(60);
    break;
  
    case (3):
    ledcWrite(LEDC_CH_4, 255*.7); //70%
    LCD1.setCursor(6,1);
    LCD1.print ("V%=");
    LCD1.print(70);  
    break;
    
    case (4):
    ledcWrite(LEDC_CH_4, 255*.8); //80% 
    LCD1.setCursor(6,1);
    LCD1.print ("V%=");
    LCD1.print(80);
    break;
    
    case (5):
    ledcWrite(LEDC_CH_4, 255*.9); //90% 
    LCD1.setCursor(6,1);
    LCD1.print ("V%=");
    LCD1.print(90);
    break;
  
    case (6):
    ledcWrite(LEDC_CH_4, 255); //100%  
    LCD1.setCursor(6,1); 
    LCD1.print ("V%=");
    LCD1.print(100);
    break;
    

    default:
    ledcWrite(LEDC_CH_4, 0); //0%
    break;
  }
  

}

//////FUNCIONAMIENTO PRENDER MOTOR PASO A PASO

void fnPP (bool SentPP)     //Sentido y PWM
{ 
  switch (estadoVEL)
  {
    case (1):
    stepper.setSpeed(5);
    tmpp=millis()+tiempo_motorpp;
    while(millis()<tmpp){
      if(SentPP){
      stepper.step(1);
      }else{
      stepper.step(-1);
      }
    }
    break;
    
    case (2):
    stepper.setSpeed(6);
    tmpp=millis()+tiempo_motorpp;
    while(millis()<tmpp){
      if(SentPP){
      stepper.step(1);
      }else{
      stepper.step(-1);
      }
    }
   
 
    break;
  
    case (3):
    stepper.setSpeed(7);
    tmpp=millis()+tiempo_motorpp;
    while(millis()<tmpp){
      if(SentPP){
      stepper.step(1);
      }else{
      stepper.step(-1);
      }
    }
 
   
    break;
    
    case (4):
   
    stepper.setSpeed(8);
    tmpp=millis()+tiempo_motorpp;
    while(millis()<tmpp){
      if(SentPP){
      stepper.step(1);
      }else{
      stepper.step(-1);
      }
    }
    break;
    
    case (5):
    
    stepper.setSpeed(9);
    tmpp=millis()+tiempo_motorpp;
    while(millis()<tmpp){
      if(SentPP){
      stepper.step(1);
      }else{
      stepper.step(-1);
      }
    }
    break;
  
    case (6):
   
    stepper.setSpeed(9.8);
    tmpp=millis()+tiempo_motorpp;
    while(millis()<tmpp){
      if(SentPP){
      stepper.step(1);
      }else{
      stepper.step(-1);
      }
    }
    break;
    

    default:

  delay(12000);    
    
    break;

    

 }
}



///TIEMPO DE FUNCIONAMIENTO DEL MOTOR PASO A PASO CON EL POTENCIOMETRO

void fnPOT(){
 tiempoPOT = analogRead(POT);
 tiempoPOT/=4;
  
    LCD1.home();
    if (tiempoPOT<= 255){ tiempo_motorpp = 1000; LCD1.setCursor(0,0);LCD1.print("tPP=1s");} 
     if (tiempoPOT<= 511& tiempoPOT>255) {tiempo_motorpp = 2000; LCD1.setCursor(0,0);LCD1.print("tPP=2s");} 
     if (tiempoPOT<= 767 & tiempoPOT>511) {tiempo_motorpp = 3000; LCD1.setCursor(0,0);LCD1.print("tPP=3s");} 
     if (tiempoPOT<= 1023& tiempoPOT>767) {tiempo_motorpp = 4000; LCD1.setCursor(0,0);LCD1.print("tPP=4s");}
    
  
    }




void setup()     
{ 

  Serial.begin(9600);
  //Entradas PULLUP
  pinMode (DS0, INPUT_PULLUP);
  pinMode (DS1, INPUT_PULLUP);
  //Entradas
  pinMode (ON, INPUT_PULLUP);         //Botón ON como entrada
  pinMode (SEL, INPUT_PULLUP);        //Botón SEL como entrada 
  
  // Salidas  
  pinMode(HOMEled,OUTPUT);   

  stepper.setSpeed(5);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ON),clkON,FALLING);
  attachInterrupt(digitalPinToInterrupt(SEL),clkSEL,FALLING);

  //Configuración del LEDC del motor DC
  ledcSetup (LEDC_CH_4,LEDC_FREQ1,LEDC_RES_3);
  ledcAttachPin (LEDC_PIN,LEDC_CH_4);
  

  // Configuración del servomotor
  servo.attach (CTL,500,7500);
  servo.write(0);

  // Configuración despliegue de cristal líquido
  LCD1.begin(16,2);            
  LCD1.clear();                // Limpia LCD

  //Led de Estado HOME
  digitalWrite (HOMEled,HIGH);

  //Configuración de motor a pasos

  for (i=0;i<4;i++)
  {pinMode(O[i],OUTPUT);}

velPP=0;
}

void loop() 
{
             
  
  DMotor=true;
  
  fnLCD();
  fnCDTime();
  fnPOT();



 if (ONbtn) // Inicio de Rutina
  {
    //1               ************PP(lev) CCW
    fnLCD();
    LCD1.setCursor(0,1);
    LCD1.print("MOTPP");
    LCD1.setCursor(6,1);
    LCD1.print ("Vms=");  
    LCD1.print (velPP); 
    LCD1.setCursor(13,1);
    LCD1.print("CCW");
    DMotor=false;  
    fnPP(false);
    delay(1000); 


    //2               *********SERVO 0-180 (dex)  CW
    LCD1.setCursor(0,0);                  //DESPEJAR PANTALLA
    LCD1.clear();
    LCD1.setCursor(0,1);
    LCD1.print("MOTSERVO");
    LCD1.setCursor(13,1);
    LCD1.print("CW");

    fnPOT();

    fnCDTime ();
     for(ang=0;ang<180;ang++)
    {

        servo.write(ang);
        delay(13);

    }

    delay(1000);       


    //3                        *************PP (dex)  CW
 
    LCD1.clear();
    DMotor=true;
    
    LCD1.setCursor(0,1);
    LCD1.print("MOTPP");
    LCD1.setCursor(6,1);
    LCD1.print ("Vms=");   
    LCD1.print (velPP); 
    LCD1.setCursor(13,1);
    LCD1.print("CW");
    
   fnPOT();

    fnCDTime ();
    
    fnPP(true);
    delay(1000);                              
  

    //4                     ************** CD (dex) CW
    DMotor=true;
   LCD1.clear();
    LCD1.setCursor(6,1);
    LCD1.print ("V%=");
    LCD1.setCursor(0,1);
    LCD1.print("MOTCD");
    LCD1.setCursor(13,1);
    LCD1.print("CW");
    fnPOT();
    fnCD (true);  
    fnCDTime ();  
   
    digitalWrite(IN1,LOW); //Apagado del Motor
    digitalWrite(IN2,LOW);
    delay(1000);    
                             
    //5               ************PP (lev) CCW
    
    LCD1.clear();
    DMotor=false;
    
    LCD1.setCursor(0,1);
    LCD1.print("MOTPP");
    LCD1.print ("Vms=");   
    LCD1.print (velPP); 
    LCD1.setCursor(13,1);
    LCD1.print("CCW");

    
    
    fnPOT();

    fnCDTime ();
 
  

    fnPP(false);
    delay(1000); 

                         
    //6          *************SERVO 180-0 (lev) CCW
    LCD1.clear();
    LCD1.setCursor(0,1);
    LCD1.print("MOTSERVO");
    LCD1.setCursor(13,1);
    LCD1.print("CCW");
    fnPOT();

    fnCDTime ();
 
   
    for(ang=180;ang>0;ang--)
    {

        servo.write(ang);
        delay(13);

    }




    delay(1000);      


    //7         ***************PP (dex) CW

     LCD1.clear();
         DMotor=false;
     
    LCD1.setCursor(0,1);
    LCD1.print("MOTPP");
    LCD1.print ("Vms=");   
    LCD1.print (velPP); 
    LCD1.setCursor(13,1);
    LCD1.print("CW");
    fnPOT();

    
    fnCDTime ();
    

    fnPP(true);
    delay(1000);        


    //8           ***********llamar void CD lev
   
    LCD1.clear();
    LCD1.setCursor(6,1);
    LCD1.print ("V%=");
    LCD1.setCursor(6,1);
    LCD1.print ("V%=");
    LCD1.setCursor(0,1);
    LCD1.print("MOTCD");
    LCD1.setCursor(13,1);
    LCD1.print("CCW");
    fnPOT();
    fnCD (false);             //llama a funcionar el motor en sentido false=Lev
    fnCDTime ();
    
    digitalWrite(IN1,LOW); //Apagado del Motor
    digitalWrite(IN2,LOW);
    delay(1000);     


   //9     *************regresar a HOME 
      LCD1.clear();
      LCD1.setCursor(0,1);                                
      LCD1.print ("MOT");     
      LCD1.setCursor(13,1);                                
      LCD1.print ("TRN"); 
      LCD1.setCursor(6,1);
      LCD1.print ("V=1");
      fnPOT();
      LCD1.setCursor(9,0);
      fnCDTime ();
      
    
    ONbtn=false;
    
  
  }
}
