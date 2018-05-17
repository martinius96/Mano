#include <Wire.h> 
#include <LiquidCrystal_I2C.h> //kniznica pre displej s I2C
#include <EEPROM.h> //kniznica pre EEPROM pamat
#include <SPI.h>                     //kniznica SPI podporuje aj I2C pripojenie
#include <Ethernet.h>                //kniznica k ethernet shieldu
// DEFINOVANIE CASOV PREMENNYCH ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ODOZVA   50 // casova odozva nizsie ignoruje zmenu stavu tlacitka, enkodera
#define cas      10000 // cas vypnutia displej
#define cas2     2000 // cas vypnutia BAT RELE
// DEFINOVANIE PINOV PREMENNYCH ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MOSFET        5 // mosfet P solar
#define MOSFET1       6 // mosfet N baterka
#define MOSFET2       15 // mosfet N spotrebic
#define MOSFET3       7 // mosfet N svetlo
#define TLACITKO   17 // tlacidlo pre solar
#define TLACITKO1  18 // tlacidlo pre bateriu 
#define TLACITKO2  19 // tlacidlo pre spotrebic
#define TLACITKO3  16 // tlacidlo pre svetlo
#define encSW 3 // tlacidlo enkoder
#define encA  2 // spinac A enkoder
#define encB  1 // spinac B enkoder
#define RELE     22 // pripajanie siete rele
#define MOS      14 // pripajanie siete mosfet N
#define BAT       9 // odpajanie baterie mosfet N
#define AConBAToff 1 // stav 1
#define ACoffBATon 2 // stav 2
#define AConBATon  3 // stav 3
const int napatiepin = A1; // vstup pre napatie 2
const int napatiepin1 = A0; // vstup pre napatie 1
const int prudpin = A2; // vstup pre prud 1
const int prudpin1 = A3; // vstup pre prud 2
const int prudpin2 = A4; // vstup pre prud 3
// NASTAVENIE ETHERNETU ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte mac[] = { 0x20, 0x1A, 0x06, 0x75, 0x8C, 0xC3 };    //MAC adresa --> volitelna ANDRAS-0x08, 0x60, 0x6E, 0x83, 0x3F, 0x73 INTRAK-0x20, 0x1A, 0x06, 0x75, 0x8C, 0xC3
char server[] = "www.arduino.php5.sk";      //adresa webservera (moze byt aj IP adresa)
IPAddress ip(147, 232, 178, 111);                               //IP adresa zariadenia v lokalnej sieti ANDRAS- 147, 232, 102, 1 INTRAK- 147, 232, 178, 111 
EthernetClient client;                                          //spustenie ethernetu ako clienta 
String readString;
int x=0;
char lf=10;

LiquidCrystal_I2C lcd(0x27, 20, 4); // Nastavenie adresy 0x27 a počet znakov a riadkov displeja
#define eepromAddr 0
// KONSTANTY NA VYPOCET NAPATIA 1 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float vst_napatie =0.0;
float napatie;
// KONSTANTY NA VYPOCET NAPATIA 2 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float napatie1;
// KONSTANTY NA VYPOCET NAPATIA 3 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float napatie2;
// KONSTANTY NA VYPOCET NAPATIA 3 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float napatie3;
// KONSTANTY NA VYPOCET PRUDU 1 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int i;
int timer;
int konstanta = 100; // konstanta na prepočet nameraného napätia na prud pouzijeme 100 pre 20A verziu
int offset = 2500;// premenna na nastavenie offsetu, polovina Vcc
int analog= 0;
float napatieprud = 0.0;
float prud =0.0;
// KONSTANTY NA VYPOCET PRUDU 2 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int analog1= 0;
float napatieprud1 = 0.0;
float prud1 =0.0;
// KONSTANTY NA VYPOCET PRUDU 3 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int analog2= 0;
float napatieprud2 = 0.0;
float prud2 =0.0;
// VYPOCET OSTATNYCH VELICIN //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float vykon =0.0;
float vykon1 =0.0;
float vykon2 =0.0;
float akapacita =0.0;
float akapacita1 =0.0;
float akapacita2 =0.0;
float awatthodiny =0.0;
float awatthodiny1 =0.0;
float awatthodiny2 =0.0;
float kapacita =0.0;
float kapacita1 =0.0;
float kapacita2 =0.0;
float watthodiny =0.0;
float watthodiny1 =0.0;
float watthodiny2 =0.0;
volatile uint32_t sec=0;
volatile uint32_t hod=0;
// NABITIE BATERKY V % ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float prepocet=0.0;
float vysledok=0.0;
// PREPINANIE BATERKA A SIET //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long AConTime=0;
int curState=ACoffBATon;
int reqState=ACoffBATon;
int supplyChange=ACoffBATon;
int stateSteady=1;
// AUTOMATICKE ODPAJANIE SPOTREBICOV //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int a=1;
int err_prev=0;
volatile uint32_t err_millis=0;
//PREMENNE TLACIDLA ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool  MinulyStav = 1;  // priznak predchadzajuceho stavu tlacidla (0 .. stisnute)
bool  MinulyStav1 = 1;  // priznak predchadzajuceho stavu tlacidla1 (0 .. stisnute)
bool  MinulyStav2 = 1;  // priznak predchadzajuceho stavu tlacidla2 (0 .. stisnute)
bool  MinulyStav3 = 1;  // priznak predchadzajuceho stavu tlacidla3 (0 .. stisnute)
unsigned long CasZmeny; // premenna pre ulozenie casu zmeny stavu tlacidla
unsigned long CasZmeny1; // premenna pre ulozenie casu zmeny stavu tlacidla1
unsigned long CasZmeny2; // premenna pre ulozenie casu zmeny stavu tlacidla2
unsigned long CasZmeny3; // premenna pre ulozenie casu zmeny stavu tlacidla3
//NASTAVENIE ENKODERA//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile uint8_t Pos = 0;
volatile uint8_t SW = 0;
volatile uint8_t dispPos = 1;
volatile uint8_t prevPos = 1;
volatile uint8_t dispSW = 1;
volatile uint32_t ctlTime = 0;
// POLIA PRE ZOBRAZENIE HODNOT NA DISPLEJ /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int poleznak[] = {0,10,0,10,0,10,0,19};
int poleriadok[] = {0,0,1,1,2,2,3};
int poleVelicina[7];
char poleNuly[7];
int iznak=0;
int iriadok=0;
float stav=0;
volatile uint32_t casvyp=0;
int rele1=0;
int bat1=0;
// VLASTNE ZNAKY NA DISPLEJI //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte solar[8] = {
0b11111,
0b10101,
0b11111,
0b10101,
0b11111,
0b10101,
0b11111
};
byte baterka[8] = {
0b01110,
0b11011,
0b10001,
0b10001,
0b10001,
0b10001,
0b10001,
0b11111
};
byte spotrebic[8] = {
0b01110,
0b10001,
0b10101,
0b10101,
0b01110,
0b01110,
0b01110,
0b00100
};
byte siet[8] = {
0b01010,
0b01010,
0b11111,
0b10001,
0b10001,
0b01110,
0b00100,
0b00100
};
byte nabita[8] = {
0b00000,
0b00010,
0b00100,
0b01000,
0b11111,
0b00010,
0b00100,
0b01000
};
void setup() {
  lcd.begin();       //nastavenie znakov a riadkov displeja 
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("OSTROVNY  SYSTEM");
  lcd.setCursor(2, 2);
  lcd.print("MARIAN VERNARSKY");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.noBacklight();
  lcd.createChar(0, solar);
  lcd.createChar(1, baterka);
  lcd.createChar(2, spotrebic);
  lcd.createChar(3, siet);
  lcd.createChar(4, nabita);
  for(i=0;i<7;i++){ poleVelicina[i]=EEPROM.read(i);}  
// INICIALIZACIA VSTUPOV A VYSTUPOV ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pinMode(prudpin, INPUT);
  pinMode(prudpin1, INPUT);
  pinMode(prudpin2, INPUT);
  pinMode(MOSFET, OUTPUT);             
  pinMode(TLACITKO, INPUT_PULLUP);  
  pinMode(MOSFET1, OUTPUT);            
  pinMode(TLACITKO1, INPUT_PULLUP);  
  pinMode(MOSFET2, OUTPUT);            
  pinMode(TLACITKO2, INPUT_PULLUP);
  pinMode(MOSFET3, OUTPUT);            
  pinMode(TLACITKO3, INPUT_PULLUP);
  pinMode(MOS, OUTPUT);  
  pinMode(RELE, OUTPUT);             
  pinMode(BAT, OUTPUT);             
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(encSW, INPUT_PULLUP);
  //digitalWrite(BAT,HIGH);
// PRERUSENIA PRE ENKODER /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  attachInterrupt(digitalPinToInterrupt(encA), encRot, FALLING);
  attachInterrupt(digitalPinToInterrupt(encSW), encPush, FALLING);
  interrupts();
  }
// ROTACIA PRE ENKODER ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void encRot() 
{
if(((millis()-ctlTime) > ODOZVA) && (dispPos == 0))
  { prevPos = Pos;
     if(digitalRead(encB)) { if(Pos < 6){ Pos++; } else { Pos=0; } }
    else{ if(Pos >0){ Pos--; } else { Pos=6; } }
    dispPos = 1;
    ctlTime = millis();
    casvyp = millis();
 timer = 1;
  }  
}
// TLACIDLO ENKODERA //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void encPush(){ //switch ISR
  if(((millis()-ctlTime) > ODOZVA) && (dispSW == 0)) 
  {
   if(poleVelicina[Pos]<5) { poleVelicina[Pos]++; } else { poleVelicina[Pos] = 1; }
    dispSW = 1;
    ctlTime = millis();
    casvyp= millis();
    timer=1;  
  }  
}
// RESET SOFTVEROVY ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void softReset(){
asm volatile ("  jmp 0");
}
// DOPLNOVANIE NUL ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void printSpace(float fNum){
 uint8_t noZ=0;

if((fNum< 10000.0)&&(fNum>=1000.0)){noZ=1;}
if((fNum< 1000.0)&&(fNum>=100.0)){noZ=2;}
if((fNum< 100.0)&&(fNum>=10.0)){noZ=3;}
if((fNum< 10.0)&&(fNum>=0.0)){noZ=4;}
if((fNum>-1000.0)&&(fNum<=-100.0)){noZ=1;}
if((fNum>-100.0)&&(fNum<=-10.0)){noZ=2;}
if((fNum>-10.0)&&(fNum<=-0.01)){noZ=3;}
  switch(noZ){
    case 1: lcd.print(""); break;
    case 2: lcd.print(" "); break;
    case 3: lcd.print("  "); break;
    case 4: lcd.print("   "); break;
  }
}
void loop() {
if(Ethernet.begin(mac)==0){ Ethernet.begin(mac,ip); }
  if((millis()-casvyp)<cas){
  lcd.backlight(); //lcd.backlight();
  }
  else{
  lcd.backlight(); //lcd.noBacklight();
  }    
// ZOBRAZENIE HODNOT NA DISPLEJ POMOCOU ENKODERA //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(dispPos)
  {
    iznak=poleznak[prevPos];
    iriadok=poleriadok[prevPos];
    lcd.setCursor(iznak,iriadok);
    lcd.print(" ");
 iznak=poleznak[Pos];
    iriadok=poleriadok[Pos];
    lcd.setCursor(iznak,iriadok);
    lcd.print(">");
    dispPos = 0;
  }
  for(i = 0; i < 7; i++)
   { 
    iznak=poleznak[i];
    iriadok=poleriadok[i];
    int iznak2=iznak+1;
    lcd.setCursor(iznak2,iriadok);
    
    switch(iriadok) 
    { 
      case 0:
                switch (poleVelicina[i]) 
                { 
                            case 1: lcd.print(napatie3,2);lcd.print("V ");printSpace(napatie3); break;
                            case 2: lcd.print(prud,2);lcd.print("A "); printSpace(prud); break;
                            case 3: lcd.print(vykon,2);lcd.print("W "); printSpace(vykon); break;
                            case 4: lcd.print(kapacita,2);lcd.print("Ah"); printSpace(kapacita); break;
                            case 5: lcd.print(watthodiny,2);lcd.print("Wh"); printSpace(watthodiny); break;
                }
      break;
      case 1:
                switch (poleVelicina[i]) 
                { 
                  
                            case 1: lcd.print(napatie2,2);lcd.print("V ");printSpace(napatie2); break;
                            case 2: lcd.print(prud1,2);lcd.print("A "); printSpace(prud1); break;
                            case 3: lcd.print(vykon1,2);lcd.print("W "); printSpace(vykon1); break;
                            case 4: lcd.print(kapacita1,2);lcd.print("Ah"); printSpace(kapacita1); break;
                            case 5: lcd.print(watthodiny1,2);lcd.print("Wh"); printSpace(watthodiny1); break;  
                }
      break;
      case 2:
                switch (poleVelicina[i]) 
                { 
                            case 1: lcd.print(napatie,2);lcd.print("V ");printSpace(napatie); break;
                            case 2: lcd.print(prud2,2);lcd.print("A "); printSpace(prud2); break;
                            case 3: lcd.print(vykon2,2);lcd.print("W "); printSpace(vykon); break;
                            case 4: lcd.print(kapacita2,2);lcd.print("Ah"); printSpace(kapacita2); break;
                            case 5: lcd.print(watthodiny2,2);lcd.print("Wh"); printSpace(watthodiny2); break;   
                }
      break;
      case 3:
                switch (poleVelicina[i]) 
                { 
                            case 1:lcd.write(byte(0));if (digitalRead(MOSFET) == 0){lcd.print("VYP ");}else{lcd.print("ZAP ");}
                                   lcd.write(byte(1));if (digitalRead(MOSFET1) == 1){lcd.print("VYP ");}else{lcd.print("ZAP ");}
                                   lcd.write(byte(2));if (digitalRead(MOSFET2) == 0){lcd.print("VYP ");}else{lcd.print("ZAP ");}
                                   lcd.write(byte(3));if (digitalRead(RELE) == 0){lcd.print("VYP");}else{lcd.print("ZAP");}; break;            
                            case 2:prepocet=napatie-10.5;
                                   vysledok=(prepocet*100)/2.3;//hodnota 2.3 je rozdiel minimálnej a maximálnej hodnoty baterie 12.8-10.5=2.3
                                   if(prepocet<0){lcd.print("BATERIA VYBITA !!! ");}
                                   else if(prepocet>2.4){lcd.print("BATERIA NABITA !!! ");}
                                   else{lcd.write(byte(4));lcd.print(":");lcd.print(vysledok,2);lcd.print("%         ");printSpace(vysledok);} break;
                            case 3: lcd.print(hod);lcd.print(" hodiny");break;
                            case 4: lcd.print(hod);lcd.print(" hodiny"); break;
                            case 5: lcd.print(hod);lcd.print(" hodiny"); break;       
                            
                }
      break;    
    }
  dispSW = 0;  
  }
// VYPOCET MERANEHO NAPATIA 1 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int analog_value = analogRead(napatiepin);
    vst_napatie = analog_value * 0.0048828125; 
   napatie = (vst_napatie / 0.32);
   if(napatie<0){
    napatie=0;
    } 
// VYPOCET MERANEHO NAPATIA 2 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   int analog_value1 = analogRead(napatiepin1);
    napatie1 = analog_value1 * (5.0/ 1024.0);
// VYPOCET NAPATIA 3 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    napatie2=napatie-napatie1;
    if(napatie2<0){
    napatie2=0;
    }
// VYPOCET NAPATIA 4 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if(prud>0.1){napatie3=napatie2+2;}
else{napatie3=0;}
// VYPOCET MERANEHO PRUDU 1 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
     analog = analogRead(prudpin);// nacitanie hodnoty analogového vstupu
     napatieprud = analog * 4.8828125;  // prepocet napatia na prud podla informacii od vyrobcu
     prud = (napatieprud - offset) / konstanta;
    
  if(prud<0){
    prud=0;
  }
//VYPOCET MERANEHO PRUDU 2 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    analog1 = analogRead(prudpin1); // nacitanie hodnoty analogového vstupu
    napatieprud1 = analog1 * 4.8828125; // prepocet napatia na prud podla informacii od vyrobcu
    prud1 = (napatieprud1 - offset) / konstanta;  
//VYPOCET MERANEHO PRUDU 2 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    analog2 = analogRead(prudpin2); // nacitanie hodnoty analogového vstupu
    napatieprud2 = analog2 * 4.8828125; // prepocet napatia na prud podla informacii od vyrobcu
    prud2 = (napatieprud2 - offset) / konstanta;
// VYPOCET OSTATNYCH VELICIN //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  sec = millis()/1000;
  hod = millis()/3600000;
  vykon = napatie3*prud;
  vykon1 = napatie2*prud1; 
  vykon2 = napatie*prud2;   
  akapacita = prud*hod;
  akapacita1 = prud1*hod;
  akapacita2 = prud2*hod;
  kapacita=kapacita+akapacita;
  kapacita1=kapacita1+akapacita1;
  kapacita2=kapacita2+akapacita2;
  awatthodiny=vykon*hod; 
  awatthodiny1=vykon1*hod;
  awatthodiny2=vykon2*hod;
  watthodiny=watthodiny+awatthodiny;
  watthodiny1=watthodiny1+awatthodiny1;
  watthodiny2=watthodiny2+awatthodiny2;
// SPOJENIE S WEB STRANKOU A ODOSLANIE HODNOT ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
   if (client.connect(server, 80)){
  client.println("GET /Mano/values/reset.txt HTTP/1.1");
  client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();             
  }
  while(client.connected() && !client.available()) 
  while (client.connected() || client.available()) { //pramenna
    char c = client.read(); //dostan bity z buffera
    if (c==lf) x=(x+1); //pocitaj 
    else if (x==12) readString += c; //nasa premenna
   } 
if(readString=="RST"){
  client.stop(); //ukonc spojenie
    if (client.connect(server, 80)){
  client.println("GET /Mano/arduino/reset_vykonany.php HTTP/1.1");
  client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println(); 
    softReset();            
  }
  }
  readString = ("");
  x=0;
  client.stop(); //ukonc spojenie
  if (client.connect(server, 80)){
  client.println("GET /Mano/values/solar.txt HTTP/1.1");
  client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();             
  }
  while(client.connected() && !client.available()) 
  while (client.connected() || client.available()) { //pramenna
    char c = client.read(); //dostan bity z buffera
    if (c==lf) x=(x+1); //pocitaj 
    else if (x==12) readString += c; //nasa premenna
   } 
if(readString=="ZAP"){
  digitalWrite(MOSFET, LOW); 
  }else if(readString=="VYP"){
  digitalWrite(MOSFET, HIGH);  
    }    
  readString = ("");
  x=0;
  client.stop(); //ukonc spojenie

 if (client.connect(server, 80)){
  client.println("GET /Mano/values/bateria.txt HTTP/1.1");
  client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();             
  }
  while(client.connected() && !client.available()) 
  while (client.connected() || client.available()) { //pramenna
    char c = client.read(); //dostan bity z buffera
    if (c==lf) x=(x+1); //pocitaj 
    else if (x==12) readString += c; //nasa premenna
   } 
if(readString=="ZAP"){
  digitalWrite(MOSFET1, LOW); 
  }else if(readString=="VYP"){
  digitalWrite(MOSFET1, HIGH);  
    }    
  readString = ("");
  x=0;
  client.stop(); //ukonc spojenie

   if (client.connect(server, 80)){
  
  client.println("GET /Mano/values/spotrebic.txt HTTP/1.1");
  client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();             
    
  }
  while(client.connected() && !client.available()) 
  while (client.connected() || client.available()) { //pramenna
    char c = client.read(); //dostan bity z buffera
    if (c==lf) x=(x+1); //pocitaj 
    else if (x==12) readString += c; //nasa premenna

   } 
if(readString=="ZAP"){
  digitalWrite(MOSFET2, HIGH); 
  }else if(readString=="VYP"){
  digitalWrite(MOSFET2, LOW);  
    }    
  readString = ("");
  x=0;
  client.stop(); //ukonc spojenie
  
  if (client.connect(server, 80)){
  
  client.println("GET /Mano/values/svetlo.txt HTTP/1.1");
  client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();             
    
  }
  while(client.connected() && !client.available()) 
  while (client.connected() || client.available()) { //pramenna
    char c = client.read(); //dostan bity z buffera
    if (c==lf) x=(x+1); //pocitaj 
    else if (x==12) readString += c; //nasa premenna

   } 
if(readString=="ZAP"){
  digitalWrite(MOSFET3, HIGH); 
  }else if(readString=="VYP"){
  digitalWrite(MOSFET3, LOW);  
    }    
  readString = ("");
  x=0;
  client.stop(); //ukonc spojenie

  if (client.connect(server, 80)) {               // AK SA NAPOJI NA SERVER NA PORTE 80 (HTTP)
    client.print("GET /Mano/arduino/zapistabulku.php?solarnapatie=");         
    client.print(napatie3);   
    client.print("&solarprud=");                     
    client.print(prud);    
    client.print("&solarvykon=");                      
    client.print(vykon);         
    client.print("&solarah=");                     
    client.print(kapacita); 
    client.print("&solarwh=");                    
    client.print(watthodiny);
    client.print("&baterianapatie=");
    client.print(napatie2);   
    client.print("&bateriaprud=");                     
    client.print(prud1);    
    client.print("&bateriavykon=");                      
    client.print(vykon1);         
    client.print("&bateriaah=");                     
    client.print(kapacita1); 
    client.print("&bateriawh=");                    
    client.print(watthodiny1);
    client.print("&spotrebicnapatie=");
    client.print(napatie);   
    client.print("&spotrebicprud=");                     
    client.print(prud2);    
    client.print("&spotrebicvykon=");                      
    client.print(vykon2);         
    client.print("&spotrebicah=");                     
    client.print(kapacita2); 
    client.print("&spotrebicwh=");                    
    client.print(watthodiny2); 
    client.print("&cas=");               
    client.print(sec);
    client.print("&siet=");  
if (digitalRead(RELE) == 0){client.print("VYP");}else{client.print("ZAP");}
    client.println(" HTTP/1.1");                 // UKONCENIE REQUESTU ZALOMENIM RIADKA A DOPLNENIM HLAVICKY HTTP S VERZIOU
    client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();                            //ZALOMENIE RIADKA KLIENTSKEHO ZAPISU
    client.stop();                                   // UKONCENIE PRIPOJENIA ETHERNET SHIELDU
  }  
//OVLADANIE TLACIDLA  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if (digitalRead(TLACITKO) == 0)           // či je tlačitko zopnute
  {                                                  
    if (MinulyStav == 1 && millis() - CasZmeny > ODOZVA) // či je nejaká odozva         
    {                                          
                                               // od zmeny stavu tlacitka je (50ms)
      MinulyStav = 0;                             // nastav priznak tlacitko stlacene
      digitalWrite(MOSFET, !digitalRead(MOSFET));       // zmen stav MOSFET
              if(digitalRead(MOSFET) == 1){
    if (client.connect(server, 80)) {               // AK SA NAPOJI NA SERVER NA PORTE 80 (HTTP)
    client.print("GET /Mano/zap_solar.php");         
    client.println(" HTTP/1.1");                 // UKONCENIE REQUESTU ZALOMENIM RIADKA A DOPLNENIM HLAVICKY HTTP S VERZIOU
    client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();                            //ZALOMENIE RIADKA KLIENTSKEHO ZAPISU
    client.stop();                                   // UKONCENIE PRIPOJENIA ETHERNET SHIELDU
     
  }
  }else if (digitalRead(MOSFET) == 0){
    
       if (client.connect(server, 80)) {               // AK SA NAPOJI NA SERVER NA PORTE 80 (HTTP)
    client.print("GET /Mano/vyp_solar.php");         
    client.println(" HTTP/1.1");                 // UKONCENIE REQUESTU ZALOMENIM RIADKA A DOPLNENIM HLAVICKY HTTP S VERZIOU
    client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();                            //ZALOMENIE RIADKA KLIENTSKEHO ZAPISU
    client.stop();                                   // UKONCENIE PRIPOJENIA ETHERNET SHIELDU
     
  }
    } 
    }
  }
  else                                      // nieje stlacene tlacitko
  {
    CasZmeny = millis();                       // zapamataj si posMOSFETny cas od kedy nebolo stlacene tlacitko
    MinulyStav = 1;                            // nuluj priznak, tlacitko stlacene 
 
}
//OVLADANIE TLACIDLA 1  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if (digitalRead(TLACITKO1) == 0)           // či je tlačitko zopnute
  {                                                  
    if (MinulyStav1 == 1 && millis() - CasZmeny1 > ODOZVA) // či je nejaká odozva         
    {                                          
                                               // od zmeny stavu tlacitka je (50ms)
      MinulyStav1 = 0;                             // nastav priznak tlacitko stlacene
      //digitalWrite(BAT, !digitalRead(BAT));
      digitalWrite(MOSFET1, !digitalRead(MOSFET1));       // zmen stav MOSFET
      if(digitalRead(MOSFET1) == 1){
    if (client.connect(server, 80)) {               // AK SA NAPOJI NA SERVER NA PORTE 80 (HTTP)
    client.print("GET /Mano/vyp_bateria.php");         
    client.println(" HTTP/1.1");                 // UKONCENIE REQUESTU ZALOMENIM RIADKA A DOPLNENIM HLAVICKY HTTP S VERZIOU
    client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();                            //ZALOMENIE RIADKA KLIENTSKEHO ZAPISU
    client.stop();                                   // UKONCENIE PRIPOJENIA ETHERNET SHIELDU
     
  }
  }else if (digitalRead(MOSFET1) == 0){
    
       if (client.connect(server, 80)) {               // AK SA NAPOJI NA SERVER NA PORTE 80 (HTTP)
    client.print("GET /Mano/zap_bateria.php");         
    client.println(" HTTP/1.1");                 // UKONCENIE REQUESTU ZALOMENIM RIADKA A DOPLNENIM HLAVICKY HTTP S VERZIOU
    client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();                            //ZALOMENIE RIADKA KLIENTSKEHO ZAPISU
    client.stop();                                   // UKONCENIE PRIPOJENIA ETHERNET SHIELDU
     
  }
    }      
    }
  }
  else                                      // nieje stlacene tlacitko
  {
    CasZmeny1 = millis();                       // zapamataj si posMOSFETny cas od kedy nebolo stlacene tlacitko
    MinulyStav1 = 1;                            // nuluj priznak, tlacitko stlacene 
 
}
//OVLADANIE TLACIDLA 2 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if (digitalRead(TLACITKO2) == 0)           // či je tlačitko zopnute
  {                                                  
    if (MinulyStav2 == 1 && millis() - CasZmeny2 > ODOZVA) // či je nejaká odozva         
    {                                          
                                               // od zmeny stavu tlacitka je (50ms)
      MinulyStav2 = 0;                             // nastav priznak tlacitko stlacene
      digitalWrite(MOSFET2, !digitalRead(MOSFET2));       // zmen stav MOSFET
      a=1;
      err_prev=0;
      if(digitalRead(MOSFET2) == 1){
    if (client.connect(server, 80)) {               // AK SA NAPOJI NA SERVER NA PORTE 80 (HTTP)
    client.print("GET /Mano/zap_spotrebic.php");         
    client.println(" HTTP/1.1");                 // UKONCENIE REQUESTU ZALOMENIM RIADKA A DOPLNENIM HLAVICKY HTTP S VERZIOU
    client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();                            //ZALOMENIE RIADKA KLIENTSKEHO ZAPISU
    client.stop();                                   // UKONCENIE PRIPOJENIA ETHERNET SHIELDU
     
  }
  }else if (digitalRead(MOSFET2) == 0){
    
       if (client.connect(server, 80)) {               // AK SA NAPOJI NA SERVER NA PORTE 80 (HTTP)
    client.print("GET /Mano/vyp_spotrebic.php");         
    client.println(" HTTP/1.1");                 // UKONCENIE REQUESTU ZALOMENIM RIADKA A DOPLNENIM HLAVICKY HTTP S VERZIOU
    client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();                            //ZALOMENIE RIADKA KLIENTSKEHO ZAPISU
    client.stop();                                   // UKONCENIE PRIPOJENIA ETHERNET SHIELDU 
  }
    }      
    }
  }
  else                                      // nieje stlacene tlacitko
  {
    CasZmeny2 = millis();                       // zapamataj si posMOSFETny cas od kedy nebolo stlacene tlacitko
    MinulyStav2 = 1;                            // nuluj priznak, tlacitko stlacene 
 
}
//OVLADANIE TLACIDLA 3 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if (digitalRead(TLACITKO3) == 0)           // či je tlačitko zopnute
  {                                                  
    if (MinulyStav3 == 1 && millis() - CasZmeny3 > ODOZVA) // či je nejaká odozva         
    {                                          
                                               // od zmeny stavu tlacitka je (50ms)
      MinulyStav3 = 0;                             // nastav priznak tlacitko stlacene
      digitalWrite(MOSFET3, !digitalRead(MOSFET3));       // zmen stav MOSFET
      if(digitalRead(MOSFET3) == 1){
    if (client.connect(server, 80)) {               // AK SA NAPOJI NA SERVER NA PORTE 80 (HTTP)
    client.print("GET /Mano/zap_svetlo.php");         
    client.println(" HTTP/1.1");                 // UKONCENIE REQUESTU ZALOMENIM RIADKA A DOPLNENIM HLAVICKY HTTP S VERZIOU
    client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();                            //ZALOMENIE RIADKA KLIENTSKEHO ZAPISU
    client.stop();                                   // UKONCENIE PRIPOJENIA ETHERNET SHIELDU
     
  }
  }else if (digitalRead(MOSFET3) == 0){
    
       if (client.connect(server, 80)) {               // AK SA NAPOJI NA SERVER NA PORTE 80 (HTTP)
    client.print("GET /Mano/vyp_svetlo.php");         
    client.println(" HTTP/1.1");                 // UKONCENIE REQUESTU ZALOMENIM RIADKA A DOPLNENIM HLAVICKY HTTP S VERZIOU
    client.println("Host: www.arduino.php5.sk"); // ADRESA HOSTA, NA KTOREHO BOL MIERENY REQUEST (NIE PHP SUBOR)
    client.println("Connection: close");         //UKONCENIE PRIPOJENIA ZA HTTP HLAVICKOU
    client.println();                            //ZALOMENIE RIADKA KLIENTSKEHO ZAPISU
    client.stop();                                   // UKONCENIE PRIPOJENIA ETHERNET SHIELDU
     
  }
    }      
    }
  }
  else                                      // nieje stlacene tlacitko
  {
    CasZmeny3 = millis();                       // zapamataj si posMOSFETny cas od kedy nebolo stlacene tlacitko
    MinulyStav3 = 1;                            // nuluj priznak, tlacitko stlacene 
 
}
// PREPINANIE MEDZI BATERKOU A SIETOU /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if(stateSteady==1){
  if(napatie2<10.5){
    reqState=AConBAToff; supplyChange = AConBAToff; stateSteady=0; AConTime=millis();}
  else if((napatie2>12)&& ((millis()-AConTime)>10000)){
    reqState=ACoffBATon; supplyChange = ACoffBATon; stateSteady=0;}
}

if(stateSteady==0){
  switch (reqState)
  {
    case AConBATon:    digitalWrite(RELE,HIGH);digitalWrite(MOS,HIGH);digitalWrite(BAT,HIGH);   //state 3
                       curState = AConBATon; reqState = supplyChange; 
                       break;
                  
    case ACoffBATon:   if(curState == AConBAToff){reqState = AConBATon;}  //state 2
                       else{digitalWrite(RELE, LOW);digitalWrite(MOS,LOW);digitalWrite(BAT,HIGH); curState = ACoffBATon; stateSteady=1;}
                       break;
                  
    case AConBAToff:   if(curState == ACoffBATon){reqState = AConBATon;} //state 1
                       else{digitalWrite(RELE,HIGH);digitalWrite(MOS,HIGH);digitalWrite(BAT, LOW); curState = AConBAToff; stateSteady=1;}
                       break;
    
    default:           digitalWrite(RELE,HIGH);digitalWrite(MOS,HIGH);digitalWrite(BAT,HIGH); stateSteady=1;//safe mode 
  }
}
// AUTOMATICKE ODPAJANIE SPOTREBICOV //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if(err_prev==0){
  if(prud2<=(-4)){
   digitalWrite(MOSFET2, LOW);
   err_prev=1;
   err_millis=millis();
   a++;
   }/*
   }
   else{ if((millis()-err_millis)>5000){ 
    if(a<4){
   digitalWrite(MOSFET2, HIGH);
   err_prev=0;
    }   
  }*/
  }
// ULOZENIE VOLBY NA DISPLEJ DO EEPROM PAMATE /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
for(i=0;i<7;i++){
    EEPROM.update(i,poleVelicina[i]);
  }      
}
