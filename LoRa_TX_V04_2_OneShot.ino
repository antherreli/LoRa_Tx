//======================================================================
/*
    Envia os dados brutos do sensor uma vez a cada X minutos.
    existe um WDT ativo d e5 segundos 
    após cada cilclo o sistema entre em deeep sleep visando o baixo consumo de energia 

    Giovanni Antherreli   
    16/10/2025


*/



//=============BIBLIOTECAS========================================//



#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "LoRaConfig.h" //arquivo com configurações do radio LoRa
#include <EEPROM.h>
#include "sd_read_write.h"

#include <WiFi.h>
#include <BluetoothSerial.h>

#include "driver/adc.h"
#include <esp_bt.h>

#include "esp_system.h"
#include "rom/ets_sys.h"

#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#include "esp32/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C2
#include "esp32c2/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C6
#include "esp32c6/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32H2
#include "esp32h2/rom/rtc.h"
#else 
#error Target CONFIG_IDF_TARGET is not supported
#endif

//================================================================//

// gerenciamento de rede
#define SensorADDR     0x0B //endereço nó sensor 
#define MAC_ADDR   0xF5 //byte de controle para RX



#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10    /* Time ESP32 will go to sleep (in seconds) */
//RTC_DATA_ATTR volatile int bootCount = 0;
RTC_DATA_ATTR volatile int counter = 0;


const int wdtTimeout = 20000;  //tempo em ms para ativar watchdog
hw_timer_t * timer = NULL;

//============HARDWARE===========================================//
//PINOS PARA UART VIRTUAL 
#define rxPin         34
#define txPin         33
//define momento de leitura e escrita DO SENSOR habilitando esses pinos 
#define RE            47
#define DE            8 
//controle do sensor 
#define sensorControl 48  //habilita mosfet Q3 alimentando os sensores 
#define Vext_Ctrl     36  //não utilizado 
#define bussControl   45  //habilita o barramento RS485
#define LED           35  // não utilizado
//================================================================//

//OBJETO DA UART VIRTUAL
HardwareSerial mod(1);

//objeto SPI
SPIClass sd_spi(HSPI); //SPI para controle do módulo SD card



//===========================state machine===================================//

#define READ_SENSOR       0           //lê dados dos sensores 
#define DATA_MANAGEMENT   1           //gerencia dados em cartão SD, caso necessário
#define TX                2           //transmite buffer TX
#define LOW_POWER         3           //entra em modo deep sleep para poupar bateria
#define WAIT              4           //agurada até o próximo ciclo de leitura

 
//variaveis globais:
volatile char state = READ_SENSOR;  //variável responsável por armazenar o estado atual da máquina de estados

//==================VARIÁVEIS GLOBAIS =============================//
double txNumber;

bool lora_idle=true; 
bool readStatus = false; 
volatile uint8_t txCounter = 0;

//buffer auxiliar para transmissão de dados
volatile uint8_t bufferAux01[11] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //bufer de leitura auxiliar para sensor 02
volatile uint8_t bufferAux02[11] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //buffer de leitura auxiliar para sensor 02
volatile uint8_t  bufferEC[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0}; // buffer de leitura de consutividade do sensor 02
volatile uint8_t bufferTx[14] = { 0x00, 0x00, //reservado para endereços 
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //slot para dados do sensor 01
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //slot para dados do sensor 02
                             

String dataMessage;
 
//================================================================//



//PROTÓTIPO DAS FUNÇÕES DE CALBACK DO RÁDIO LORA 
static RadioEvents_t RadioEvents;
void OnTxDone( void );      //ESSA FUNÇÃO É EXECTUADA QUANDO A TRANSMISSÃO É FINALIZADA
void OnTxTimeout( void );   //ESSA FUNÇÃO É EXECTUADA QUANDO O RECEPTOR NÃO RESPONDE DENTRO DO TEMPO PRE DERTEMINADO 

//PROTÓTIPO DAS FUNÇÕES AUXILIARES
bool wait(unsigned long tempoEsperado);
void readSensor(void);   //lê os dados do sensor 01 e armazena em buffer local
void readSensor02(void);  //lê os dados do sensor 02 e armazena em buffer local
void writeEEPROM (uint8_t *buffer, uint8_t size, uint8_t ADDR); //escreve os dados do sensor na EEPROM interna  
void sdInit(void); 
void gravaDados(volatile uint8_t size, uint8_t counter); 
void disableWiFi();
void disableBluetooth();
void ARDUINO_ISR_ATTR resetModule(); // Função de calback chamada após o disparo do WDT
void print_wakeup_reason();
void print_reset_reason(int reason);
void sensorON(void);
void sensorOFF(void);
void carregaBufferTX(void);
void debugTx(void);




void setup() {
    
    


    //modem sleep 
    disableWiFi();
    disableBluetooth();

    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    mod.begin(9600, SERIAL_8N1, rxPin, txPin);
    //print_wakeup_reason();
    Serial.printf("Endereco sensor: %x\n", SensorADDR);
    Serial.println("CPU0 reset reason:");
    print_reset_reason(rtc_get_reset_reason(0));
    //nincializa cartão SD
    sdInit();

      // If the data.txt file doesn't exist
    // Create a file on the SD card and write the data labels
    /*File file = SD.open("/data.txt");
    if(!file) {
      Serial.println("File doesn't exist");
      Serial.println("Creating file...");
      writeFile(SD, "/data.txt", "Counter, Temperature, Humidity, Condutivity \r\n");
    }
    else {
      Serial.println("File already exists");  
    }
    file.close();*/
    

    //Increment boot number and print it every reboot
    //++bootCount;
    //Serial.println("Boot number: " + String(bootCount));
    state = READ_SENSOR;
    
    //inicialização dos pinos de controle 
    
    pinMode(RE, OUTPUT);
    digitalWrite(RE, LOW);
    
    pinMode(sensorControl, OUTPUT);
    digitalWrite(sensorControl, HIGH);

    pinMode(bussControl, OUTPUT);
    digitalWrite(bussControl, LOW);

    pinMode(Vext_Ctrl, OUTPUT);
    digitalWrite(Vext_Ctrl, HIGH);

    //pinMode(LED, OUTPUT);
    //digitalWrite(LED, HIGH);


    delay(10);
    

    //delay(100);
 
	
    txNumber=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 

    
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    timer = timerBegin(1000000);                   //timer 1Mhz resolution
    timerAttachInterrupt(timer, &resetModule);           //attach callback
    timerAlarm(timer, wdtTimeout * 1000, false, 0); //set time in us
   }

  

   

void loop()
{
     
    
    switch(state){

        case READ_SENSOR:
          timerWrite(timer, 0); //reset timer (feed watchdog)
          delay(10);
          
          sensorON();
          readSensor();
          readSensor02();
          sensorOFF();
          
          state = DATA_MANAGEMENT;
          ++counter; 
          break;

        case DATA_MANAGEMENT:

          timerWrite(timer, 0); //reset timer (feed watchdog)
          //gravaDados((uint8_t *)bufferAux, 11, counter);      //grava dados de temperature e humidade em formato cvs para backup
          carregaBufferTX();
          Serial.println("Dados Armazenados...");
          
         state = TX;
          
          break;  
        
        case TX:
          timerWrite(timer, 0); //reset timer (feed watchdog)
          delay(10);
          //txCounter++;
          //bufferAux[11] = SensorADDR_;
          debugTx();
          Radio.Send( (uint8_t *)bufferTx, 14);
          //state = LOW_POWER;
         
          state=WAIT;
          break;

        case LOW_POWER:
          
          Serial.println("Go to Sleep, MR Eames " + String(TIME_TO_SLEEP) +" Seconds");
          esp_deep_sleep_start();
          
          break;

        case WAIT:
          timerWrite(timer, 0); //reset timer (feed watchdog)
          Radio.IrqProcess( );
          //DOES NOTHING  
          //Serial.println("waiting...");
          break;

        default:
          break;  


    }
    
	
}



// =====================================================================
// Funções auxiliares
// ======================================================================

void OnTxDone( void )
{
	Serial.println("TX done......");
  for (uint8_t i=0; i<14; i++) bufferTx[i] = 0x00;
  state = LOW_POWER;
	lora_idle = true;
  
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
    state = LOW_POWER;
}



void readSensor(void)
{
 
  
  

  // =========================
  //  Sensor 01
  // =========================
  /*volatile const uint8_t MS[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x03, 0x05, 0xF8};
  volatile uint8_t bufferInterno[11]; //buffer auxiliar para leitura de dados do sensor
  
  //habilita o barramento para transmissão
  digitalWrite(RE, HIGH);
  
  for (uint8_t i=0; i<8; i++) mod.write(MS[i]); 
  mod.flush(); 

  //habilita o barramento para leitura
  digitalWrite(RE, LOW);
  //digitalWrite(DE, LOW);

  delay(500);

  uint8_t c = 0; 
  while(mod.available()){
    
    if(c < 11) bufferAux01[c++] = mod.read(); //armazena dados do sensor em buffer temporário
  }*/
  //teste para validação 
  uint8_t frame[11] = {
    0x01,
    0x03,
    0x06,
    0x02, 0x92,
    0xFF, 0x9B,
    0x03, 0xE8,
    0x38, 0x75
  };

  for(uint8_t i = 0; i<11; i++){
    bufferAux01[i] = frame[i];
  }


  

}



void sdInit(void)
{
  sd_spi.begin(SCK, MISO, MOSI, CS);

  if (!SD.begin(CS,sd_spi)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

}

void gravaDados(uint8_t *buffer, uint8_t size, uint8_t counter)
{

  uint16_t buff = ((uint16_t)(buffer[3] << 8 ) & 0xFF00) | buffer[4];
  float humidity = (float)buff / 10; 

  buff = ((uint16_t)(buffer[5] << 8 ) & 0xFF00) | buffer[6];
  float temperature = (float)buff / 10;
  temperature  = (buff & 0x1000) ? (6553.4 - temperature) * -1 : temperature;

  uint16_t conductivity = ((uint16_t)(buffer[7] << 8 ) & 0xFF00) | buffer[8];
  Serial.println("Sensoe 02: ");
  Serial.println("Humidity: \t" + String(humidity));
  Serial.println("Temperature: \t" + String(temperature));
  Serial.println("Conductivity: \t" + String(conductivity));

  //Concatenate all info separated by commas
    //dataMessage = String(counter) + "," + String(temperature) + "," + String(humidity) + "," + String(conductivity)+ "\r\n";
    //.print("Saving data: ");
    //Serial.println(dataMessage);

    //Append the data to file
    //appendFile(SD, "/data.txt", dataMessage.c_str());

    //for (uint8_t i=0; i<11; i++) bufferAux[i] = 0x00;

}

void disableWiFi(){
    //adc_power_off();
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
    Serial2.println("");
    Serial.println("WiFi disconnected!");
}
void disableBluetooth(){
    // Quite unusefully, no relevable power consumption
    btStop();
    Serial.println("");
    Serial.println("Bluetooth stop!");
}

void ARDUINO_ISR_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}  

void print_reset_reason(int reason)
{
  switch ( reason)
  {
    case 1 : Serial.println ("POWERON_RESET");break;          /**<1,  Vbat power on reset*/
    case 3 : Serial.println ("SW_RESET");break;               /**<3,  Software reset digital core*/
    case 4 : Serial.println ("OWDT_RESET");break;             /**<4,  Legacy watch dog reset digital core*/
    case 5 : Serial.println ("DEEPSLEEP_RESET");break;        /**<5,  Deep Sleep reset digital core*/
    case 6 : Serial.println ("SDIO_RESET");break;             /**<6,  Reset by SLC module, reset digital core*/
    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;       /**<7,  Timer Group0 Watch dog reset digital core*/
    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;       /**<8,  Timer Group1 Watch dog reset digital core*/
    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9,  RTC Watch dog Reset digital core*/
    case 10 : Serial.println ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : Serial.println ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
    
    
    case 12 : Serial.println ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : Serial.println ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : Serial.println ("NO_MEAN");
  }
}

void readSensor02(void)
{
  
  
  // =========================
  //  Sensor 02
  // ========================= 

  //readStatus = false;
  volatile const uint8_t MS[] =     {0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x65, 0xCB};     //frame para leitura de umidade e temperatura
  volatile const uint8_t CMD_EC[] = {0x01, 0x03, 0x00, 0x15, 0x00, 0x01, 0x95, 0xCE}; //frame para leitura de condutividade 

  //habilita o barramento para transmissão
  digitalWrite(RE, HIGH);
  
  for (uint8_t i=0; i<8; i++) mod.write(MS[i]); 
  mod.flush(); 

  //habilita o barramento para leitura
  digitalWrite(RE, LOW);
 

  delay(500);

  uint8_t c = 0; 
  while(mod.available()){
    
    if(c < 9) bufferAux02[c++] = mod.read(); 
    
  }

  
  
  
  // =========================
  // CONDUTIVIDADE
  // =========================

  // Limpa buffer serial
 while(mod.available()) mod.read();

  //habilita o barramento para transmissão
  digitalWrite(RE, HIGH);
  
  for (uint8_t i=0; i<8; i++)
  { 
    mod.write(CMD_EC[i]); 
  }
  mod.flush(); 

  //habilita o barramento para leitura
  digitalWrite(RE, LOW);
  delay(500);

  
  uint8_t Ec = 0; 
  while(mod.available()){
    
    if(Ec < 7) 
    {
      bufferEC[Ec++] = mod.read(); 
    }
    
  }

  


}

void sensorON(void)
{
  //liga sensor
  //habilita barramento de leitura 
  digitalWrite(bussControl, HIGH);
  delay(50); 
  digitalWrite(sensorControl, HIGH);
  delay(2000);

}

void sensorOFF(void)
{
  digitalWrite(bussControl, LOW);
  digitalWrite(sensorControl, LOW);
  delay(50); 
}

void carregaBufferTX(void)
{
  bufferTx[0] = MAC_ADDR;
  bufferTx[1] = SensorADDR;
  // Sensor 1 -> bytes 2 a 7
          for(uint8_t i =2; i < 8; i++)
          {
             bufferTx[i] = bufferAux01[i+1];
          }
  // Sensor 1 -> bytes 8 a 13
          for(uint8_t i =8; i < 12; i++)
          {
             bufferTx[i] = bufferAux02[i-5];
          }       
  bufferTx[12] = bufferEC[3];
  bufferTx[13] = bufferEC[4];
   

}

void debugTx(void)

{

  //Serial.printf("Endereco sensor: %x\n", SensorADDR);
  Serial.println("Frame Tx:");
  for(uint i = 0; i<14; i++) 
  {
    if (bufferTx[i] < 0x10) Serial.print("0");
    Serial.print(bufferTx[i],HEX);
    Serial.print(" ");
  }
  Serial.println();

  uint16_t buff = ((uint16_t)(bufferTx[2] << 8 ) & 0xFF00) | bufferTx[3];
  float humidity = (float)buff / 10; 

  buff = ((uint16_t)(bufferTx[4] << 8 ) & 0xFF00) | bufferTx[5];
  float temperature = (float)buff / 10;
  temperature  = (buff & 0x1000) ? (6553.4 - temperature) * -1 : temperature;
  
  uint16_t conductivity = ((uint16_t)(bufferTx[6] << 8) & 0xFF00) | bufferTx[7];
  Serial.println("===========Sensor 01================");
  Serial.println("Humidity: \t" + String(humidity));
  Serial.println("Temperature: \t" + String(temperature));
  Serial.println("Conductivity: \t" + String(conductivity));

  //====================================================================
  uint16_t buff_ = ((uint16_t)(bufferTx[8] << 8 ) & 0xFF00) | bufferTx[9];
  float humidity_ = (float)buff_ / 10; 

  buff_ = ((uint16_t)(bufferTx[10] << 8 ) & 0xFF00) | bufferTx[11];
  float temperature_ = (float)buff_ / 10;
  temperature_  = (buff_ & 0x1000) ? (6553.4 - temperature_) * -1 : temperature_;
  uint16_t conductivity_ = ((uint16_t)(bufferTx[12] << 8) & 0xFF00) | bufferTx[13];
  Serial.println("===========Sensor 02================");
  Serial.println("Humidity: \t" + String(humidity_));
  Serial.println("Temperature: \t" + String(temperature_));
  Serial.println("Conductivity: \t" + String(conductivity_));

}





