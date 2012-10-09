/*                          _                                                      _      
                           | |                                                    | |     
  ___ _ __ ___   ___  _ __ | |__   __ _ ___  ___       _ __   __ _ _ __   ___   __| | ___ 
 / _ \ '_ ` _ \ / _ \| '_ \| '_ \ / _` / __|/ _ \     | '_ \ / _` | '_ \ / _ \ / _` |/ _ \
|  __/ | | | | | (_) | | | | |_) | (_| \__ \  __/  _  | | | | (_| | | | | (_) | (_| |  __/
 \___|_| |_| |_|\___/|_| |_|_.__/ \__,_|___/\___| (_) |_| |_|\__,_|_| |_|\___/ \__,_|\___|
                                                                                          
*/
//--------------------------------------------------------------------------------------
// Relay's data recieved by emontx up to COSM
// Relay's data recieved by emonglcd up to COSM
// Decodes reply from server to set software real time clock
// Relay's time data to emonglcd - and any other listening nodes.
// Looks for 'ok' reply from http request to verify data reached COSM

// emonBase Documentation: http://openenergymonitor.org/emon/emonbase

// Authors: Trystan Lea and Glyn Hudson
// Modified for COSM by Roger James
// Part of the: openenergymonitor.org project
// Licenced under GNU GPL V3
//http://openenergymonitor.org/emon/license

// EtherCard Library by Jean-Claude Wippler and Andrew Lindsay
// JeeLib Library by Jean-Claude Wippler
//--------------------------------------------------------------------------------------

//#define DEBUG     //comment out to disable serial printing to increase long term stability 
#define UNO       //anti crash wachdog reset only works with Uno (optiboot) bootloader, comment out the line if using delianuova

#include <JeeLib.h>	     //https://github.com/jcw/jeelib
#include <avr/wdt.h>

#define MYNODE 25            
#define freq RF12_433MHZ     // frequency
#define group 210            // network group 

//---------------------------------------------------
// Data structures for transfering data between units
//---------------------------------------------------

typedef struct { double power1, power2, power3, voltage; } PayloadTX;
PayloadTX emontx;

typedef struct { double importexport, mainsvoltage; } PayloadImport;
PayloadImport emonimport;

//typedef struct { double hour, mins;} PayloadBase;
//PayloadBase emonbase;
typedef struct { double generating, voltage2, ct2;} PayloadGen;
PayloadGen emongen;

//---------------------------------------------------

//---------------------------------------------------------------------
// The PacketBuffer class is used to generate the json string that is send via ethernet - JeeLabs
//---------------------------------------------------------------------
class PacketBuffer : public Print {
public:
    PacketBuffer () : fill (0) {}
    const char* buffer() { return buf; }
    byte length() { return fill; }
    void reset()
    { 
      memset(buf,NULL,sizeof(buf));
      fill = 0; 
    }
    virtual size_t write (uint8_t ch)
        { if (fill < sizeof buf) buf[fill++] = ch; }
    byte fill;
    char buf[150];
    private:
};
PacketBuffer str;

//--------------------------------------------------------------------------
// Ethernet
//--------------------------------------------------------------------------
#include <EtherCard.h>		//https://github.com/jcw/ethercard 

// ethernet interface mac address, must be unique on the LAN
static byte mymac[] = { 0x42,0x31,0x42,0x21,0x30,0x11 };

//IP address of remote sever, only needed when posting to a server that has not got a dns domain name (staticIP e.g local server) 
byte Ethernet::buffer[700];
static uint32_t timer;

//Domain name of remote webserver - leave blank if posting to IP address 
char website[] PROGMEM = "api.cosm.com";

const int redLED = 6;                     // NanodeRF RED indicator LED
//const int redLED = 17;  		  // Open Kontrol Gateway LED indicator

const int greenLED = 5;                   // NanodeRF GREEN indicator LED

int ethernet_error = 0;                   // Etherent (controller/DHCP) error flag
int rf_error = 0;                         // RF error flag - high when no data received 
int ethernet_requests = 0;                // count ethernet requests without reply                 

int dhcp_status = 0;
int dns_status = 0;
int emonglcd_rx = 0;                      // Used to indicate that emonglcd data is available
int data_ready=0;                         // Used to signal that emontx data is ready to be sent
unsigned long last_rf;                    // Used to check for regular emontx data - otherwise error

char line_buf[50];                        // Used to store line of http reply header
int posts1,posts2,posts3,posts4,posts5 = 0;                            //AG counter of posts
double impexp=0;
//**********************************************************************************************************************
// SETUP
//**********************************************************************************************************************
void setup () {
  
  //Nanode RF LED indictor  setup - green flashing means good - red on for a long time means bad! 
  //High means off since NanodeRF tri-state buffer inverts signal 
  pinMode(redLED, OUTPUT); digitalWrite(redLED,LOW);            
  pinMode(greenLED, OUTPUT); digitalWrite(greenLED,LOW);       
  delay(100); digitalWrite(redLED,HIGH);                          // turn off redLED
  
  Serial.begin(9600);
//  Serial.println("\n[webClient]");
posts1 = 0;
posts2 = 0;
posts3 = 0;
posts4 = 0;
posts5 = 0;
  //if (ether.begin(sizeof Ethernet::buffer, mymac, 10) == 0) {	//for use with Open Kontrol Gateway 
 // if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) {	//for use with NanodeRF
  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) {
    Serial.println( "Failed to access Ethernet controller");
    ethernet_error = 1;  
  }

  dhcp_status = 0;
  dns_status = 0;
  ethernet_requests = 0;
  ethernet_error=0;
  rf_error=0;
 
//For use with the modified JeeLib library to enable setting RFM12B SPI CS pin in the sketch. Download from: https://github.com/openenergymonitor/jeelib 
//rf12_set_cs(9);  //Open Kontrol Gateway	
//rf12_set_cs(10); //emonTx, emonGLCD, NanodeRF, JeeNode

  rf12_initialize(MYNODE, freq,group);
  last_rf = millis()-40000;                                       // setting lastRF back 40s is useful as it forces the ethernet code to run straight away
   
  digitalWrite(greenLED,HIGH);                                    // Green LED off - indicate that setup has finished 
 
  #ifdef UNO
  wdt_enable(WDTO_8S); 
  #endif;
}

//**********************************************************************************************************************
// LOOP
//**********************************************************************************************************************
void loop () {
  
  #ifdef UNO
  wdt_reset();
  #endif

  dhcp_dns();   // handle dhcp and dns setup - see dhcp_dns tab
  
  // Display error states on status LED
  if (ethernet_error==1 || rf_error==1 || ethernet_requests > 0) digitalWrite(redLED,LOW);
    else digitalWrite(redLED,HIGH);

  //-----------------------------------------------------------------------------------------------------------------
  // 1) On RF recieve
  //-----------------------------------------------------------------------------------------------------------------
  if (rf12_recvDone()){      
      if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0)
      {
        int node_id = (rf12_hdr & 0x1F);
        emontx = *(PayloadTX*) rf12_data;
        emongen = *(PayloadGen*) rf12_data;
        emonimport = *(PayloadImport*) rf12_data;                              // get emontx data
        if (emontx.power1 < 13) emontx.power1=0;
        if (emontx.power2 < 13) emontx.power2=0;
        if (emontx.power3 < 13) emontx.power3=0;
        if (node_id == 1)                                               // EMONTX
        {
         // emontx = *(PayloadTX*) rf12_data;                              // get emontx data
//          Serial.println();                                              // print emontx data to serial
//          Serial.println("emonTx data rx1");
          last_rf = millis();                                            // reset lastRF timer
          
          delay(50);                                                     // make sure serial printing finished
                               
          // CSV data feed_id,value\r\n one feed per line
          
          str.reset();                                                   // Reset csv string      
          str.println("rf_fail1,0");                                      // RF recieved so no failure
          str.print("power1,");          str.println(emontx.power1);     // Add power reading
          str.print("power2,");          str.println(emontx.power2);     // Add power reading
          str.print("power3,");          str.println(emontx.power3);     // Add power reading   
          str.print("battery_voltage1,"); str.println(emontx.voltage/1000.00);    // Add emontx battery voltage reading
          str.print("posts1,"); str.println(posts1);
          data_ready = 1;                                                // data is ready
          rf_error = 0;
          posts1 ++;
        }
    if (node_id == 2)                                               // EMONTX
        {
       //   emontx = *(PayloadTX*) rf12_data;                              // get emontx data
//          Serial.println();                                              // print emontx data to serial
//          Serial.println("emonTx data rx2");
          last_rf = millis();                                            // reset lastRF timer
          
          delay(50);                                                     // make sure serial printing finished
          
          str.reset();                                                   // Reset csv string      
          str.println("rf_fail2,0");                                      // RF recieved so no failure
          str.print("power4,");          str.println(emontx.power1);     // Add power reading
          str.print("power5,");          str.println(emontx.power2);     // Add power reading
          str.print("power6,");          str.println(emontx.power3);     // Add power reading   
          str.print("battery_voltage2,"); str.println(emontx.voltage/1000.00);    // Add emontx battery voltage reading
          str.print("posts2,"); str.println(posts2);
          data_ready = 1;                                                // data is ready
          rf_error = 0;
          posts2 ++;
        }
if (node_id == 3)                                               // EMONTX
        {
        //  emontx = *(PayloadTX*) rf12_data;                              // get emontx data
//          Serial.println();                                              // print emontx data to serial
//          Serial.println("emonTx data rx3");
          last_rf = millis();                                            // reset lastRF timer
          
          delay(50);                                                     // make sure serial printing finished
      
          str.reset();                                                   // Reset csv string      
          str.println("rf_fail3,0");                                      // RF recieved so no failure
          str.print("power7,");          str.println(emontx.power1);     // Add power reading
          str.print("power8,");          str.println(emontx.power2);     // Add power reading
          str.print("power9,");          str.println(emontx.power3);     // Add power reading   
          str.print("battery_voltage3,"); str.println(emontx.voltage/1000.00);    // Add emontx battery voltage reading
          str.print("posts3,"); str.println(posts3);
          data_ready = 1;                                                // data is ready
          rf_error = 0;
          posts3 ++;
        }
if (node_id == 10)                                               // EMON Main Import/Export Meter
        {
 //         emonimport = *(PayloadImport*) rf12_data;                              // get emontx data
//          Serial.println();                                              // print emontx data to serial//
//          Serial.println("emonTx data rx10");
          last_rf = millis();                                            // reset lastRF timer
          
          delay(50);                                                     // make sure serial printing finished
                               
         impexp=emonimport.importexport;
          str.reset();                                                   // Reset csv string      
          str.println("rf_fail10,0");                                      // RF recieved so no failure
          str.print("ImportExport,");     str.println(emonimport.importexport);     // Add power reading
          str.print("Voltage,");          str.println(emonimport.mainsvoltage);     // Add power reading
          str.print("posts4,"); str.println(posts4);
          data_ready = 1;                                                // data is ready
          rf_error = 0;
          posts4 ++;
        }

if (node_id == 12)                                               // EMONTX PV (with second clamp)
        {
 //         emongen = *(PayloadGen*) rf12_data;                              // get emontx data
   //       Serial.println();                                              // print emontx data to serial
   //       Serial.println("emonTx data rx12");
          last_rf = millis();                                            // reset lastRF timer
          
          delay(50);                                                     // make sure serial printing finished
         
          str.reset();                                                   // Reset csv string      
          str.println("rf_fail12,0");                                      // RF recieved so no failure
          str.print("Generating,");       str.println(emongen.generating);     // Add power reading
          str.print("Voltage2,");         str.println(emongen.voltage2/100.00);     // Add power reading
          str.print("CT2,");              str.println(emongen.ct2);     // Add power reading
          str.print("Consuming,");              str.println(abs(emongen.generating)+impexp);     // Add power reading
          str.print("posts5,"); str.println(posts5);
          data_ready = 1;                                                // data is ready
          rf_error = 0;
          posts5 ++;
        }
         
      }
    }

  //-----------------------------------------------------------------------------------------------------------------
  // 2) If no data is recieved from rf12 module the server is updated every 30s with RFfail = 1 indicator for debugging
  //-----------------------------------------------------------------------------------------------------------------
  if ((millis()-last_rf)>30000)
  {
    last_rf = millis();                                                 // reset lastRF timer
    str.reset();                                                        // reset csv string
    str.println("rf_fail,1");                                           // No RF received in 30 seconds so send failure 
    data_ready = 1;                                                     // Ok, data is ready
    rf_error=1;
  }


  //-----------------------------------------------------------------------------------------------------------------
  // 3) Send data via ethernet
  //-----------------------------------------------------------------------------------------------------------------
  ether.packetLoop(ether.packetReceive());
  
  if (data_ready) {
    
         
    Serial.println("CSV data"); Serial.println(str.buf); // print to serial csv string

    // Example of posting to COSM
    ethernet_requests ++;
    ether.httpPost(PSTR("/v2/feeds/74545.csv?_method=put"), website, PSTR("X-ApiKey:KEYHERE"), str.buf, my_callback);
    data_ready =0;
      }
  
//  if (ethernet_requests > 10) delay(10000); // Reset the nanode if more than 10 request attempts have been tried without a reply

}
//**********************************************************************************************************************

//-----------------------------------------------------------------------------------
// Ethernet callback
// recieve reply and decode
//-----------------------------------------------------------------------------------
static void my_callback (byte status, word off, word len) {

//  Serial.println("Server Reply");
  get_header_line(1, off);
//  Serial.println(line_buf);
//  Serial.println(strlen(line_buf));
  
  if (0 == strncmp(line_buf, "HTTP/1.1 200 OK", 15)) {
    
    get_header_line(2,off);      // Get the date and time from the header
    Serial.print("ok recv from server | ");    // Print out the date and time
    Serial.println(line_buf);    // Print out the date and time
    
    // Decode date time string to get integers for hour, min, sec, day
    // We just search for the characters and hope they are in the right place
 //   char val[1];
 //   val[0] = line_buf[23]; val[1] = line_buf[24];
 //   int hour = atoi(val);
 //   val[0] = line_buf[26]; val[1] = line_buf[27];
 //   int minute = atoi(val);
 //   val[0] = line_buf[29]; val[1] = line_buf[30];
 //   int second = atoi(val);
 //   val[0] = line_buf[11]; val[1] = line_buf[12];
 //   int day = atoi(val);
    
    // Don't send all zeros, happens when server failes to returns reponce to avoide GLCD getting mistakenly set to midnight
 //   if (hour>0 || minute>0 || second>0) 
 //   {  
 //     delay(100);
    
 //     char data[] = {'t',hour,minute,second};
 //     int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
 //     rf12_sendStart(0, data, sizeof data);
 //     rf12_sendWait(0);
    
 //     Serial.println("time sent to emonGLCD");
 //   }
    
    ethernet_requests = 0; ethernet_error = 0;
  }
}
