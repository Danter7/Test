/*
 * This program demonstrates .....
 * 
*/

/*
 * Arduino IDE tools settings:
 * Board            = ESP32 Dev Module
 * Upload Speed     = 921600
 * CPU Frequency    = 240MHz (WiFi/BT)
 * Flash Frequency  = 80MHz
 * Flash Mode       = QIO
 * Flash Size       = 4MB (32Mb)
 * Partition Scheme = Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)
 * Core Debug Level = None
 * PSRAM            = Disabled
*/

//#############################################################################
//------------------------------Include Jordan's Custom Libraries--------------
    #include "CustomArduinoFunctions.h"
    #include "MotorDrive.h"
    #include "CustomDataTypes.h"
    //#include "SD_Card.h"
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//#############################################################################
//------------------------------------------NTP Libraries----------------------
    #include <NTPClient.h>
    //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//#############################################################################
//------------------------------------------Wifi Libraries---------------------
    #include <WiFi.h>         //-- Wifi library for ESP32
    #include <WiFiMulti.h>    //-- Wifi library for connecting to the strongest wifi network from multiple options
    #include <WiFiUdp.h>
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//#############################################################################
//------------------------------------------WebServer Libraries----------------
    #include <AsyncTCP.h>
    #include <ESPAsyncWebServer.h>
    #include "FS.h"
    #include "SD.h"
    #include "SPI.h"
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//#############################################################################
//------------------------------------------LCD Libraries----------------------
    #include <Wire.h>
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>
    
    #include "CustomLcdIcons.h"
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//#############################################################################
//-------------------------------Background timer library----------------------
    #include "elapsedMillis.h"                      //-- [top]-[Sketch]-[include library]-[library manager]-[search]-["elapsedMillis"]-[install "elapsedMillis" by Paul Stoffregen]
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//#############################################################################
//------------------------------------Gyroscope Libraries----------------------
    #include "I2Cdev.h"
    //#include "MPU6050_6Axis_MotionApps_V6_12.h"  //changed to this version 10/04/19
    #include "MPU6050_6Axis_MotionApps20.h"
    #include "elapsedMillis.h"                      //-- [top]-[Sketch]-[include library]-[library manager]-[search]-["elapsedMillis"]-[install "elapsedMillis" by Paul Stoffregen]
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//#############################################################################
//-------------------------------Background timer library----------------------
    #include <SoftwareSerial.h>                     //-- A built in arduino library
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\






//######################################################################################################
//------------------------------------------UNIVERSAL Positioning Variables-----------------------------
    float x_Rotation = 0;
    float y_Rotation = 0;
    float z_Rotation = 0;
    
    float GlobalHeadding = 0;         //-- θ°
    float LocalHeaddingOffset = 0;    //-- θ°
    float LocalHeadding = 0;          //-- θ°

    float Robot_Position_X = 0;
    float Robot_Position_Y = 0;
    float Last_Robot_Position_X = 0;
    float Last_Robot_Position_Y = 0;
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//---------------------------------------WiFi GENERAL Variables-----------------------------------------
    WiFiMulti wifiMulti;
    
    bool WifiConnected = false;
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//------------------------------------WiFi NTP-CLIENT Variables-----------------------------------------
    const long utcOffsetInSeconds = (12 + 0)*60*60; //-- for UTC +12 for New Zealand https://en.wikipedia.org/wiki/List_of_UTC_time_offsets
  //const long utcOffsetInSeconds = (12 + 1)*60*60; //-- for UTC +12 for New Zealand https://en.wikipedia.org/wiki/List_of_UTC_time_offsets
    //-- (12 + 0) or (12 + 1) based on DaylightSavingsTime-DST shift. The dates for DST can be found here: https://www.govt.nz/browse/recreation-and-the-environment/daylight-saving/
    
    const char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    
    // Define NTP Client to get time
    WiFiUDP ntpUDP;
    NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//------------------------------------WiFi SERVER Variables---------------------------------------------
    bool ServerOn = false;

    bool RecievingPath = false;
    int NewPathWaypoint_Count = 0;
    VectorWaypoint Path1_ArrayOfVectorWaypoints[5];
    bool RecievingNewPathViaUpload;
    int NewPathViaUpload_Length;
    int NewPathViaUpload_WaypointCounter;

      
    
    // Set web server port number to 80
    //WiFiServer server(80);
    // Create AsyncWebServer object on port 80
    AsyncWebServer server(80);
    
    // Variable to store the HTTP request
    String header;
    
    // Current time
    unsigned long currentTime = millis();
    // Previous time
    unsigned long previousTime = 0; 
    // Define timeout time in milliseconds (example: 2000ms = 2s)
    const long timeoutTime = 2000;

    String TempWiFiCommand_1;
    String TempWiFiCommand_2;
    String TempWiFiCommand_3;
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//------------------------------------------LCD Variables-----------------------------------------------
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); //-- Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//---------------------------------------Gyroscope Variables--------------------------------------------
    // class default I2C address is 0x68
    // specific I2C addresses may be passed as a parameter here
    // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
    // AD0 high = 0x69
    MPU6050 mpu;
    //MPU6050 mpu(0x69); // <-- use for AD0 high
    
    
    
    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    
    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    
    //extra stuff
    int IMU_CHECK_INTERVAL_MSEC = 100;
    elapsedMillis sinceLastIMUCheck;
    float global_yawval = 0.0; //contains most recent yaw value from IMU
    int global_fifo_count = 0; //made global so can monitor from outside GetIMUHeadingDeg() fcn
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//##################################################
//-- Battery variables -----------------------------
    int BatteryLevel_Count = 1;
    
    int WifiSymbol_State = 0;
    
    int  BatteryLevel_Quarters = 0;
    int  BatteryLevel_Percentage = 0;
    bool batteryLow = false;
    bool charging = true;
    bool displayBattery = true;

    
    bool WifiIcon_Blinking = true;
    bool WifiIcon_Blink = true;
    
    int BatteryBlink_TIMER_INTERVAL = 600;      //-- elapsed timer interval
    elapsedMillis BatteryBlink_ELAPSED_TIMER;   //-- elapsed timer

    int WifiBlink_TIMER_INTERVAL = 1000;        //-- elapsed timer interval
    elapsedMillis WifiBlink_ELAPSED_TIMER;      //-- elapsed timer
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//##################################################
//-- SD-card variables -----------------------------
    String Map_TextFile_AsString;  //-- 900 can fit a 30x30 square grid.
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//##################################################
//-- SoftwareSerial variables ----------------------
    SoftwareSerial SoftSerial(27, 14); // RX, TX
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\















//######################################################################################################
//----------------------------------------------Setup WIFI GENERAL--------------------------------------
    void Setup_WiFi()
    {
      // Connect to Wi-Fi network with SSID and password
      WiFi.mode(WIFI_STA);

      //--              ssid               password
      wifiMulti.addAP("JPOYNTER",         "test123a");
      wifiMulti.addAP("Js IPhone",        "test123a");
      wifiMulti.addAP("vodafoneBC6D22",   "PoynterWIFI@167");

      // WiFi.scanNetworks will return the number of networks found
      int n = WiFi.scanNetworks();
      Serial.println("scan done");
      if (n == 0)
      {
        Serial.println("no networks found");
      }
      else
      {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i) {
          // Print SSID and RSSI for each network found
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print(WiFi.SSID(i));
          Serial.print(" (");
          Serial.print(WiFi.RSSI(i));
          Serial.print(")");
          Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
          delay(10);
        }
      }
      
      // Connect to Wi-Fi using wifiMulti (connects to the SSID with strongest connection)
      Serial.println("Connecting to WIFI");
      if(wifiMulti.run() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
      }
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//----------------------------------------------Setup Web Server---------------------------------------
    bool initSDCard(){
      if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return(0);
      }
      uint8_t cardType = SD.cardType();
    
      if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return(0);
      }

      Serial.println("SD Card Setup");
      Serial.print("SD Card Type: ");
      if(cardType == CARD_MMC){
        Serial.println("MMC");
        return(1);
      } else if(cardType == CARD_SD){
        Serial.println("SDSC");
        return(1);
      } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
        return(1);
      } else {
        Serial.println("UNKNOWN");
        return(1);
      }
      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.printf("SD Card Size: %lluMB\n", cardSize);
      Serial.println("");
    }
    
    String processor(const String& var)
    {
      Serial.println(var);
    
      // YOU MUST WRITE PLACEHOLDER VARIABLES INTO YOUR HTML FILE USING A TILDA AT EACH END.
      // FOR EXAMPLE:   ~PLACEHOLDER~
      // FURTHER MORE:  <p>Humidity: ~PLACEHOLDER_HUMIDITY~ %</p>
     
      if(var == "PLACEHOLDER"){
        return("Hello from the SD card.");
      }
      else if(var == "PLACEHOLDER_TEMPERATURE"){
        return String(random(10, 20));
      }
      else if(var == "PLACEHOLDER_HUMIDITY"){
        return String(random(0, 50));
      }
      else if(var == "PLACEHOLDER_BatteryPercentage"){
        return String(5);
      }
      else if(var == "PLACEHOLDER_CompassHeadding"){
        return String(53);
      }
      else if(var == "PLACEHOLDER_RobotPosition_X"){
        return String(10);
      }
      else if(var == "PLACEHOLDER_RobotPosition_Y"){
        return String(-5);
      }
      else if(var == "PLACEHOLDER_Map"){
        if(Map_TextFile_AsString != "")
          return Map_TextFile_AsString;
        else
          return("No map read from the SD card.");
      }

      else{                         //-- if we do not know what to do with the placeholder
        return("~" + var + "~");    //-- then simply return it. THIS STOPS IT FROM CRASHING
      }
      //return String();            //-- replaced by the else statement just above
    }
    
    void Setup_WiFi_Server()
    {
      Serial.println("Web Server Setup");


      //Serial.println("// YOU MUST WRITE PLACEHOLDER VARIABLES INTO YOUR HTML FILE USING A TILDA AT EACH END.");
      //Serial.println("// FOR EXAMPLE:   ~PLACEHOLDER~");
      //Serial.println("// FURTHER MORE:  <p>Humidity: ~PLACEHOLDER_HUMIDITY~ %</p>");
      
      //################################################################Load page without placeholder replacements
      /*server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SD, "/index.html", "text/html");
      });*/
    
      //################################################################Load page WITH placeholder replacements
      server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){                      //for ["/"] request -> same as ["/index.html"]
        request->send(SD, "/index.html", "text/html", false, processor);
      });
      server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){            //for ["/index.html"] request
        request->send(SD, "/index.html", "text/html", false, processor);
      });
      server.on("/UploadPath.html", HTTP_GET, [](AsyncWebServerRequest *request){       //for ["/UploadPath.html"] request
        request->send(SD, "/UploadPath.html", "text/html", false, processor);
      });
      server.on("/PathRunning_1.html", HTTP_GET, [](AsyncWebServerRequest *request){    //for ["/PathRunning_1.html"] request
        request->send(SD, "/PathRunning_1.html", "text/html", false, processor);
      });
      server.on("/About.html", HTTP_GET, [](AsyncWebServerRequest *request){            //for ["/About.html"] request
        request->send(SD, "/About.html", "text/html", false, processor);
      });
      server.on("/Map.html", HTTP_GET, [](AsyncWebServerRequest *request){              //for ["/Map.html"] request
        request->send(SD, "/Map.html", "text/html", false, processor);
      });
      server.on("/Original.html", HTTP_GET, [](AsyncWebServerRequest *request){         //for ["/Original.html"] request
        request->send(SD, "/Original.html", "text/html", false, processor);
      });

      server.on("/RunNavigatePath/Path1", HTTP_GET, [](AsyncWebServerRequest *request){         //for ["/Original.html"] request
        Serial.println("Hello!");
        NavigatePath_FromSDCard();
        request->send(SD, "/index.html", "text/html", false, processor);
      });
    
      server.serveStatic("/", SD, "/");
    
      server.begin();

      Serial.println("Done");
      Serial.println("");
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//----------------------------------------------Setup LCD-----------------------------------------------
    void Setup_LCD()
    {
      // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
      if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))    //-- Address 0x3C for 128x32 -- Address 0x3D for 128x64
      {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
      }
    
      // Clear the buffer.
      display.clearDisplay();
      display.display();
    
    
      // text display tests
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
    
      display.clearDisplay();
      LCD_PrintLn(0, 0, "Time: ##:##:##");  //-- add some functions to the library that let you get the 12hr am/pm time or the 24hr time? I added getDate_J, it was a copy-paste of code that was already there, it will be super easy    
      //LCD_PrintLn(0, 3, "Day");
      LCD_PrintLn(0, 1, "YYYY/MM/DD");       //-- add some functions to the library that let you get the year, month, and day? I added getDate_J, it was a copy-paste of code that was already there, it will be super easy    
    
      LCD_PrintLn(0, 3, "ypr -#.## -#.## -#.##");  //-- print the [yaw-pitch-roll] measured from the gyroscope
      
      UpdateAndDraw_OBar();

      display.dim(true);  //-- dim the display
      
      display.display();
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//-------------------------------------------Setup Gyroscope--------------------------------------------
    void Setup_Gyroscope()
    {
      Serial.println("Gyroscope Setup");
      
      // I2C bus init done in SBWIRE.h
    
      // initialize device
      /*
      Serial.println(F("Initializing MPU6050..."));
      */
      mpu.initialize();
    
      // verify connection
      /*
      Serial.println(F("Testing device connections..."));
      Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      */
    
      // load and configure the DMP
      /*
      Serial.println(F("Initializing DMP..."));
      */
      devStatus = mpu.dmpInitialize();
    
      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
      // make sure it worked (returns 0 if so)
      if (devStatus == 0)
      {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        /*
        mpu.PrintActiveOffsets();
        */
    
        // turn on the DMP, now that it's ready
        /*
        Serial.println(F("Enabling DMP..."));
        */
        mpu.setDMPEnabled(true);
    
    
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        /*
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        */
        dmpReady = true;
    
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
      }
      else
      {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        /*
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        */
      }
    
      sinceLastIMUCheck = 0; //this manages 'other program stuff' cycle
    
      //print out column headers
      /*
      Serial.println("MSec\tYawDeg");
      */

      Serial.println("");
      Serial.println("Done");
      Serial.println("");
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//----------------------------------------------Setup IR Distance Sensor--------------------------------
    void Setup_IR_DistanceSensor()
    {
      //pinMode(IR_DistanceSensorPin, OUTPUT);

      pinMode(          IR_DistanceSensorPin,  INPUT_PULLUP);
      //attachInterrupt(  IR_DistanceSensorPin,  IR_LedgeDetector_ISR, RISING);   //-- The IR sensor goes to 0 when a ledge is detected. So attach an interrupt for a volatage fall.
      //- set to attach and detach only when the motors are moving.
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//----------------------------------------------Setup NPT Client----------------------------------------
    void Setup_NPT_Client()
    {
      timeClient.begin();         //-- begin the internal RTC-RealTimeClock in the NodeMCU
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//----------------------------------------------Read SD Card Map----------------------------------------
    int TxtFileLineCount(String PassedString){
    String TempString = PassedString;
    int NewPos = 0;
    bool Done = false;
    int Count = 0;
  
    while(Done != true)
    {
      NewPos = TempString.indexOf('\n') + 1;
      if(NewPos != TempString.length())
      {        
        String Temp = TempString.substring(NewPos, TempString.length());
        TempString = "";
        TempString = Temp;
        Count++;
      }
      else
      {
        Count++;
        Done = true;
      }
    }
  
    return(Count);
  }
    
    
    
    
    String Path1_TextFile_AsString = "";
    
    String Jordan_ReadTextFileToString(fs::FS &fs, const char * path){
        Serial.printf("Reading file: %s\n", path);
    
        //--------------------------------------------------------------------------------------- Open the file
        File file = fs.open(path);
        if(!file){
            Serial.println("Failed to open file for reading");
            return("");
        }
        //---------------------------------------------------------------------------------------
    
        //--------------------------------------------------------------------------------------- Read the text file to a string
        String TempString;
        while(file.available()){
            char TempChar = file.read();
            TempString += TempChar;
        }
        file.close();
        TempString += '\n'; //-- add a new line character at the very end of the string
        //---------------------------------------------------------------------------------------
    
    
    
        return(TempString);
    
        
    
    
        /*
        List<int> IntList;
        int Num = 7;
        IntList.add(Num);
        int *firstElementPtr = IntList.getPointer(0); // Here, have to be the '*' to get the int Value, because otherwise a pointer (memory address) will be returned.
        Serial.println(*firstElementPtr);
        free(firstElementPtr); // free the pointer because it is an immutable list
    
        List<String> StrList;
        String Str = "Word";
        StrList.add(Str);
        String *StrFirstElementPtr = StrList.getPointer(0); // Here, have to be the '*' to get the int Value, because otherwise a pointer (memory address) will be returned.
        Serial.println(*StrFirstElementPtr);
        free(StrFirstElementPtr); // free the pointer because it is an immutable list
        */
    }
    
    
    
    
    int NumberOfDescriptionLines = 3;

    void Read_SD_Card_Map_STEP1()
    {
      int Offset_X        = 10;
      int Offset_Y        = 120;
      int Size_X_InBlocks = 13;
      int Size_Y_InBlocks = 13;
    
      int BorderThickness = 3;
      int PixelSize       = 5;
      
      //-- Draw the map boarder
      //tft.drawRect(Offset_X-1,  Offset_Y-1, ((Size_X_InBlocks)*PixelSize)+2,  ((Size_Y_InBlocks)*PixelSize)+2,  TFT_BLUE);  //-- can use HEX 0x00A0FF
      //tft.drawRect(Offset_X-2,  Offset_Y-2, ((Size_X_InBlocks)*PixelSize)+4,  ((Size_Y_InBlocks)*PixelSize)+4,  TFT_BLUE);
      //tft.drawRect(Offset_X-3,  Offset_Y-3, ((Size_X_InBlocks)*PixelSize)+6,  ((Size_Y_InBlocks)*PixelSize)+6,  TFT_BLUE);

      //-- Write the headding "Map"
      //tft.setCursor(Offset_X, Offset_Y-20);
      //tft.setTextColor(TFT_BLUE);  //tft.setTextSize(2);
      //tft.println("Map");
      //-- Write the map name
      //tft.setCursor((Offset_X+(Size_X_InBlocks)*PixelSize)+5, Offset_Y);
      //tft.setTextColor(TFT_BLUE);  //tft.setTextSize(1);
      //tft.println("Length: 00:00:00");
      //-- 
      //tft.setCursor((Offset_X+(Size_X_InBlocks)*PixelSize)+5, Offset_Y+8);
      //tft.setTextColor(TFT_BLUE);  //tft.setTextSize(1);
      //tft.println("ETA: 00:00:00");


      if(Map_TextFile_AsString != "")                                                     //-- if the Map file was sucessfully opened and read, the string should not be empty. So proceed:
      {

        //--draw the robot's square----------------------------------------------------------------
        //--draw it in the centre of the screen.
        int RobotCoOrdinates_X = 0;
        int RobotCoOrdinates_Y = 0;
        
        if(IsNumerEven(Offset_X) == true)
          RobotCoOrdinates_X = Offset_X+((Size_X_InBlocks/2)*PixelSize);
        else
          RobotCoOrdinates_X = Offset_X+(((Size_X_InBlocks-1)/2)*PixelSize);

        if(IsNumerEven(Offset_Y) == true)
          RobotCoOrdinates_Y = Offset_Y+((Size_Y_InBlocks/2)*PixelSize);
        else
          RobotCoOrdinates_Y = Offset_Y+(((Size_Y_InBlocks-1)/2)*PixelSize);

        //tft.fillRect(RobotCoOrdinates_X, RobotCoOrdinates_Y, PixelSize, PixelSize, TFT_GREEN);
        //-----------------------------------------------------------------------------------------
      }
      else
      {
        Serial.println("Map display canceled because the .txt file read failed");
        Serial.println("");

        //-- update the LCD to show that there is no map
        //tft.setCursor(Offset_X+BorderThickness+2, Offset_Y+BorderThickness+2);
        //tft.setTextColor(TFT_BLUE);  //tft.setTextSize(2);
        //tft.println("None");
      }
    }

    void Read_SD_Card_Map()
    {
      String Temp_MapTextFileString = Map_TextFile_AsString;

      int Offset_X        = 10;
      int Offset_Y        = 120;
      int Size_X_InBlocks = 13;
      int Size_Y_InBlocks = 13;
    
      int BorderThickness = 3;
      int PixelSize       = 5;

      Read_SD_Card_Map_STEP1();
      
      
      if(Map_TextFile_AsString != "")                                                     //-- if the Map file was sucessfully opened and read, the string should not be empty. So proceed:
      {
        Serial.println(Temp_MapTextFileString);
        Serial.println("Character count: " + String(Temp_MapTextFileString.length()));
        int NumberOfLines = TxtFileLineCount(Temp_MapTextFileString);
        Serial.println("Line count: " + String(NumberOfLines));
  
        
        String ArrayOfLines[NumberOfLines /*- NumberOfDescriptionLines*/];                   //-- set the array size to be the number of lines
        
        bool Done = false;
        int Count = 0;
        while(Done != true)
        {
          int NewPos = 0;
          NewPos = Temp_MapTextFileString.indexOf('\n');
          //Serial.println("Index: " + String(NewPos));
          
          String Line = Temp_MapTextFileString.substring(0, NewPos);
          //Serial.println("Line: " + Line);
          //if(count >= NumberOfDescriptionLines)                                                                      //-- if we are past the first three description lines, then:
          ArrayOfLines[Count] = Line;                                                       //-- add the newly detected line to the array
          
          String Temp = Temp_MapTextFileString.substring(NewPos+1, Temp_MapTextFileString.length());  //-- remove the newly detected line from the string
          //Serial.println("Remaining: \n" + Temp);
          Temp_MapTextFileString = "";
          Temp_MapTextFileString = Temp;
          Count++;
      
          if(NewPos == Temp_MapTextFileString.length())
          {
            String Line = Temp_MapTextFileString.substring(0, NewPos-1);
            ArrayOfLines[Count] = Line;                                         //-- add the newly detected line to the array
            
            Done = true;
          }
        }

        Serial.println("Line count2: " + String(ArrayLength(ArrayOfLines)));

        Map_TextFile_AsString = "";
        for(int i = 3; i < ArrayLength(ArrayOfLines); i++)
        {
          if(i < (ArrayLength(ArrayOfLines)-1))
            Map_TextFile_AsString += (ArrayOfLines[i].substring(0, ArrayOfLines[i].length()-1) + '\n');    //-- remove the new-line craracter from the end of this line
          else
            Map_TextFile_AsString += (ArrayOfLines[i].substring(0, ArrayOfLines[i].length()) + '\n');      //-- dont remove the last character, because there is no new-line character
          
          /*if(i < (ArrayLength(ArrayOfLines)-1))
            Map_TextFile_AsString += (ArrayOfLines[i].substring(0, ArrayOfLines[i].length()-1)   + " |" + '\n');    //-- remove the new-line craracter from the end of this line
          else
            Map_TextFile_AsString += (ArrayOfLines[i].substring(0, ArrayOfLines[i].length())   + " |" + '\n');      //-- dont remove the last character, because there is no new-line character*/
        }
  



        
        
        Serial.println("Under Here:");
        for(int i = 3; i < NumberOfLines; i++)              //-- start from after the legend and description starting at line 3
        {
          Serial.println(ArrayOfLines[i]);
    
          for(int x = 0; x < ArrayOfLines[i].length(); x++)
          {
            if(ArrayOfLines[i][x] == 'B'){}
              //tft.fillRect(Offset_X+(x*PixelSize), Offset_Y+((i-3)*PixelSize), PixelSize, PixelSize, TFT_RED);
            
            else if(ArrayOfLines[i][x] == 'O'){}
              //tft.fillRect(Offset_X+(x*PixelSize), Offset_Y+((i-3)*PixelSize), PixelSize, PixelSize, TFT_YELLOW);
          }
        }
      }
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


int Last_FreeHeap = 0;

void setup()
{
  Serial.begin(115200);       //-- setup serial communication
  Serial.println("");

  SoftSerial.begin(9600);
  SoftSerial.println("analogWrite(3, 0);");
  SoftSerial.println("analogWrite(5, 0);");
  SoftSerial.println("analogWrite(6, 0);");
  SoftSerial.println("analogWrite(9, 0);");


  SoftSerial.println("DistplaySetupSteps" );
  
  
//Serial.println("FreeHeap: " + String(ESP.getFreeHeap()));                                                                   Last_FreeHeap = ESP.getFreeHeap();

  //Setup_SDcard();             //-- [External] A LocalFolder library, Saved next to the .ino
  
  Setup_LCD();                //-- [Internal] setup the LCD
//Serial.println("UsedByMe: " + String(Last_FreeHeap - ESP.getFreeHeap()) + "    " + "FreeHeap: " + ESP.getFreeHeap());
//int LCD_128x64_Setup_Mem = (Last_FreeHeap - ESP.getFreeHeap());                                                             Last_FreeHeap = ESP.getFreeHeap();
  UpdateTheLCD();
//Serial.println("UsedByMe: " + String(Last_FreeHeap - ESP.getFreeHeap()) + "    " + "FreeHeap: " + ESP.getFreeHeap());
//int LCD_128x64_Update_Mem = (Last_FreeHeap - ESP.getFreeHeap());                                                            Last_FreeHeap = ESP.getFreeHeap();

  

//SoftSerial.println("LCD_240x280(tft.println(Hello World!));");




  Setup_WiFi();               //-- [Internal] setup NodeMCU WiFi


//tft.setTextColor(TFT_GREEN);  tft.setTextSize(1);  tft.print("SD Card      ");
  if(initSDCard()){
    //Serial.println("UsedByMe: " + String(Last_FreeHeap - ESP.getFreeHeap()) + "    " + "FreeHeap: " + ESP.getFreeHeap());
  }
  else{
    //tft.setTextColor(TFT_RED);  tft.setTextSize(1);  tft.println("(Fail)  Mem:" /*+ String(Last_FreeHeap - ESP.getFreeHeap())*/);   //Last_FreeHeap = ESP.getFreeHeap();
  }

  Setup_WiFi_Server();        //-- [Internal] setup WIFI server

  Setup_Gyroscope();          //-- [Internal] The code does not work with the gyroscope setup.

  Setup_IR_DistanceSensor();  //-- [Internal]

  Setup_MotorDrive();         //-- [External] A LocalFolder library, Saved next to the .ino

  Setup_NPT_Client();         //-- [Internal]


  Map_TextFile_AsString = Jordan_ReadTextFileToString(SD, "/Navigation/Map.txt");
//Serial.println("UsedByMe: " + String(Last_FreeHeap - ESP.getFreeHeap()) + "    " + "FreeHeap: " + ESP.getFreeHeap());       Last_FreeHeap = ESP.getFreeHeap();
  Read_SD_Card_Map();
  Path1_TextFile_AsString = Jordan_ReadTextFileToString(SD, "/Navigation/Path_1.txt");
  Read_PathFromSDCard();




  SoftSerial.println("IpAddress(" + WiFi.localIP().toString() + ");" );


  //NavigatePath_FromSDCard();
}











const uint32_t connectTimeoutMs = 10000;  //-- WiFi connect timeout per AP. Increase when connecting takes longer.

String OldMemText = "";

void loop()
{
  CheckWifiConnection();


  

  if(Serial.available()){
      String Command = Serial.readStringUntil('\n');
      CommandOperation(Command);
  }

  
  //Serial.println("FreeHeap: " + String(ESP.getFreeHeap()));
  //tft.setCursor(0, 200);  tft.setTextColor(TFT_BLACK);  tft.setTextSize(2);  tft.println(OldMemText);
  //tft.setCursor(0, 200);  tft.setTextColor(TFT_GREEN);  tft.setTextSize(2);  tft.println("Free Mem:" + String(ESP.getFreeHeap()));
  //OldMemText = ("Free Mem:" + String(ESP.getFreeHeap()));
  //delay(500);
  

                      //-- MicroController CODE version #4 uses a CPU-requested poll driven Gyroscope function
                      //-- this means the CPU isnt constantly interrupted by both the SPI and an inerrupt driven gyroscope
                      //--
  Read_Gyroscope();   //-- read the values from the gyroscope.
                      //-- the gyroscope read function waits for an elapsed 100ms before reading as decleared in "IMU_CHECK_INTERVAL_MSEC"
                      //-- you MUST read the gyroscope outside of the SPI communication function.
                      //-- trying to read from the gyro from inside the SPI communcation function causes a "FIFO overflow!"



  


  UpdateTheLCD();


  if (button1.pressed) {
      Serial.printf("Btn1 pressed %u times\n", button1.numberKeyPresses);
      button1.pressed = false;
  }
  if(MotorRunning == true){
    Serial.print("MotorRunning ");
    Serial.println(DriveMotor_Right_RotaryEncoder.Count);
    DriveMotor_CheckTurnOff();
  }
  if(PathMove_Forward.Running == true){
    Serial.print("PathMove_Forward.Running ");
    Serial.println(DriveMotor_Right_RotaryEncoder.Count);
    PathMove_Forward.CheckEnd();
  }
  
}









//######################################################################################################
//------------------------------------------Loops for Serial Communication------------------------------
    void CommandOperation(String Command)
    {
      if(Command.indexOf("Run Path") >= 0 || Command.indexOf("Run path") >= 0 || Command.indexOf("run path") >= 0) 
      {
        NavigatePath_FromSDCard();
        Serial.println("Ran Path");
      }
      else
      {
          Serial.println("Invalid Command: [" + Command + "]");
      }
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//------------------------------------------Loops for Wifi GENERAL--------------------------------------
    void CheckWifiConnection()
    {
      //#############   DISPLAY WIFI STRENGTH   ##########
      //if the connection to the stongest network is lost, it will connect to the next strongest network on the list
      if (wifiMulti.run(connectTimeoutMs) == WL_CONNECTED) {
        /*Serial.print("WiFi connected: ");
        Serial.print(WiFi.SSID());
        Serial.print(" ");
        Serial.println(WiFi.RSSI());*/

        timeClient.update();              //-- update the internal RTC clock to match the local time
                                          //-- if you call this update when there is no wifi connection then the clock lags.
                                          //-- this causes a 2 or 3 second delay that becomes a compounding issue with time. after 10 minutes it gets really bad.
        WifiConnected = true;
      }
      else
      {
        //Serial.println("WiFi not connected!");
        WifiConnected = false;
      }

      /*if( WiFi.status() == WL_CONNECTED ) //-- if there is a wifi connection
        WifiConnected = true;
      else  
        WifiConnected = false;*/

      
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//-------------------------------------------Loops for LCD----------------------------------------------
    void UpdateTheLCD()
    {
      display.clearDisplay();
      LCD_PrintLn(16, 0, (String)WiFi.RSSI());
      
      //LCD_PrintLn(0, 2, "Time: " + timeClient.getFormattedTime());  //-- add some functions to the library that let you get the 12hr am/pm time or the 24hr time? I added getDate_J, it was a copy-paste of code that was already there, it will be super easy    
      LCD_PrintLn(43, 0, timeClient.getFormattedTime());

      LCD_PrintLn(0, 2, "IP:" );//+ (String)WiFi.localIP());
      LCD_PrintLn(18, 2, WiFi.localIP().toString());
    
      //LCD_PrintLn(0, 3, TempWiFiCommand_2);

      z_Rotation = global_yawval;
      LCD_PrintLn( 0, 4, ("Rot: Z " + String(z_Rotation)));  //-- print the [yaw-pitch-roll] measured from the gyroscope
      
      //LCD_PrintLn(0, 5, "Pos: X " + (String)Path1_ArrayOfVectorWaypoints[0].Position.x + "  Z " + (String)Path1_ArrayOfVectorWaypoints[0].Position.z);
      //LCD_PrintLn(0, 5, "Pos: X " + (String)Robot_Position_X + "  Y " + (String)Robot_Position_Y);
    
    
      
      UpdateAndDraw_OBar();                                          //-- run the icon display function
      
      display.display();
    }
    
    
    
    void LCD_PrintLn( int xPos, int yLine, String text )  //-- my custom print line fuction that includes the cursor position and the text in one
    {
      display.setCursor( xPos , (yLine*9) );  //-- offset each of the lines by 9 pixels so they dont overlap
      display.println(text);
    }


    void UpdateAndDraw_OBar()
    {
    
      if (BatteryBlink_ELAPSED_TIMER >= BatteryBlink_TIMER_INTERVAL)
      {
        BatteryBlink_ELAPSED_TIMER = 0;
        
    
        if(BatteryLevel_Count <= 0){                     //-- if the count is 0, turn on the icon and set it to 0
          displayBattery = true;
          BatteryLevel_Quarters = 0;
        }
        else if(BatteryLevel_Count <= 4){                //-- if between 1 and 4 then set the charge level between 1 and 4
          BatteryLevel_Quarters = BatteryLevel_Count;
        }
        else if(BatteryLevel_Count == 5){                //-- if 5 then turn off the icon and reset the counter to -1
          displayBattery = false;
          BatteryLevel_Count = -1;
            
        }
        BatteryLevel_Count ++;                            //-- counter ++
      }
    
      if(displayBattery)
        darwBattery();
      if(charging)
        darwChargingIcon();
    
    
      Update_WifiIcon();
      Darw_WifiIcon();
      
    }
    
    void Update_WifiIcon()
    {
      if(WifiConnected == true)
      {
        WifiSymbol_State = 3;                       //-- show state 3                           [Dot +2 lines]
      }
      else
      {
        if(WifiBlink_ELAPSED_TIMER <= 700)          //-- for 700ms show state 1                 [Dot]
        {
          WifiSymbol_State = 1;
        }
        else if(WifiBlink_ELAPSED_TIMER <= 1400)    //-- between 700ms -> 1400ms show state 2   [Dot +1 line]
        {
          WifiSymbol_State = 2;
        }
        else if(WifiBlink_ELAPSED_TIMER <= 2100)    //-- between 1400ms -> 2100ms show state 3  [Dot +2 lines]
        {
          WifiSymbol_State = 3;
        }
        else if(WifiBlink_ELAPSED_TIMER <= 3200)    //-- between 2100ms -> 3200ms show state 0  [Dot + Error exclamation mark]
        {
          WifiSymbol_State = 0;
        }
        else if(WifiBlink_ELAPSED_TIMER >= 3200)    //-- after 3200ms or more reset the elapsed counter back to 0
        {
          WifiBlink_ELAPSED_TIMER = 0;
        }
      }
    
    
    }
    
    void darwBattery()
    {  
      if(BatteryLevel_Quarters == 0)
        display.drawBitmap(105, 0, batteryIcons[0], battery_WIDTH, battery_HEIGHT, 1);
      else if(BatteryLevel_Quarters == 1)
        display.drawBitmap(105, 0, batteryIcons[1], battery_WIDTH, battery_HEIGHT, 1);
      else if(BatteryLevel_Quarters == 2)
        display.drawBitmap(105, 0, batteryIcons[2], battery_WIDTH, battery_HEIGHT, 1);
      else if(BatteryLevel_Quarters == 3)
        display.drawBitmap(105, 0, batteryIcons[3], battery_WIDTH, battery_HEIGHT, 1);
      else if(BatteryLevel_Quarters == 4)
        display.drawBitmap(105, 0, batteryIcons[4], battery_WIDTH, battery_HEIGHT, 1);
    }
    
    void darwChargingIcon()
    {
      display.drawBitmap(98, 0, chargingIcon, chargingIcon_WIDTH, chargingIcon_HEIGHT, 1);
    }
    
    void Darw_WifiIcon()
    {  
      if(WifiSymbol_State == 0)
        display.drawBitmap(1, 1, WifiSymbol_Dots_0of3, WifiSymbol_Dots_WIDTH, WifiSymbol_Dots_HEIGHT, 1);
      else if(WifiSymbol_State == 1)
        display.drawBitmap(1, 1, WifiSymbol_Dots_1of3, WifiSymbol_Dots_WIDTH, WifiSymbol_Dots_HEIGHT, 1);
      else if(WifiSymbol_State == 2)
        display.drawBitmap(1, 1, WifiSymbol_Dots_2of3, WifiSymbol_Dots_WIDTH, WifiSymbol_Dots_HEIGHT, 1);
      else if(WifiSymbol_State == 3)
        display.drawBitmap(1, 1, WifiSymbol_Dots_3of3, WifiSymbol_Dots_WIDTH, WifiSymbol_Dots_HEIGHT, 1);
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//######################################################################################################
//-------------------------------------Loops for Gyroscope----------------------------------------------
    void Read_Gyroscope()
    {
      // if programming failed, don't try to do anything
      if (!dmpReady) return;
    
      if (mpu.dmpPacketAvailable())
      {
        global_yawval = GetIMUHeadingDeg(); //retreive the most current yaw value from IMU
      }
    
      //other program stuff block - executes every IMU_CHECK_INTERVAL_MSEC Msec
      //for this test program, there's nothing here except diagnostics printouts
      if (sinceLastIMUCheck >= IMU_CHECK_INTERVAL_MSEC)
      {
        sinceLastIMUCheck -= IMU_CHECK_INTERVAL_MSEC;
        //Serial.print(millis());
        //Serial.print("\t");
        /*
        Serial.println(global_yawval);
    
        if (global_fifo_count != 0)
        {
          Serial.print("FIFO Reset!");
          mpu.resetFIFO();
        }
        */
      }
    }


    float GetIMUHeadingDeg()
    {
      // At least one data packet is available
    
      mpuIntStatus = mpu.getIntStatus();
      fifoCount = mpu.getFIFOCount();// get current FIFO count
    
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
      {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        /*
        Serial.println(F("FIFO overflow!"));
        */
    
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
      }
      else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
      {
        // read all available packets from FIFO
        while (fifoCount >= packetSize) // Lets catch up to NOW, in case someone is using the dreaded delay()!
        {
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;
        }
        global_fifo_count = mpu.getFIFOCount(); //should be zero here
    
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      }
    
      float yawval = ypr[0] * 180 / M_PI;
      return  yawval;
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\







//VectorWaypoint Path1_VectorWaypoints[5];
//VectorWaypoint Path1_VectorWaypoints = new VectorWaypoint[5];
//VectorWaypoint *Path1_VectorWaypoints;

//######################################################################################################
//-------------------------------------------Loops for Navigation----------------------------------------------
    void WebServer_Command( String Command )
    {

      

      if(RecievingNewPathViaUpload == false)        //-- If we are not recieving a new path.
      {
        //UploadNewPath-[Length:7]
        if(Command.indexOf("/UploadNewPath-[Length:") >= 0)   //-- If the incomming command contains the [UploadNewPath] command
        {
          RecievingNewPathViaUpload = true;         //-- Set the [RecievingNewPathViaUpload] bool to true. Because we are now recieving a new path.

          //                         23 __     __ length-1
          //                              \   /
          //Command: UploadNewPath-[Length:|7|]
          NewPathViaUpload_Length = (Command.substring(23, Command.length()-1)).toInt();
        }
      }

      if(RecievingNewPathViaUpload == true)
      {
        //UploadNewPath/Waypoint-[Pos=(10,0,0)]-[Rot=(0,90,0)]
        if(Command.indexOf("/UploadNewPath/Waypoint") >= 0)
        {
          VectorWaypoint TempRecievedWaypoint;
          
          String PositionClip = Command.substring(Command.indexOf("Pos=(")+5, Command.length());                 //-- = 10,0,0)]-[Rot=(0,90,0)]
          PositionClip = PositionClip.substring(0, PositionClip.indexOf(")"));                    //-- = 10,0,0
          Vector3 TempPositionVector = StringToVector3(PositionClip);
          TempRecievedWaypoint.Position = TempPositionVector;

          String RotationClip = Command.substring(Command.indexOf("Rot=(")+5, Command.length());                 //-- = 10,0,0)]-[Rot=(0,90,0)]
          RotationClip = RotationClip.substring(0, RotationClip.indexOf(")"));                    //-- = 10,0,0
          Vector3 TempRotationVector = StringToVector3(RotationClip);
          TempRecievedWaypoint.Rotation = TempRotationVector;

          Path1_ArrayOfVectorWaypoints[NewPathViaUpload_WaypointCounter] = TempRecievedWaypoint;

          NewPathViaUpload_WaypointCounter++;
        }
      }
      
    }


    

    void NavigatePath_Path1()
    {
      CallSoundBassedOnRequest("StartPath");
      CallSoundBassedOnRequest("StartPath");
      delay(300);
      
      PathMove_Forward.Go();
      while(PathMove_Forward.Running == true){ delay(10); }
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);

      PathArcTurn_Left90.Go();
      while(PathArcTurn_Left90.Running == true){ delay(10); }
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);

      PathMove_Forward.Go();
      while(PathMove_Forward.Running == true){ delay(10); }
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);

      PathArcTurn_Right90.Go();
      while(PathArcTurn_Right90.Running == true){ delay(10); }
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);

      PathArcTurn_Right90.Go();
      while(PathArcTurn_Right90.Running == true){ delay(10); }
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);

      PathMove_Forward.Go();
      while(PathMove_Forward.Running == true){ delay(10); }
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);

      PathArcTurn_Left90.Go();
      while(PathArcTurn_Left90.Running == true){ delay(10); }
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);

      PathMove_Backward.Go();
      while(PathMove_Backward.Running == true){ delay(10); }
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);

      PathMove_Backward.Go();
      while(PathMove_Backward.Running == true){ delay(10); }
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);

      PathMove_Backward.Go();
      while(PathMove_Backward.Running == true){ delay(10); }
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);

      PathMove_Backward.Go();
      while(PathMove_Backward.Running == true){ delay(10); }
      delay(100);
      CallSoundBassedOnRequest("StartPath");
      CallSoundBassedOnRequest("StartPath");
    }










    String PathFromSDCard_String = "";
    String ArrayOfPathLines[50];

    void Read_PathFromSDCard()
    {
      PathFromSDCard_String = Path1_TextFile_AsString;
      String Temp_PathFromSDCard = Path1_TextFile_AsString;
      
      if(Map_TextFile_AsString != "")                                                     //-- if the Map file was sucessfully opened and read, the string should not be empty. So proceed:
      {
        Serial.println(Temp_PathFromSDCard);
        Serial.println("Character count: " + String(Temp_PathFromSDCard.length()));
        int NumberOfLines = TxtFileLineCount(Temp_PathFromSDCard);
        Serial.println("Line count: " + String(NumberOfLines));
  

        //-- create the array for each line string
        String ArrayOfLines[NumberOfLines /*- NumberOfDescriptionLines*/];                   //-- set the array size to be the number of lines

        //-- scan the whole TXT file and paste the lines into the array
        bool Done = false;
        int Count = 0;
        while(Done != true)
        {
          int NewPos = 0;
          NewPos = Temp_PathFromSDCard.indexOf('\n');
          //Serial.println("Index: " + String(NewPos));
          
          String Line = Temp_PathFromSDCard.substring(0, NewPos);
          //Serial.println("Line: " + Line);
          //if(count >= NumberOfDescriptionLines)                                                                      //-- if we are past the first three description lines, then:
          ArrayOfLines[Count] = Line;                                                       //-- add the newly detected line to the array
          
          String Temp = Temp_PathFromSDCard.substring(NewPos+1, Temp_PathFromSDCard.length());  //-- remove the newly detected line from the string
          //Serial.println("Remaining: \n" + Temp);
          Temp_PathFromSDCard = "";
          Temp_PathFromSDCard = Temp;
          Count++;
      
          if(NewPos == Temp_PathFromSDCard.length())
          {
            String Line = Temp_PathFromSDCard.substring(0, NewPos-1);
            ArrayOfLines[Count] = Line;                                         //-- add the newly detected line to the array
            
            Done = true;
          }
        }

        //-- print the length od the array to ensure the process was successful
        Serial.println("Line count2: " + String(ArrayLength(ArrayOfLines)));

        //-- enter all of the lines into a public string. This string is used for displaying on the hosted HTML website.
        PathFromSDCard_String = "";
        for(int i = 3; i < ArrayLength(ArrayOfLines); i++)
        {
          if(i < (ArrayLength(ArrayOfLines)-1))
            PathFromSDCard_String += (ArrayOfLines[i].substring(0, ArrayOfLines[i].length()-1) + '\n');    //-- remove the new-line craracter from the end of this line
          else
            PathFromSDCard_String += (ArrayOfLines[i].substring(0, ArrayOfLines[i].length()) + '\n');      //-- dont remove the last character, because there is no new-line character
        }
  


        //-- find the first line for this path. the "#RunDay&Time" line.
        int StartLineIndex = 0;
        for(int i = NumberOfLines; i > 0; i--){   //-- count backwards
          if(ArrayOfLines[i].indexOf("#RunDay&Time") != -1){
            StartLineIndex = i;
            Serial.println("Start line index: " + String(StartLineIndex));
            break;
          }
        }

        //-- find the last line for this path. the "End()" line.
        int EndLineIndex = 0;
        for(int i = NumberOfLines; i > StartLineIndex; i--){    //-- count backwards
          if(ArrayOfLines[i].indexOf("End()") != -1){
            EndLineIndex = i;
            Serial.println("End line index: " + String(EndLineIndex));
            break;
          }
        }
        

        //-- print each code line for the path to serial as debugging
        Serial.println("PathCode:");
        Serial.println("--------------------------------------------------");
        for(int i = StartLineIndex; i <= EndLineIndex; i++)              //-- start from after the legend and description starting at line 3
        {
          Serial.println(ArrayOfLines[i]);
        }
        Serial.println("--------------------------------------------------");
        Serial.println("");


        //-- run each code line for the path to move the robot
        for(int i = StartLineIndex; i <= EndLineIndex; i++)              //-- start from after the legend and description starting at line 3
        {
          ArrayOfPathLines[i] = ArrayOfLines[i];
        }
      }
      else
      {
        Serial.println("Path Navigation canceled because the .txt file read failed");
        Serial.println("");
      }
    }

    

    void NavigatePath_FromSDCard()
    {
      Last_Robot_Position_X = 0;    //-- reset these for demonstration purposes
      Last_Robot_Position_Y = 0;
      Robot_Position_X = 0;
      Robot_Position_Y = 0;

      
      Serial.println("---------------------------------");
      
      //-- run each code line for the path to move the robot
      for(int i = 0; i <= ArrayLength(ArrayOfPathLines); i++)              //-- start from after the legend and description starting at line 3
      {
        if(ArrayOfPathLines[i].indexOf("Rot1") != -1)
          Motion_Rotation(ArrayOfPathLines[i]);
        else if(ArrayOfPathLines[i].indexOf("Wait") != -1)
          Motion_Wait(ArrayOfPathLines[i]);
        else if(ArrayOfPathLines[i].indexOf("Pos1") != -1)
          Motion_Movement(ArrayOfPathLines[i]);

        UpdateTheLCD();
        
        Serial.println("---------------------------------");
      }
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\





//######################################################################################################
//------------------------------------------Robot Motion Code Blocks------------------------------------
    void Motion_Rotation( String PathCodeLine )
    {
      /*PathArcTurn_Right90.Go();
      while(PathArcTurn_Right90.Running == true){ delay(10); }*/

      int Rotation = (PathCodeLine.substring(PathCodeLine.indexOf("(")+1, PathCodeLine.indexOf(")"))).toInt();
      Serial.println("Motion_Rotation: " + String(Rotation));

      LocalHeadding = Rotation;

      if(Rotation < 0)  //-- if we are rotating counter-clockwise [left]
      {
        int AmountToRotate = ((abs(Rotation))/90);

        for(int i = 0; i < AmountToRotate; i++)
        {
          PathArcTurn_Left90.Go();
          while(PathArcTurn_Left90.Running == true){ delay(10); }
        }
      }
      else if(Rotation > 0)  //-- if we are rotating clockwise [right]
      {
        int AmountToRotate = ((Rotation)/90);
        
        for(int i = 0; i < AmountToRotate; i++)
        {
          PathArcTurn_Right90.Go();
          while(PathArcTurn_Right90.Running == true){ delay(10); }
        }
      }
      
      CallSoundBassedOnRequest("WaypointReached");
      delay(100);
    }

    void Motion_Wait( String PathCodeLine )
    {
      int Length = (PathCodeLine.substring(PathCodeLine.indexOf("(")+1, PathCodeLine.indexOf(")"))).toInt();
      Serial.println("Motion_Wait: " + String(Length));
      delay(Length);
    }

    void Motion_Movement( String PathCodeLine )
    {
      //                      From X( 20)  or X(-15)             after  "("           but before ")"
      //                                                                 |                        |
      int AmmountToMove = (PathCodeLine.substring(PathCodeLine.indexOf("(")+1, PathCodeLine.indexOf(")"))).toInt();
      Serial.println("AmmountToMove: " + String(AmmountToMove));

      if(AmmountToMove != 0) //-- if the X positions are different, then we are moving on the X axis.
      {
        for(int i = 0; i < abs(AmmountToMove); i++)
        {
          Last_Robot_Position_X = Robot_Position_X; //-- set the old values before we change our position
          Last_Robot_Position_Y = Robot_Position_Y;
          
          if(AmmountToMove > 0)
            Motion_MoveForward_Onece();
          if(AmmountToMove < 0)
            Motion_MoveBackward_Onece();

          UpdateLCD_Map(); //-- update the Map after the move
        }
        CallSoundBassedOnRequest("WaypointReached");
        delay(100);
      }
    }

    void Motion_MoveForward_Onece()
    {
      PathMove_Forward.Go();
      while(PathMove_Forward.Running == true){ delay(10); }

      if(LocalHeadding == 0 || LocalHeadding == 360)
        Robot_Position_Y += 1;
      else if(LocalHeadding == 90 || LocalHeadding == -270)
        Robot_Position_X += 1;
      else if(LocalHeadding == 180 || LocalHeadding == -180)
        Robot_Position_Y -= 1;
      else if(LocalHeadding == 270 || LocalHeadding == -90)
        Robot_Position_X -= 1;
    }

    void Motion_MoveBackward_Onece()
    {
      PathMove_Backward.Go();
      while(PathMove_Backward.Running == true){ delay(10); }

      if(LocalHeadding == 0 || LocalHeadding == 360)
        Robot_Position_Y -= 1;
      else if(LocalHeadding == 90 || LocalHeadding == -270)
        Robot_Position_X -= 1;
      else if(LocalHeadding == 180 || LocalHeadding == -180)
        Robot_Position_Y += 1;
      else if(LocalHeadding == 270 || LocalHeadding == -90)
        Robot_Position_X += 1;
    }
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


void UpdateLCD_Map()
{
  int Offset_X        = 10;
  int Offset_Y        = 120;
  int Size_X_InBlocks = 13;
  int Size_Y_InBlocks = 13;

  int BorderThickness = 3;
  int PixelSize       = 5;

  

  //tft.fillRect(Offset_X+(6*PixelSize)+((Last_Robot_Position_X)*PixelSize), Offset_Y+(7*PixelSize)+((-Last_Robot_Position_Y)*PixelSize), PixelSize, PixelSize, TFT_BLACK);

  //tft.fillRect(Offset_X+(6*PixelSize)+((Robot_Position_X)*PixelSize), Offset_Y+(7*PixelSize)+((-Robot_Position_Y)*PixelSize), PixelSize, PixelSize, TFT_GREEN);
}




bool IsNumerEven(int PassedNumber)
{
  if((PassedNumber & 1) == 0)
    return(true);               //-- if even, then true
  else
    return(true);               //-- if odd, the false
}


















//-------------------------------------------------------------------------------
