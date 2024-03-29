
//Use this website for custom icons: https://javl.github.io/image2cpp/
// in section 2 "2. Image Settings" select "Background color:" = trasparent.
//
//You can check how it looks in section 3 "3. Preview" it will appear in a little green box
//in section 4 "4. Output" set the "Identifier/Prefix:" to be a space character " " to remove the defult "epd_bitmap_" or "myBitmap"


//##########################################################
//Batteries
const unsigned char batteryIcons[] [42/*Length of the array*/] PROGMEM = { 
  {// 'battery0, 23x14px
  0x3f, 0xff, 0xfe, 0x20, 0x00, 0x02, 0x20, 0x00, 0x02, 0x20, 0x00, 0x02, 0xe0, 0x00, 0x02, 0xe0, 
  0x00, 0x02, 0xe0, 0x00, 0x02, 0xe0, 0x00, 0x02, 0xe0, 0x00, 0x02, 0xe0, 0x00, 0x02, 0x20, 0x00, 
  0x02, 0x20, 0x00, 0x02, 0x20, 0x00, 0x02, 0x3f, 0xff, 0xfe} , 
  {// 'battery1, 23x14px
  0x3f, 0xff, 0xfe, 0x20, 0x00, 0x02, 0x20, 0x00, 0x1a, 0x20, 0x00, 0x1a, 0xe0, 0x00, 0x1a, 0xe0, 
  0x00, 0x1a, 0xe0, 0x00, 0x1a, 0xe0, 0x00, 0x1a, 0xe0, 0x00, 0x1a, 0xe0, 0x00, 0x1a, 0x20, 0x00, 
  0x1a, 0x20, 0x00, 0x1a, 0x20, 0x00, 0x02, 0x3f, 0xff, 0xfe} , 
  {// 'battery2, 23x14px
  0x3f, 0xff, 0xfe, 0x20, 0x00, 0x02, 0x20, 0x01, 0xda, 0x20, 0x01, 0xda, 0xe0, 0x01, 0xda, 0xe0, 
  0x01, 0xda, 0xe0, 0x01, 0xda, 0xe0, 0x01, 0xda, 0xe0, 0x01, 0xda, 0xe0, 0x01, 0xda, 0x20, 0x01, 
  0xda, 0x20, 0x01, 0xda, 0x20, 0x00, 0x02, 0x3f, 0xff, 0xfe} , 
  {// 'battery3, 23x14px
  0x3f, 0xff, 0xfe, 0x20, 0x00, 0x02, 0x20, 0x3d, 0xda, 0x20, 0x3d, 0xda, 0xe0, 0x3d, 0xda, 0xe0, 
  0x3d, 0xda, 0xe0, 0x3d, 0xda, 0xe0, 0x3d, 0xda, 0xe0, 0x3d, 0xda, 0xe0, 0x3d, 0xda, 0x20, 0x3d, 
  0xda, 0x20, 0x3d, 0xda, 0x20, 0x00, 0x02, 0x3f, 0xff, 0xfe} , 
  {// 'battery4, 23x14px
  0x3f, 0xff, 0xfe, 0x20, 0x00, 0x02, 0x2f, 0xbd, 0xda, 0x2f, 0xbd, 0xda, 0xef, 0xbd, 0xda, 0xef, 
  0xbd, 0xda, 0xef, 0xbd, 0xda, 0xef, 0xbd, 0xda, 0xef, 0xbd, 0xda, 0xef, 0xbd, 0xda, 0x2f, 0xbd, 
  0xda, 0x2f, 0xbd, 0xda, 0x20, 0x00, 0x02, 0x3f, 0xff, 0xfe}
  
  };
#define battery_WIDTH    23
#define battery_HEIGHT   14
//##########################################################



//##########################################################
//Wifi Connected
const unsigned char WifiConnectedIcon [] PROGMEM =
{
  0xf0, 0x00, 0xfe, 0x00, 0xff, 0x80, 0x0f, 0xc0, 0x03, 0xe0, 0xe0, 0xf0, 0xf8, 0x70, 0x3c, 0x78, 
  0x0e, 0x38, 0x07, 0x38, 0xc3, 0x1c, 0xe3, 0x9c, 0xf1, 0x9c, 0xf1, 0x9c
};
#define WifiConnectedIcon_WIDTH    14
#define WifiConnectedIcon_HEIGHT   14
//##########################################################



//##########################################################
//WifiSymbol_Dots

// 'WifiSymbol_Dots_0of3', 14x14px
const unsigned char  WifiSymbol_Dots_0of3 [] PROGMEM = {
  0x07, 0xf0, 0x07, 0xf0, 0x07, 0xf0, 0x07, 0xf0, 0x07, 0xf0, 0x07, 0xf0, 0x03, 0xe0, 0x03, 0xe0, 
  0x01, 0xc0, 0x00, 0x00, 0xc1, 0xc0, 0xe1, 0xc0, 0xf1, 0xc0, 0xf0, 0x80
};

// 'WifiSymbol_Dots_1of3', 14x14px
const unsigned char  WifiSymbol_Dots_1of3 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xe0, 0x00, 0xf0, 0x00, 0xf0, 0x00
};

// 'WifiSymbol_Dots_2of3', 14x14px
const unsigned char  WifiSymbol_Dots_2of3 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0xf8, 0x00, 0x3c, 0x00, 
  0x0e, 0x00, 0x07, 0x00, 0xc3, 0x00, 0xe3, 0x80, 0xf1, 0x80, 0xf1, 0x80
};

// 'WifiSymbol_Dots_3of3', 14x14px
const unsigned char  WifiSymbol_Dots_3of3 [] PROGMEM = {
  0xf0, 0x00, 0xfe, 0x00, 0xff, 0x80, 0x0f, 0xc0, 0x03, 0xe0, 0xe0, 0xf0, 0xf8, 0x70, 0x3c, 0x78, 
  0x0e, 0x38, 0x07, 0x38, 0xc3, 0x1c, 0xe3, 0x9c, 0xf1, 0x9c, 0xf1, 0x9c
};

#define WifiSymbol_Dots_WIDTH    14
#define WifiSymbol_Dots_HEIGHT   14
//##########################################################




const unsigned char chargingIcon [] PROGMEM = {
  // 'chargingIcon, 6x14px
  0x48, 0x48, 0xfc, 0xfc, 0xfc, 0x20, 0x20, 0x20, 0x20, 0x30, 0x10, 0x10, 0x10, 0x10
};
#define chargingIcon_WIDTH    6
#define chargingIcon_HEIGHT   14
