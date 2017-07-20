#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <SD.h>
#include <UTFT_SPI.h>
#include "memorysaver.h"
#include <CurieNeuronsPro.h>

CurieNeurons hNN;
int dist=0, cat=0, nid=0, ncount=0;
int catLearn=1, nextCat=1;
#define MAX_LEN 128 
byte neuron[MAX_LEN +8]; 
int fw=320; 
int fh=240;
int rw = 121, rh = 121;
int rleft = (fw - rw)/2;
int rtop = (fh - rh) /2;
int rright = rleft + rw;
int rbottom= rtop + rh;
int bw = 11, bh = 11;
int hb = rw/bw, vb= rh/bh;
int vlen= hb*vb;
int subsample[121];
byte subsampleFeat[121]; 
int vlen2 = 96; 
int rgbhisto[96];
byte rgbhistoFeat[96];
int profdiv=2;
int hb3= rw/ profdiv, vb3= rh/profdiv;
int vlen3 = hb3 + vb3;
int cprofile[120]; //buffer
byte cprofileFeat[120]; // byte array for the neurons
bool SD_detected=false;
File SDfile;
#define SD_CS 9
char SDfilename[12] = "vectors.txt";
int sampleID=0; 
const int SPI_CS =10;
ArduCAM myCAM(OV2640, SPI_CS);
UTFT myGLCD(SPI_CS);

void setup() {
  uint8_t vid = 0, pid = 0;
  uint8_t temp = 0;
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);
  pinMode(SPI_CS, OUTPUT);
  SPI.begin();
  Serial.println("Welcome to the SMART_ArduCAM demo!");
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55) {
    Serial.println("SPI interface Error!");
    Serial.println("Check your wiring, make sure using the correct SPI port and chipselect pin");
  }
  myGLCD.InitLCD();
  extern uint8_t BigFont[];
  myGLCD.setFont(BigFont);
  if (SD.begin(SD_CS))
  {
    SD_detected=true;
    if (SD.exists(SDfilename)) SD.remove(SDfilename);
  }
  else
  {
      displayLCD_res("Warning: SD slot empty", 10,10);
      delay(100);
      Serial.println("Insert SD card if you wish to save vector data");
  }
  myCAM.InitCAM();
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
  if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))) {
    Serial.println("Can't find OV2640 module!");
  } else {
    Serial.println("OV2640 detected.");
  }
  if (hNN.begin()==0) Serial.println("\nNeural network is initialized!");
  else Serial.println("\nNeural network is NOT properly connected!");
  Serial.print("Image width="); Serial.print(fw); Serial.print(", height="); Serial.println(fh);
  Serial.print("ROI width="); Serial.print(rw); Serial.print(", height="); Serial.println(rh);
  displayLCD_res("Ready", 10,10);
  delay(100);
}

void loop() {
  while (1) {
    if (!myCAM.get_bit(ARDUCHIP_TRIG, VSYNC_MASK)) 
    {
      myCAM.set_mode(MCU2LCD_MODE);
      myGLCD.resetXY();
      myCAM.set_mode(CAM2LCD_MODE);
      while (!myCAM.get_bit(ARDUCHIP_TRIG, VSYNC_MASK));      
      getFeatureVectors();
      recognize();
    } 
    else if (Serial.available())
    {         
      getFeatureVectors(); 
      catLearn = Serial.read();
      catLearn = catLearn - 48;
      Serial.print("Command has been received:");
      Serial.print("catLearn="); Serial.println(catLearn);
      while(Serial.available())
      {
        Serial.read();
      } 
      learn(catLearn);
      if (SD_detected==true)
      {
          saveKnowledge();
          sampleID++;
      }
    }
  }
}

void getFeatureVectors() {
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  myCAM.read_fifo(); // Read the first dummy byte
  myCAM.read_fifo(); // Read the first dummy byte
  char VH, VL;
  int indexFeat1X, indexFeat1Y, indexFeat3X, indexFeat3Y, index = 0, color;
  uint8_t r, g, b, greylevel;
   for (int i = 0; i < vlen; i++) subsample[i] = 0; 
    for (int i = 0; i < vlen2; i++) rgbhisto[i] = 0; 
     for (int i = 0; i < vlen3; i++) cprofile[i] = 0; 
  for (int y = 0 ; y < fh ; y++)
  {
    for (int x = 0 ; x < fw ; x++)
    {
      VH = myCAM.read_fifo();
      VL = myCAM.read_fifo();
      color = (VH << 8) + VL;
      if (y >= rtop && y < rbottom)
      {       
        indexFeat1Y= (y - rtop) / bh;
        indexFeat3Y = (y - rtop) / profdiv;
        if (x >= rleft && x < rright)
        {             
          r = ((color >> 11) & 0x1F);
          g = ((color >> 5) & 0x3F);
          b = (color & 0x001F);
          greylevel= r+g+b; // byte value
          rgbhisto[r]++;
          rgbhisto[g + 32]++;
          rgbhisto[b + 64]++;
          indexFeat1X= (x - rleft) / bw;
          indexFeat3X = (x - rleft) / profdiv;
          if ((indexFeat1X < hb) & (indexFeat1Y < vb))
          {
              index = (indexFeat1Y * hb) + indexFeat1X;
              subsample[index] = subsample[index] + greylevel;
          }
          if ((indexFeat3X < hb3) & (indexFeat3Y < vb3))
          {
              cprofile[indexFeat3X] = cprofile[indexFeat3X] + greylevel;
              cprofile[indexFeat3Y + hb3] = cprofile[indexFeat3Y + hb3] + greylevel;
          }
        }
      }
    }
  }
  for (int i = 0; i < vlen; i++) subsampleFeat[i] = (byte)(subsample[i] / (bw * bh));
  for (int i = 0; i < vlen2; i++) rgbhistoFeat[i] = (byte)(rgbhisto[i] * 255 / (rw * rh));
  for (int i = 0; i < hb3; i++) cprofileFeat[i] = (byte)(cprofile[i] / hb3);
  for (int i = 0; i < vb3; i++) cprofileFeat[i + hb3] = (byte)(cprofile[i + hb3] / vb3);
}

void recognize() 
{
  int cat1=0, cat2=0, cat3=0;
  hNN.GCR(1);
  hNN.classify(subsampleFeat, vlen, &dist, &cat1, &nid);
  hNN.GCR(2);
  hNN.classify(rgbhistoFeat, vlen2, &dist, &cat2, &nid);
  hNN.GCR(3);
  hNN.classify(cprofileFeat, vlen3, &dist, &cat3, &nid);
  int matches=0;
  if ((cat1==cat2) & (cat1==cat3)) { cat=cat1; matches=3;}
  else if ((cat1==cat2) | (cat1==cat3)) {cat=cat1; matches=2;}
  else if (cat2==cat3) { cat=cat2; matches=2;}
  else cat=0xFFFF;
  char tmpStr[10];
  char Str[40] = {""};
  if (cat!=0xFFFF) 
  {
      strcat(Str,"Cat=");
      itoa (cat, tmpStr, 10);
      strcat(Str, tmpStr);
      strcat(Str," matches=");
      itoa (matches, tmpStr, 10);
      strcat(Str, tmpStr);
      Serial.println(Str);    
  }
  else
  {
    strcat(Str,"Unknown"); 
    Serial.println(Str);   
  }
  displayLCD_res(Str, 5,5);
  delay(10); 
}

void learn(int Category) 
{
  hNN.GCR(1);
  hNN.learn(subsampleFeat, vlen, Category);
  hNN.GCR(2);
  hNN.learn(rgbhistoFeat, vlen2, Category);
  hNN.GCR(3);
  int ncount = hNN.learn(cprofileFeat, vlen3, Category);
  char tmpStr[10];
  char Str[40] = {""};
  if (Category !=0)
  {
    strcat(Str, "Neurons=");
    itoa (ncount, tmpStr, 10);
    strcat(Str, tmpStr);
  }
  else
  {
    strcat(Str,"Forget");
  }
  Serial.println(Str);
  displayLCD_res(Str, 5, 220);
  delay(100); 
}

void saveKnowledge()
{
 
  char knFilename[12] = "neurons.knf";
  if (ncount==0) return;
  if (SD.exists(knFilename)) SD.remove(knFilename);
    SDfile = SD.open(knFilename, FILE_WRITE);
    if(! SDfile)
    {
      Serial.print(knFilename); Serial.println(" file open failed");
      return;
   }
   Serial.print("\nSaving to SD card "); Serial.println(knFilename);
    for (int i=0; i<ncount; i++)
    {
      hNN.readNeuron(i, neuron);      
      SDfile.write(neuron, MAX_LEN +8);
    }
    SDfile.close();
    Serial.println("Knowledge saved!");
}

void displayLCD_res(char* Str, int x, int y)
{
  myCAM.set_mode(MCU2LCD_MODE);
  myGLCD.setColor(255, 0, 225);
  myGLCD.print(Str,x,y,0);
  myGLCD.setColor(255, 0, 225);
  myGLCD.drawRect(rleft, rtop, rright, rbottom);
}
