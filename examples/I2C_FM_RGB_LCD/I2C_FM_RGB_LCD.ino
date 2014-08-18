#include <Arduino.h>
#include <Wire.h>
#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;
#define BTN            2
#define ROTATE         A0

uint16_t gChipID = 0;
uint8_t RDA5807P_REGW[10];

#define I2C_ADDR       0x10

#define READ	        1
#define WRITE	        0

#define ADRW 	        0x20
#define ADRR 	        0x21


//#define                 _SHARE_CRYSTAL_24MHz_
//#define                 _SHARE_CRYSTAL_12MHz_
#define                 _SHARE_CRYSTAL_32KHz_
//#define                 _FM_STEP_50K_

//5807M,5807FP,5807NN,5807NP
uint8_t RDA5807N_initialization_reg[]={
#if defined(_SHARE_CRYSTAL_24MHz_)
    0xC4, 0x51, //02H:
#elif defined(_SHARE_CRYSTAL_12MHz_)
    0xC4, 0x11, //02H:
#elif defined(_SHARE_CRYSTAL_32KHz_)
    0xC4, 0x01,
#else
    0xC0, 0x01,
#endif
    0x00, 0x00,
    0x04, 0x00,
    0xC3, 0xad,  //05h
    0x60, 0x00,
    0x42, 0x12,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,  //0x0ah
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,  //0x10h
    0x00, 0x19,
    0x2a, 0x11,
    0xB0, 0x42,
    0x2A, 0x11,  //
    0xb8, 0x31,  //0x15h
    0xc0, 0x00,
    0x2a, 0x91,
    0x94, 0x00,
    0x00, 0xa8,
    0xc4, 0x00,  //0x1ah
    0xF7, 0xcF,
    0x12, 0x14,  //0x1ch
    0x80, 0x6F,
    0x46, 0x08,
    0x00, 0x86,  //10000110
    0x06, 0x61,  //0x20H
    0x00, 0x00,
    0x10, 0x9E,
    0x23, 0xC8,
    0x04, 0x06,
    0x0E, 0x1C,  //0x25H     //0x04 0x08
};

int16_t freq = 10430;
uint16_t vol = 8;



void setup()
{
    Wire.begin();
    Serial.begin(9600);
    Serial.println("Started");

    //=======================
    //rda5807 power on
    RDA5807P_PowerOnReset();
    RDA5807P_SetMute(false);

    //=======================
    pinMode(BTN, INPUT);
    pinMode(ROTATE, INPUT);
    //=======================
    RDA5807P_SetVolumeLevel(15);
    RDA5807P_SetFreq( freq );
    lcd.begin(16,2);
    lcd.print("I2C FM Receiver");
}

void loop()
{
  if (digitalRead(BTN) == 1)
  {
    delay(100);
    if (digitalRead(BTN) == 1)
      fmSeek();
    while(digitalRead(BTN) == 1);
  }
  setVolume();
}


unsigned int temp_vol;                                
void setVolume()                                      
{                                                     
  temp_vol = analogRead( ROTATE );                    
  if (abs(temp_vol - vol)>5)                          
  {                                                   
    vol = temp_vol;                                   
    unsigned char hex_vol = map(vol, 0, 1023, 0, 0xf);
    RDA5807P_SetVolumeLevel(hex_vol);               
  }                                                   
}                                                     
                                                      
void fmSeek()                                         
{                                                     
  Serial.println("Start seeking...");                 
                                                      
  do{                                                 
      freq += 10;                                     
      if (freq > 10800) freq = 6500;                  
  } while(!RDA5807P_ValidStop(freq)); 
  lcd.setCursor(0, 1);  
  lcd.print(((float)freq)/100.0f);
  lcd.print("MHz");//((float)freq)/100.0f);  
 // lcd.display();
    delay(500);  
  Serial.print("Stable Freq:");                       
  Serial.print(((float)freq)/100.0f);                 
  Serial.println("MHz");                              
}                                                     



//===========================================================
// FM functions
//===========================================================
unsigned char OperationRDAFM_2w(unsigned char operation, unsigned char *data, int numBytes)
{
    if(operation == READ)
    {
        Wire.requestFrom(I2C_ADDR, numBytes);
        for(int i=0;i<numBytes;i++)
        {
            *data++ = Wire.read();
        }
    }else
    {
        Wire.beginTransmission(I2C_ADDR);
        for(int i=0;i<numBytes;i++)
        {
            Wire.write(*data++);
        }
        Wire.endTransmission();
    }
    return 0;
}


/**
 * @brief Reset RDA5807P while power on RDA5807P
 * @author RDA RDA Ri'an Zeng
 * @date 2008-11-05
 * @param void
 * @return void
 * @retval
 */
void  RDA5807P_PowerOnReset(void)
{
    RDA5807P_Intialization();
}


/**
 * @brief RDA5807P power off function
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param void
 * @return void
 * @retval
 */
void  RDA5807P_PowerOffProc(void)
{
    RDA5807P_REGW[1] &= (~1);
    OperationRDAFM_2w(WRITE, &(RDA5807P_REGW[0]), 2);
}

/**
 * @brief Set RDA5807P into mute mode
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param bool mute: if mute is true,then set mute; if mute is false,then set no mute
 * @return void
 * @retval
 */
void RDA5807P_SetMute(boolean mute)
{
    if(mute)
        RDA5807P_REGW[0] &=  ~(1<<6);
    else
        RDA5807P_REGW[0] |= 1<<6;

    OperationRDAFM_2w(WRITE, &(RDA5807P_REGW[0]), 2);
    delay(50);    //Dealy 50 ms
}


/*************************************************
 * @brief Set frequency function
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param int16_t curFreq:frequency value
 * @return void
 * @retval
 ***********************************************/
void RDA5807P_SetFreq(int16_t curFreq)
{
    uint16_t curChan;

    if((curFreq >= 6500)&&(curFreq < 7600))
    {
        curChan = (curFreq - 6500)/10;
        RDA5807P_REGW[3] = 0x0c;
    }
    else if((curFreq >= 7600)&&(curFreq < 10800))
    {
        curChan = (curFreq - 7600)/10;
        RDA5807P_REGW[3] = 0x08;
    }

    //SetNoMute
    RDA5807P_REGW[0] |= 1<<6;
    RDA5807P_REGW[2]=curChan>>2;
    RDA5807P_REGW[3]=(((curChan&0x0003)<<6)|0x10) | (RDA5807P_REGW[3]&0x0f);    //set tune bit

    OperationRDAFM_2w(WRITE, &(RDA5807P_REGW[0]), 4);
    delay(50);     //Delay five ms
}

/**
 * @brief Station judge for auto search
 * @In auto search mode,uses this function to judge the frequency if has a station
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param int16_t freq:frequency value
 * @return bool: if return true,the frequency has a true station;otherwise doesn't have a station
 * @retval
 */
boolean RDA5807P_ValidStop(int freq)
{ 
    uint8_t RDA5807P_reg_data[4]={0};
    uint8_t falseStation = 0;
    uint8_t i=0;
    uint16_t curChan;
    
    //curChan=RDA5807P_FreqToChan(freq);
    //03H 3:2 BAND??
    //03H 15:6 ??CHAN
    if((freq >= 6500)&&(freq < 7600))
    {
      curChan = (freq - 6500)/10;
      RDA5807P_REGW[3] = 0x0c;
    }
    else if((freq >= 7600)&&(freq < 10800))
    {
      curChan = (freq - 7600)/10;
      RDA5807P_REGW[3] = 0x08;
    }
    
    //SetNoMute
    //02H 14
    RDA5807P_REGW[0] |=	1<<6;
    
    RDA5807P_reg_data[0]=RDA5807P_REGW[0];
    RDA5807P_reg_data[1]=RDA5807P_REGW[1];
    RDA5807P_reg_data[2]=curChan>>2;//03H 15:8 CHAN
    RDA5807P_reg_data[3]=(((curChan&0x0003)<<6)|0x10) | (RDA5807P_REGW[3]&0x0f);//
    OperationRDAFM_2w(WRITE,&(RDA5807P_reg_data[0]), 4);
    
    delay(50);    //Dealy 25 ms
    
    if (0x5808 == gChipID)
        OperationRDAFM_2w(READ,&(RDA5807P_reg_data[0]), 4);	//
    else
    {
        do
      {
          i++;
          if(i>5) return 0;
    
          delay(30);
          //read REG0A&0B
          OperationRDAFM_2w(READ,&(RDA5807P_reg_data[0]), 4);
        }while((RDA5807P_reg_data[0]&0x40)==0);
    }
    
    //check FM_TURE
    if((RDA5807P_reg_data[2] &0x01)==0) falseStation=1;//0B 8  FM TRUE
    
    if(freq==9600) falseStation=1;//
    
    if (falseStation==1)
      return 0;
    else
      return 1;
}

/**
 * @brief Get the signal level(RSSI) of the current frequency
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param int16_t curf:frequency value
 * @return uint8_t: the signal level(RSSI)
 * @retval
 */
uint8_t RDA5807P_GetSigLvl( int16_t curf )
{
    uint8_t RDA5807P_reg_data[4]={0};
    OperationRDAFM_2w(READ,&(RDA5807P_reg_data[0]), 4);
    delay(50);    //Dealy 50 ms
    return  (RDA5807P_reg_data[2]>>1);  /*??rssi*/
}

/**
 * @brief Set FM volume
 * @It has better use the system volume operation to replace this function
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param uint8_t level: volume value
 * @return void
 * @retval
 */
void RDA5807P_SetVolumeLevel(uint8_t level)
{
    uint8_t RDA5807P_reg_data[8];
    uint8_t i = 0;

    for (i=0;i<8;i++)
        RDA5807P_reg_data[i] = RDA5807P_REGW[i];

    RDA5807P_reg_data[7]=(( RDA5807P_REGW[7] & 0xf0 ) | (level & 0x0f));

    RDA5807P_reg_data[3] &= (~(0x10));//disable tune

    OperationRDAFM_2w(WRITE, &(RDA5807P_reg_data[0]), 8);
    delay(50);    //Dealy 50 ms
}

/**
 * @brief Initialize RDA5807P
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param void
 * @return bool:if true,the operation is successful;otherwise is failed
 * @retval
 **/
boolean  RDA5807P_Intialization(void)
{
    uint8_t error_ind = 0;
    uint8_t RDA5807P_REGR[10]={0x0};
    uint8_t i = 0;

    RDA5807P_REGW[0] = 0x00;
    RDA5807P_REGW[1] = 0x02;

    error_ind = OperationRDAFM_2w(WRITE, (uint8_t *)&RDA5807P_REGW[0], 2);//soft reset
    delay(50);

    error_ind = OperationRDAFM_2w(READ, (uint8_t *)&RDA5807P_REGR[0], 10);
    delay(50);

    gChipID = RDA5807P_REGR[8];
    gChipID = ((gChipID << 8) | RDA5807P_REGR[9]);

    Serial.print("Chip ID: 0x");
    Serial.println(gChipID, HEX);

    for (i=0;i<8;i++)
        RDA5807P_REGW[i] = RDA5807N_initialization_reg[i];

    error_ind = OperationRDAFM_2w(WRITE, (uint8_t *)&RDA5807N_initialization_reg[0], 2); //power up
    delay(600);
    Serial.println(sizeof(RDA5807N_initialization_reg));
    error_ind = OperationRDAFM_2w(WRITE, (uint8_t *)&RDA5807N_initialization_reg[0], sizeof(RDA5807N_initialization_reg));
    
    delay(50);         //Dealy 50 ms

    if (error_ind )
       return 0;
    else
       return 1;
}
