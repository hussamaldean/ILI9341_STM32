#include "GLCD_ILI9341.h"
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common

//**********************Start of ILI9341 LCD APIs*********************/

void ILI9341_GPIO_Init(void)
{ 
  GPIO_InitTypeDef GPIO_InitStruct;
  __GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin=GPIO_PIN_4|GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull=GPIO_NOPULL;
	GPIO_InitStruct.Speed=GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
}

SPI_HandleTypeDef hspi1;

/* SPI init function */
void ILI9341_SPI_Init(void)
{

	hspi1.Instance=SPI1;
	hspi1.Init.Mode=SPI_MODE_MASTER;
	hspi1.Init.Direction=SPI_DIRECTION_1LINE;
	hspi1.Init.DataSize=SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity=SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase=SPI_PHASE_1EDGE;
	hspi1.Init.NSS=SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit=SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode=SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial=10;
	HAL_SPI_Init(&hspi1);
}

void spi1_8b_init(void){
 hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
 HAL_SPI_Init(&hspi1);
}
void spi1_16b_init(void){
 hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
 HAL_SPI_Init(&hspi1);
}	
	
void lcd9341_senddata(unsigned char data) {
  HAL_SPI_Transmit(&hspi1, &data,1,0x1);
}
void lcd9341_send16bData(uint8_t msb,uint8_t lsb) {
	uint8_t masData[]={lsb,msb};
	HAL_SPI_Transmit(&hspi1,masData,1,0x1);
}



void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

 __SPI1_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin=GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5;
	GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull=GPIO_NOPULL;
	GPIO_InitStruct.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate=GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);


}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{



    __SPI1_CLK_DISABLE();
HAL_GPIO_DeInit(GPIOA,GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5);
}


static uint16_t _width = ILI9341_TFTWIDTH;
static uint16_t _height = ILI9341_TFTHEIGHT;
uint32_t StX=0; 
uint32_t StY=0; 
uint16_t StTextColor = YELLOW;
static uint8_t ColStart=0, RowStart=0; 
void static writedata(uint8_t d);
void static setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void static writecommand(uint8_t c) ;
void setTextSize1(uint8_t s_x, uint8_t s_y);
void ILI9341_SetCursor(uint32_t newX, uint32_t newY);
void standard_Init_Cmd(void);


void ILI9341_Init(void) {

 ILI9341_GPIO_Init();
 ILI9341_SPI_Init();
 standard_Init_Cmd();
  ILI9341_SetCursor(0,0);
  StTextColor = YELLOW;
  ILI9341_FillScreen(WHITE);        
}

void ILI9341_SetCursor(uint32_t newX, uint32_t newY){

  StX = newX;
  StY = newY;
}

void ILI9341_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  for (int16_t i = x; i < x + w; i++) {
  ILI9341_DrawFastVLine(i, y, h, color);
  }
}

void ILI9341_FillScreen(uint16_t color) {
  ILI9341_FillRect(0, 0, _height, _width, color);  
}

void static writecommand(uint8_t c) {
  LCD_DC0; //Set DC low
  lcd9341_senddata(c);
}
void static writedata(uint8_t d) {
  LCD_DC1;//Set DC HIGH
   lcd9341_senddata(d);
}

void static setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {

  writecommand(ILI9341_CASET); // Column addr set
  writedata(0x00);
  writedata(x0+ColStart);     // XSTART
  writedata(0x00);
  writedata(x1+ColStart);     // XEND

  writecommand(ILI9341_RASET); // Row addr set
  writedata(0x00);
  writedata(y0+RowStart);     // YSTART
  writedata(0x00);
  writedata(y1+RowStart);     // YEND

  writecommand(ILI9341_RAMWR); // write to RAM
}

void ILI9341_SendCommand(unsigned char ch){lcd9341_sendCmd(ch);}
void write_data16(unsigned char h){lcd9341_sendData(h);}
void ILI9341_SendData(unsigned char h){writedata(h);}
void standard_Init_Cmd(void)
{
	LCD_CS1;
	LCD_RST0;
	HAL_Delay(1);
	LCD_RST1;
	HAL_Delay(10);
	LCD_RST0;
	HAL_Delay(10);
	LCD_RST1;
	HAL_Delay(120);
	LCD_CS0;
	 ILI9341_SendCommand (ILI9341_RESET); // software reset comand
   HAL_Delay(100);
   ILI9341_SendCommand (ILI9341_DISPLAY_OFF); // display off
   //------------power control------------------------------
   ILI9341_SendCommand (ILI9341_POWER1); // power control
   ILI9341_SendData   (0x26); // GVDD = 4.75v
   ILI9341_SendCommand (ILI9341_POWER2); // power control
   ILI9341_SendData   (0x11); // AVDD=VCIx2, VGH=VCIx7, VGL=-VCIx3
   //--------------VCOM-------------------------------------
   ILI9341_SendCommand (ILI9341_VCOM1); // vcom control
   ILI9341_SendData   (0x35); // Set the VCOMH voltage (0x35 = 4.025v)
   ILI9341_SendData   (0x3e); // Set the VCOML voltage (0x3E = -0.950v)
   ILI9341_SendCommand (ILI9341_VCOM2); // vcom control
   ILI9341_SendData   (0xbe);

   //------------memory access control------------------------
   ILI9341_SendCommand (ILI9341_MAC); // memory access control
   ILI9341_SendData(0x48);

   ILI9341_SendCommand (ILI9341_PIXEL_FORMAT); // pixel format set
   ILI9341_SendData   (0x55); // 16bit /pixel

	 ILI9341_SendCommand(ILI9341_FRC);
   ILI9341_SendData(0);
   ILI9341_SendData(0x1F);
   //-------------ddram ----------------------------
   ILI9341_SendCommand (ILI9341_COLUMN_ADDR); // column set
   ILI9341_SendData   (0x00); // x0_HIGH---0
   ILI9341_SendData   (0x00); // x0_LOW----0
   ILI9341_SendData   (0x00); // x1_HIGH---240
   ILI9341_SendData   (0x1D); // x1_LOW----240
   ILI9341_SendCommand (ILI9341_PAGE_ADDR); // page address set
   ILI9341_SendData   (0x00); // y0_HIGH---0
   ILI9341_SendData   (0x00); // y0_LOW----0
   ILI9341_SendData   (0x00); // y1_HIGH---320
   ILI9341_SendData   (0x27); // y1_LOW----320

   ILI9341_SendCommand (ILI9341_TEARING_OFF); // tearing effect off
   //LCD_write_cmd(ILI9341_TEARING_ON); // tearing effect on
   //LCD_write_cmd(ILI9341_DISPLAY_INVERSION); // display inversion
   ILI9341_SendCommand (ILI9341_Entry_Mode_Set); // entry mode set
   // Deep Standby Mode: OFF
   // Set the output level of gate driver G1-G320: Normal display
   // Low voltage detection: Disable
   ILI9341_SendData   (0x07);
   //-----------------display------------------------
   ILI9341_SendCommand (ILI9341_DFC); // display function control
   //Set the scan mode in non-display area
   //Determine source/VCOM output in a non-display area in the partial display mode
   ILI9341_SendData   (0x0a);
   //Select whether the liquid crystal type is normally white type or normally black type
   //Sets the direction of scan by the gate driver in the range determined by SCN and NL
   //Select the shift direction of outputs from the source driver
   //Sets the gate driver pin arrangement in combination with the GS bit to select the optimal scan mode for the module
   //Specify the scan cycle interval of gate driver in non-display area when PTG to select interval scan
   ILI9341_SendData   (0x82);
   // Sets the number of lines to drive the LCD at an interval of 8 lines
   ILI9341_SendData   (0x27);
   ILI9341_SendData   (0x00); // clock divisor

   ILI9341_SendCommand (ILI9341_SLEEP_OUT); // sleep out
   HAL_Delay(100);
   ILI9341_SendCommand (ILI9341_DISPLAY_ON); // display on
   HAL_Delay(100);
   ILI9341_SendCommand (ILI9341_GRAM); // memory write
   HAL_Delay(5);
  
}


void static pushColor(uint16_t color) {
  writedata((uint8_t)(color >> 8));
  writedata((uint8_t)color);
}

void ILI9341_DrawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x,y);

  pushColor(color);
}

void ILI9341_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
  uint8_t hi = color >> 8, lo = color;

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((y+h-1) >= _height) h = _height-y;
  setAddrWindow(x, y, x, y+h-1);

  while (h--) {
    writedata(hi);
    writedata(lo);
  }
}

void ILI9341_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  uint8_t hi = color >> 8, lo = color;
	
  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  while (w--) {
    writedata(hi);
    writedata(lo);
  }
}

void ILI9341_DrawBitmap(int16_t x, int16_t y, const uint16_t *image, int16_t w, int16_t h){
  int16_t skipC = 0;                      
  int16_t originalWidth = w;              
  int i = w*(h - 1);

  if((x >= _width) || ((y - h + 1) >= _height) || ((x + w) <= 0) || (y < 0)){
    return;                             
  }
  if((w > _width) || (h > _height)){    
   
    return;
  }
  if((x + w - 1) >= _width){            
    skipC = (x + w) - _width;           
    w = _width - x;
  }
  if((y - h + 1) < 0){                  
    i = i - (h - y - 1)*originalWidth;  
    h = y + 1;
  }
  if(x < 0){                            
    w = w + x;
    skipC = -1*x;                       
    i = i - x;                          
    x = 0;
  }
  if(y >= _height){                     
    h = h - (y - _height + 1);
    y = _height - 1;
  }

  setAddrWindow(x, y-h+1, x+w-1, y);

  for(y=0; y<h; y=y+1){
    for(x=0; x<w; x=x+1){
                                        
      writedata((uint8_t)(image[i] >> 8));
                                        
      writedata((uint8_t)image[i]);
      i = i + 1;                       
    }
    i = i + skipC;
    i = i - 2*originalWidth;
  }
}

void ILI9341_DrawCharS(int16_t x, int16_t y, char c, int16_t textColor, int16_t bgColor, uint8_t size){
  uint8_t line; 
  int32_t i, j;
  if((x >= _width)            || 
     (y >= _height)           || 
     ((x + 5 * size - 1) < 0) || 
     ((y + 8 * size - 1) < 0))   
    return;

  for (i=0; i<6; i++ ) {
    if (i == 5)
      line = 0x0;
    else
      line = Font[(c*5)+i];
    for (j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) 
          ILI9341_DrawPixel(x+i, y+j, textColor);
        else {  
          ILI9341_FillRect(x+(i*size), y+(j*size), size, size, textColor);
        }
      } else if (bgColor != textColor) {
        if (size == 1) // default size
          ILI9341_DrawPixel(x+i, y+j, bgColor);
        else {  // big size
          ILI9341_FillRect(x+i*size, y+j*size, size, size, bgColor);
        }
      }
      line >>= 1;
    }
  }
}

void ILI9341_DrawChar(int16_t x, int16_t y, char c, int16_t textColor, int16_t bgColor, uint8_t size){
  uint8_t line; // horizontal row of pixels of character
  int32_t col, row, i, j;// loop indices
  if(((x + 5*size - 1) >= _width)  || // Clip right
     ((y + 8*size - 1) >= _height) || // Clip bottom
     ((x + 5*size - 1) < 0)        || // Clip left
     ((y + 8*size - 1) < 0)){         // Clip top
    return;
  }

  setAddrWindow(x, y, x+6*size-1, y+8*size-1);

  line = 0x01;        // print the top row first
  // print the rows, starting at the top
  for(row=0; row<8; row=row+1){
    for(i=0; i<size; i=i+1){
      // print the columns, starting on the left
      for(col=0; col<5; col=col+1){
        if(Font[(c*5)+col]&line){
          // bit is set in Font, print pixel(s) in text color
          for(j=0; j<size; j=j+1){
            pushColor(textColor);
          }
        } else{
          // bit is cleared in Font, print pixel(s) in background color
          for(j=0; j<size; j=j+1){
            pushColor(bgColor);
          }
        }
      }
      // print blank column(s) to the right of character
      for(j=0; j<size; j=j+1){
        pushColor(bgColor);
      }
    }
    line = line<<1;   // move up to the next row
  }
}
uint16_t ILI9341_Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((b & 0xF8) << 8) | ((g & 0xFC) << 3) | (r >> 3);
}

uint32_t ILI9341_DrawString(uint16_t x, uint16_t y, char *pt, int16_t textColor){
  uint32_t count = 0;
  if(y>15) return 0;
  while(*pt){
    ILI9341_DrawCharS(x*6, y*10, *pt, textColor, BLACK, 1);
    pt++;
    x = x+1;
    if(x>20) return count;  // number of characters printed
    count++;
  }
  return count;  // number of characters printed
}

int32_t Ymax,Ymin,X;        // X goes from 0 to 127
int32_t Yrange; //YrangeDiv2;


void ILI9341_PlotClear(int32_t ymin, int32_t ymax){
  ILI9341_FillRect(0, 32, 128, 128, ILI9341_Color565(228,228,228)); // light grey
  if(ymax>ymin){
    Ymax = ymax;
    Ymin = ymin;
    Yrange = ymax-ymin;
  } else{
    Ymax = ymin;
    Ymin = ymax;
    Yrange = ymax-ymin;
  }
  //YrangeDiv2 = Yrange/2;
  X = 0;
}
int TimeIndex;               
int32_t Ymax, Ymin, Yrange;  
uint16_t PlotBGColor; 

void ILI9341_SimplePlotPoint(int32_t y)
{
		int32_t j;
  if(y<Ymin) y=Ymin;
  if(y>Ymax) y=Ymax;
  j = 32+(127*(Ymax-y))/Yrange;
  if(j<32) j = 32;
  if(j>159) j = 159;
  ILI9341_DrawPixel(X,   j,  BLUE);
  ILI9341_DrawPixel(X+1, j,  BLUE);
  ILI9341_DrawPixel(X,   j+1,BLUE);
  ILI9341_DrawPixel(X+1, j+1,BLUE);
}

void ILI9341_PlotPoint(int32_t data1, uint16_t color1){
  data1 = ((data1 - Ymin)*100)/Yrange;
  if(data1 > 98){
    data1 = 98;
    color1 = RED;
  }
  if(data1 < 0){
    data1 = 0;
    color1 = RED;
  }
  ILI9341_DrawPixel(TimeIndex + 11, 116 - data1, color1);
  ILI9341_DrawPixel(TimeIndex + 11, 115 - data1, color1);
}
void ILI9341_PlotIncrement(void){
  TimeIndex = TimeIndex + 1;
  if(TimeIndex > 99){
    TimeIndex = 0;
  }
  ILI9341_DrawFastVLine(TimeIndex + 11, 17, 100, PlotBGColor);
}

int32_t lastj=0;

void ILI9341_PlotLine(int32_t y)
{
	int32_t i,j;
  if(y<Ymin) y=Ymin;
  if(y>Ymax) y=Ymax;
  // X goes from 0 to 127
  // j goes from 159 to 32
  // y=Ymax maps to j=32
  // y=Ymin maps to j=159
  j = 32+(127*(Ymax-y))/Yrange;
  if(j < 32) j = 32;
  if(j > 159) j = 159;
  if(lastj < 32) lastj = j;
  if(lastj > 159) lastj = j;
  if(lastj < j){
    for(i = lastj+1; i<=j ; i++){
      ILI9341_DrawPixel(X,   i,   BLUE) ;
      ILI9341_DrawPixel(X+1, i,   BLUE) ;
    }
  }else if(lastj > j){
    for(i = j; i<lastj ; i++){
      ILI9341_DrawPixel(X,   i,   BLUE) ;
      ILI9341_DrawPixel(X+1, i,   BLUE) ;
    }
  }else{
    ILI9341_DrawPixel(X,   j,   BLUE) ;
    ILI9341_DrawPixel(X+1, j,   BLUE) ;
  }
  lastj = j;
}

void ILI9341_PlotBar(int32_t y){
int32_t j;
  if(y<Ymin) y=Ymin;
  if(y>Ymax) y=Ymax;

  j = 32+(127*(Ymax-y))/Yrange;
  ILI9341_DrawFastVLine(X, j, 159-j,BLACK);

}

void ILI9341_PlotNext(void){
  if(X==127){
    X = 0;
  } else{
    X++;
  }
}

void lcd9341_sendCmd(unsigned char cmd) {
   LCD_DC0; //Set DC low
   lcd9341_senddata(cmd);
}

void lcd9341_sendData(unsigned char data) {
   LCD_DC1;//Set DC HIGH
   lcd9341_senddata(data);
}
//Define Screen area
void lcd9341_at(unsigned int startX, unsigned int startY, unsigned int stopX, unsigned int stopY) {
	lcd9341_sendCmd(0x2A);
	LCD_DC1;
	lcd9341_senddata(0x00);
	lcd9341_senddata(startX);
	lcd9341_senddata(0x00);
	lcd9341_senddata(stopX);
	lcd9341_sendCmd(0x2B);
	LCD_DC1;
	lcd9341_senddata(0x00);
	lcd9341_senddata(startY);
	lcd9341_senddata(0x00); 
	lcd9341_senddata(stopY);
}




void ILI9341_Drawaxes(uint16_t axisColor, uint16_t bgColor, char *xLabel,char *yLabel1, uint16_t label1Color, char *yLabel2, uint16_t label2Color,int32_t ymax, int32_t ymin)
	{
  int i;

  Ymax = ymax;
  Ymin = ymin;
  Yrange = Ymax - Ymin;
  TimeIndex = 0;
  PlotBGColor = bgColor;
  ILI9341_FillRect(0, 0, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT, bgColor);
		//for (int i=0;i<320;i++){
//for (int j=0;j<240;j++){ILI9341_DrawPixel(i,j,bgColor);}}
  ILI9341_DrawFastHLine(10, 140, 101, axisColor);
  ILI9341_DrawFastVLine(10, 17, 124, axisColor);
  for(i=20; i<=110; i=i+10){
    ILI9341_DrawPixel(i, 141, axisColor);
  }
  for(i=17; i<120; i=i+10){
    ILI9341_DrawPixel(9, i, axisColor);
  }
  i = 50;
  while((*xLabel) && (i < 100)){
    ILI9341_DrawChar(i, 145, *xLabel, axisColor, bgColor, 1);
    i = i + 6;
    xLabel++;
  }
  if(*yLabel2){ // two labels
    i = 26;
    while((*yLabel2) && (i < 50)){
      ILI9341_DrawChar(0, i, *yLabel2, label2Color, bgColor, 1);
      i = i + 8;
      yLabel2++;
    }
    i = 82;
  }else{ // one label
    i = 42;
  }
  while((*yLabel1) && (i < 120)){
   ILI9341_DrawChar(0, i, *yLabel1, label1Color, bgColor, 1);
    i = i + 8;
    yLabel1++;
  }
}
void ILI9341_setRotation(uint8_t m) {
  unsigned rotation = m % 4; // can't be higher than 3
  switch (rotation) {
  case 0:
    m = (MADCTL_MX | MADCTL_BGR);
    _width = ILI9341_TFTWIDTH;
    _height = ILI9341_TFTHEIGHT;
    break;
  case 1:
    m = (MADCTL_MV | MADCTL_BGR);
    _width = ILI9341_TFTHEIGHT;
    _height = ILI9341_TFTWIDTH;
    break;
  case 2:
    m = (MADCTL_MY | MADCTL_BGR);
    _width = ILI9341_TFTWIDTH;
    _height = ILI9341_TFTHEIGHT;
    break;
  case 3:
    m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
    _width = ILI9341_TFTHEIGHT;
    _height = ILI9341_TFTWIDTH;
    break;
  }

  writecommand(ILI9341_MADCTL);
	lcd9341_sendData(m);
	
	
}

