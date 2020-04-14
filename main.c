#include "adc.h"
#include "GLCD_ILI9341.h"
#define YMAX 5000
#define YMIN 0

volatile int sensorValue;

ADC_HandleTypeDef hadc1;
void drawaxes(void);
void drawInfoBar(void);
void plotData(void);

int  main(){

	HAL_Init();
	ADCI_Init();
	ILI9341_Init();
	lcd9341_at(0,0,240,320);
	ILI9341_setRotation(1);
	drawaxes();
	drawInfoBar();//ACR_BYTE0_ADDRESS
//for (int i=0;i<320;i++){
//for (int j=0;j<240;j++){
//ILI9341_DrawPixel(i,j,WHITE);
//}

//}
	//drawaxes();
//drawInfoBar();
	__enable_irq();
	while(1){
	//drawInfoBar();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,1);
		sensorValue =  HAL_ADC_GetValue(&hadc1);
		plotData();
	}
}

void plotData(void)
{
	ILI9341_PlotPoint(sensorValue,GREEN);
	ILI9341_PlotIncrement();
	
	
}
void drawaxes(void){
    ILI9341_Drawaxes(AXISCOLOR, BGCOLOR, "Time", "ADC", LIGHTCOLOR, "", 0, YMAX, YMIN);
}

void drawInfoBar(void)
{
ILI9341_DrawString(1, 0, "CPU =", GREEN);
ILI9341_DrawString(7, 0, "75%", BLUE );
ILI9341_DrawString(11, 0, "Temp =", GREEN);
ILI9341_DrawString(18,0, "30", BLUE );



}

void SysTick_Handler(void)
{
	HAL_IncTick();

}

