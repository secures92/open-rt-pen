#ifndef USER_TIP_TABLE_H_
#define USER_TIP_TABLE_H_

#define TIP_CALIBRATION_POINTS	9
#define MAX_TIP_MICROVOLT		6700

typedef struct TipCalibration_s
{
	uint16_t temperature;
	uint16_t microvolt;
} TipCalibration_t;


static TipCalibration_t TEMPERATURE_TABLE[TIP_CALIBRATION_POINTS] = {
	{0, 0},
	{128, 1200},
	{178, 2268},
	{228, 3000},
	{278, 3770},
	{328, 4500},
	{378, 5250},
	{428, 5970},
	{478, 6700}
};

uint16_t TIP_InterpolateTemperature(uint16_t microvolt)
{
	uint8_t i;
	uint16_t temperature = 0;
	for(i = 0; i < TIP_CALIBRATION_POINTS-1; i++)
	{
		if(microvolt >= TEMPERATURE_TABLE[i].microvolt && microvolt < TEMPERATURE_TABLE[i+1].microvolt)
		{
			temperature = TEMPERATURE_TABLE[i].temperature + (microvolt - TEMPERATURE_TABLE[i].microvolt) * (TEMPERATURE_TABLE[i+1].temperature - TEMPERATURE_TABLE[i].temperature) / (TEMPERATURE_TABLE[i+1].microvolt - TEMPERATURE_TABLE[i].microvolt);
			break;
		}
	}
	return temperature;
}

#endif /* USER_TIP_TABLE_H_ */
