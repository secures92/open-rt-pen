#ifndef USER_TIP_TABLE_H_
#define USER_TIP_TABLE_H_

#define TIP_CALIBRATION_POINTS	10
#define MAX_TIP_MICROVOLT		7000 // Appx. dT of 500 K

typedef struct TipCalibration_s
{
	uint16_t temperature;
	uint16_t microvolt;
} TipCalibration_t;


static TipCalibration_t TEMPERATURE_TABLE[TIP_CALIBRATION_POINTS] = {
	{0, 0},
	{25, 335},
	{75, 1006},
	{125, 1714},
	{175, 2342},
	{225, 3049},
	{275, 3753},
	{325, 4496},
	{375, 5243},
	{425, 5940}
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
