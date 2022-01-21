#ifndef __ADC_PARAMETERS_HPP__
#define __ADC_PARAMETERS_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#ifdef __cplusplus
}
#endif

class measurement_temperature_c {

private:

public:

	double analog_value;

	double temperature_thermistor;
	double temperature_tmp36;

	double filtered_temperatur_thermistor;
	double filtered_temperature_tmp36;

	float v_supply;
	float r_10k_voltage_divider;
	float b_param_for_thermistor;
	float thermistor_reference_temperature;
	float thermistor_voltage_out;
	float thermistor_current_resistance;
	float thermistor_reference_resistance;


	uint16_t adc_resolotion_u16;

	measurement_temperature_c() {

		this->analog_value = 0;
		this->temperature_thermistor = 0;
		this->temperature_tmp36 = 0;

		this->filtered_temperatur_thermistor = 0;
		this->filtered_temperature_tmp36 = 0;

		this->v_supply = 3.3;
		this->r_10k_voltage_divider = 10000;
		this->b_param_for_thermistor = 3450;
		this->thermistor_reference_temperature = 298.15;
		this->thermistor_voltage_out = 0;
		this->thermistor_current_resistance = 0;
		this->thermistor_reference_resistance = 10000;

		this->adc_resolotion_u16 = 4096;

	}

	void read_analog_value_via_thermistor(ADC_HandleTypeDef *hadc,
			uint32_t channel) {

		ADC_ChannelConfTypeDef sConfig = { 0 };
		uint32_t adc_sample_period_u32 = ADC_SAMPLETIME_480CYCLES;
		sConfig.Channel = channel;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = adc_sample_period_u32;

		if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
			Error_Handler();
		}

		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
		this->analog_value = HAL_ADC_GetValue(hadc);
		HAL_ADC_Stop(hadc);

		this->thermistor_voltage_out = (this->v_supply
				* (float) (this->analog_value / this->adc_resolotion_u16));
		this->thermistor_current_resistance = (this->v_supply
				- this->thermistor_voltage_out)
				/ (this->thermistor_voltage_out / this->r_10k_voltage_divider);

		this->temperature_thermistor = ((this->thermistor_reference_temperature * this->b_param_for_thermistor)
				 / (this->thermistor_reference_temperature * log(this->thermistor_current_resistance / this->thermistor_reference_resistance)
				 + this->b_param_for_thermistor)) - 273.15;


	}

	void read_temp_via_tem36(ADC_HandleTypeDef *hadc, uint32_t channel) {

		ADC_ChannelConfTypeDef sConfig = { 0 };
		uint32_t adc_sample_period_u32 = ADC_SAMPLETIME_480CYCLES;
		sConfig.Channel = channel;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = adc_sample_period_u32;

		if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
			Error_Handler();
		}

		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
		this->analog_value = HAL_ADC_GetValue(hadc);
		HAL_ADC_Stop(hadc);

		this->temperature_tmp36 = (((double) this->analog_value) * 3.3
				/ pow(2, 12) - 0.5) * 100.0;
	}

};

typedef measurement_temperature_c *measurement_temperature_c_ptr;

#endif
