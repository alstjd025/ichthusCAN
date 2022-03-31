/*
* CRC Calculation Library (DLL)
* Author: Yogyui (SeungHee-Lee)
*/
#define _CRC_H_

#include <vector>
#include <memory.h>
#include <iostream>

#define LUT_SIZE 256

class CRC8
{
public:
	CRC8(uint8_t polynomial, uint8_t init_value, bool reflect_input, bool reflect_output, uint8_t xor_output, bool use_lut);
	CRC8(bool use_lut = false) : CRC8(0x07, 0x00, false, false, 0x00, use_lut) {};

public:
	uint8_t calculate(const char* string);
	uint8_t calculate(const uint8_t* data_in, const int length);
	uint8_t calculate(std::vector<uint8_t> data);

private:
	uint8_t polynomial;
	uint8_t init_value;
	bool reflect_input;
	bool reflect_output;
	uint8_t xor_output;

	uint8_t reflect(uint8_t value);

	bool use_lut;
	uint8_t lookup_table[LUT_SIZE];
	void create_lookup_table();
};
