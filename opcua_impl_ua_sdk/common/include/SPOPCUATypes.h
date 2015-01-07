#ifndef __SPOPCUATYPES_H__
#define __SPOPCUATYPES_H__

#include <string>

enum SPOPCUAValueTypes
{
	None,
	Int32,
	UInt32,
	Int64,
	UInt64,
	String,
	Boolean
};

class SPOPCUAValue
{

public:
	SPOPCUAValue();
	SPOPCUAValue(long value);
	SPOPCUAValue(unsigned long value);
	SPOPCUAValue(long long value);
	SPOPCUAValue(unsigned long long value);
	SPOPCUAValue(std::string value);
	SPOPCUAValue(bool value);
	SPOPCUAValue(std::string type, std::string value_str);
	~SPOPCUAValue();

	SPOPCUAValueTypes GetType();
	std::string GetTypeString();

	// value getters
	long GetInt32();
	unsigned long GetUInt32();
	long long GetInt64();
	unsigned long long GetUInt64();
	std::string GetString();
	bool GetBoolean();

	// value setters
	void SetInt32(long value);
	void SetUInt32(unsigned long value);
	void SetInt64(long long value);
	void SetUInt64(unsigned long long value);
	void SetString(std::string value);
	void SetBoolean(bool value);

	std::string ToString();

	static SPOPCUAValueTypes BuildTypeFromString(std::string type_str);

private:

	SPOPCUAValueTypes type;

	// value containers
	long value_int32;
	unsigned long value_uint32;
	long long value_int64;
	unsigned long long value_uint64;
	std::string value_string;
	bool value_boolean;
};

#endif