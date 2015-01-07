
#include <sstream>
#include "SPOPCUATypes.h"

///////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------
// AUX INLINE TYPE CONVERTERS
inline const char* const BoolToString(bool b)
{
  return b ? "true" : "false";
}
//
inline const long StringToInt32(std::string str)
{
	return std::stol(str.c_str());
}
//
inline const unsigned long StringToUInt32(std::string str)
{
	return std::atol(str.c_str());
}
//
inline const long long StringToInt64(std::string str)
{
	return std::stoll(str.c_str());
}

//
inline const unsigned long long StringToUInt64(std::string str)
{
	return std::atoll(str.c_str());
}
//
inline const bool StringToBool(std::string str)
{
	return (str == "true") ? true : false;
}
//---------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////
SPOPCUAValue::SPOPCUAValue()
{
	type = None;
}

///////////////////////////////////////////////////////////////////////
SPOPCUAValue::SPOPCUAValue(long value)
{
	type = Int32;
	value_int32 = value;
}

///////////////////////////////////////////////////////////////////////
SPOPCUAValue::SPOPCUAValue(unsigned long value)
{
	type = UInt32;
	value_uint32 = value;
}

///////////////////////////////////////////////////////////////////////
SPOPCUAValue::SPOPCUAValue(long long value)
{
	type = Int64;
	value_int64 = value;
}

///////////////////////////////////////////////////////////////////////
SPOPCUAValue::SPOPCUAValue(unsigned long long value)
{
	type = UInt64;
	value_uint64 = value;
}

///////////////////////////////////////////////////////////////////////
SPOPCUAValue::SPOPCUAValue(std::string value)
{
	type = String;
	value_string = value;
}

///////////////////////////////////////////////////////////////////////
SPOPCUAValue::SPOPCUAValue(bool value)
{
	type = Boolean;
	value_boolean = value;
}



///////////////////////////////////////////////////////////////////////
SPOPCUAValue::SPOPCUAValue(std::string type_str, std::string value_str)
{

	type = SPOPCUAValue::BuildTypeFromString(type_str);

	switch (type) {
		case None: break;
		case Int32:
			value_int32 = StringToInt32(value_str);
			break;
		case UInt32:
			value_uint32 = StringToUInt32(value_str);
			break;
		case Int64:
			value_int64 = StringToInt64(value_str);
			break;
		case UInt64:
			value_uint64 = StringToUInt64(value_str);
			break;
		case String:
			value_string = value_str;
			break;
		case Boolean:
			value_boolean = StringToBool(value_str);
			break;
	}
}

///////////////////////////////////////////////////////////////////////
SPOPCUAValue::~SPOPCUAValue()
{

}

///////////////////////////////////////////////////////////////////////
SPOPCUAValueTypes SPOPCUAValue::GetType()
{
	return type;
}

///////////////////////////////////////////////////////////////////////
std::string SPOPCUAValue::GetTypeString()
{
	switch (type) {
		case None: return "none";
		case Int32: return "int32";
		case UInt32: return "uint32";
		case Int64: return "int64";
		case UInt64: return "uint64";
		case String: return "string";
		case Boolean: return "boolean";
	}
}

///////////////////////////////////////////////////////////////////////
long SPOPCUAValue::GetInt32()
{
	return value_int32;
}

///////////////////////////////////////////////////////////////////////
unsigned long SPOPCUAValue::GetUInt32()
{
	return value_uint32;
}

///////////////////////////////////////////////////////////////////////
long long SPOPCUAValue::GetInt64()
{
	return value_int64;
}

///////////////////////////////////////////////////////////////////////
unsigned long long SPOPCUAValue::GetUInt64()
{
	return value_uint64;
}

///////////////////////////////////////////////////////////////////////
std::string SPOPCUAValue::GetString()
{
	return value_string;
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAValue::GetBoolean()
{
	return value_boolean;
}

///////////////////////////////////////////////////////////////////////
void SPOPCUAValue::SetInt32(long value)
{
	type = Int32;
	value_int32 = value;
}

///////////////////////////////////////////////////////////////////////
void SPOPCUAValue::SetUInt32(unsigned long value)
{
	type = UInt32;
	value_uint32 = value;
}

///////////////////////////////////////////////////////////////////////
void SPOPCUAValue::SetInt64(long long value)
{
	type = Int64;
	value_int64 = value;
}

///////////////////////////////////////////////////////////////////////
void SPOPCUAValue::SetUInt64(unsigned long long value)
{
	type = UInt64;
	value_uint64 = value;
}

///////////////////////////////////////////////////////////////////////
void SPOPCUAValue::SetString(std::string value)
{
	type = String;
	value_string = value;
}

///////////////////////////////////////////////////////////////////////
void SPOPCUAValue::SetBoolean(bool value)
{
	type = Boolean;
	value_boolean = value;
}

///////////////////////////////////////////////////////////////////////
SPOPCUAValueTypes SPOPCUAValue::BuildTypeFromString(std::string type_str)
{
	if( type_str == "int32" )
		return Int32;

	if( type_str == "uint32" )
		return UInt32;

	if( type_str == "int64" )
		return Int64;

	if( type_str == "uint64" )
		return UInt64;

	if( type_str == "string" )
		return String;

	if( type_str == "bool" )
		return Boolean;

	return None;
}

///////////////////////////////////////////////////////////////////////
std::string SPOPCUAValue::ToString()
{
	switch (type) {
		case None: return std::string("undefined");
		case Int32: return std::to_string( value_int32 );
		case UInt32: return std::to_string( value_uint32 );
		case Int64: return std::to_string( value_int64 );
		case UInt64: return std::to_string( value_uint64 );
		case String: return value_string;
		case Boolean: return BoolToString( value_boolean );
	}
	return std::string("undefined");
}