/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "params.h"
#include "my_string.h"

namespace Param
{

#define PARAM_ENTRY(category, name, unit, min, max, def, id) { category, #name, unit, min, max, def, id, TYPE_PARAM },
#define TESTP_ENTRY(category, name, unit, min, max, def, id) { category, #name, unit, min, max, def, id, TYPE_TESTPARAM },
#define VALUE_ENTRY(name, unit, id) { 0, #name, unit, 0, 0, 0, id, TYPE_SPOTVALUE },
static const Attributes attribs[] =
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef TESTP_ENTRY
#undef VALUE_ENTRY

#define PARAM_ENTRY(category, name, unit, min, max, def, id) def,
#define TESTP_ENTRY(category, name, unit, min, max, def, id) def,
#define VALUE_ENTRY(name, unit, id) 0.0f,
static float values[] =
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef TESTP_ENTRY
#undef VALUE_ENTRY

#define PARAM_ENTRY(category, name, unit, min, max, def, id) FLAG_NONE,
#define TESTP_ENTRY(category, name, unit, min, max, def, id) FLAG_NONE,
#define VALUE_ENTRY(name, unit, id) FLAG_NONE,
static uint8_t flags[] =
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef TESTP_ENTRY
#undef VALUE_ENTRY

//Duplicate ID check
#define PARAM_ENTRY(category, name, unit, min, max, def, id) ITEM_##id,
#define TESTP_ENTRY(category, name, unit, min, max, def, id) ITEM_##id,
#define VALUE_ENTRY(name, unit, id) ITEM_##id,
enum _dupes
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef TESTP_ENTRY
#undef VALUE_ENTRY


/**
* Set a parameter (accepts 5-bit fixed-point for CAN SDO compatibility)
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter (5-bit fixed-point format)
* @return 0 if set ok, -1 if ParamVal outside of allowed range
*/
int Set(PARAM_NUM ParamNum, s32fp ParamVal)
{
    char res = -1;
    
    // Convert from 5-bit fixed-point to float
    float floatVal = FP_TOFLOAT(ParamVal);

    if (floatVal >= attribs[ParamNum].min && floatVal <= attribs[ParamNum].max)
    {
        values[ParamNum] = floatVal;
        Change(ParamNum);
        res = 0;
    }
    return res;
}

/**
* Get a parameters fixed point value (returns 5-bit fixed-point for CAN SDO)
*
* @param[in] ParamNum Parameter index
* @return Parameters value in 5-bit fixed-point format
*/
s32fp Get(PARAM_NUM ParamNum)
{
    // Convert from float to 5-bit fixed-point for SDO protocol
    return FP_FROMFLT(values[ParamNum]);
}

/**
* Get a parameters integer value
*
* @param[in] ParamNum Parameter index
* @return Parameters value
*/
int GetInt(PARAM_NUM ParamNum)
{
    return (int)values[ParamNum];
}

/**
* Get a parameters float value
*
* @param[in] ParamNum Parameter index
* @return Parameters value
*/
float GetFloat(PARAM_NUM ParamNum)
{
    return values[ParamNum];
}

/**
* Get a parameters boolean value, 1.00=True
*
* @param[in] ParamNum Parameter index
* @return Parameters value
*/
bool GetBool(PARAM_NUM ParamNum)
{
    return (int)values[ParamNum] == 1;
}

/**
* Set a parameters digit value
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter
*/
void SetInt(PARAM_NUM ParamNum, int ParamVal)
{
   values[ParamNum] = (float)ParamVal;
}

/**
* Set a parameters fixed point value without range check and callback
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter (5-bit fixed-point format)
*/
void SetFixed(PARAM_NUM ParamNum, s32fp ParamVal)
{
   values[ParamNum] = FP_TOFLOAT(ParamVal);
}

/**
* Set a parameters floating point value without range check and callback
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter
*/
void SetFloat(PARAM_NUM ParamNum, float ParamVal)
{
   values[ParamNum] = ParamVal;
}

/**
* Get the paramater index from a parameter name
*
* @param[in] name Parameters name
* @return Parameter index if found, PARAM_INVALID otherwise
*/
PARAM_NUM NumFromString(const char *name)
{
    PARAM_NUM paramNum = PARAM_INVALID;
    const Attributes *pCurAtr = attribs;

    for (int i = 0; i < PARAM_LAST; i++, pCurAtr++)
    {
         if (0 == my_strcmp(pCurAtr->name, name))
         {
             paramNum = (PARAM_NUM)i;
             break;
         }
    }
    return paramNum;
}

/**
* Get the paramater index from a parameters unique id
*
* @param[in] id Parameters unique id
* @return Parameter index if found, PARAM_INVALID otherwise
*/
PARAM_NUM NumFromId(uint32_t id)
{
    PARAM_NUM paramNum = PARAM_INVALID;
    const Attributes *pCurAtr = attribs;

    for (int i = 0; i < PARAM_LAST; i++, pCurAtr++)
    {
         if (pCurAtr->id == id)
         {
             paramNum = (PARAM_NUM)i;
             break;
         }
    }
    return paramNum;
}

/**
* Get the parameter attributes
*
* @param[in] ParamNum Parameter index
* @return Parameter attributes
*/
const Attributes *GetAttrib(PARAM_NUM ParamNum)
{
    return &attribs[ParamNum];
}

/** Load default values for all parameters */
void LoadDefaults()
{
   const Attributes *curAtr = attribs;

   for (int idx = 0; idx < PARAM_LAST; idx++, curAtr++)
   {
      if (curAtr->id > 0)
         values[idx] = curAtr->def;
   }
}

void SetFlagsRaw(PARAM_NUM param, uint8_t rawFlags)
{
   flags[param] = rawFlags;
}

void SetFlag(PARAM_NUM param, PARAM_FLAG flag)
{
   flags[param] |= (uint8_t)flag;
}

void ClearFlag(PARAM_NUM param, PARAM_FLAG flag)
{
   flags[param] &= (uint8_t)~flag;
}

PARAM_FLAG GetFlag(PARAM_NUM param)
{
   return (PARAM_FLAG)flags[param];
}

PARAM_TYPE GetType(PARAM_NUM param)
{
   return (PARAM_TYPE)attribs[param].type;
}

uint32_t GetIdSum()
{
#ifndef PARAM_ID_SUM_START_OFFSET
#define PARAM_ID_SUM_START_OFFSET 0
#endif // PARAM_ID_SUM_START_OFFSET
#define PARAM_ENTRY(category, name, unit, min, max, def, id) id +
#define TESTP_ENTRY(category, name, unit, min, max, def, id) id +
#define VALUE_ENTRY(name, unit, id) id +
   return PARAM_LIST PARAM_ID_SUM_START_OFFSET;
#undef PARAM_ENTRY
#undef TESTP_ENTRY
#undef VALUE_ENTRY
}

}
