// ----------------------------------------------------------------------------
// Copyright 2006-2007, Martin D. Flynn
// All rights reserved
// ----------------------------------------------------------------------------
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
// http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ----------------------------------------------------------------------------

#ifndef _PROPMAN_H
#define _PROPMAN_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include "defaults.h"

#include "stdtypes.h"
#include "gpstools.h"

#include "props.h"
#include "cerrors.h"
#include "cmderrs.h"

#include "packet.h"
// ----------------------------------------------------------------------------

#define PROP_REFRESH_GET            0x1     // before get value
#define PROP_REFRESH_SET            0x2     // after set value
typedef UInt16 PropertyRefresh_t;

// ----------------------------------------------------------------------------
// Property manager errors

#define PROP_ERROR_OK               0x00000000L
#define PROP_ERROR_INVALID_KEY      0x01000000L
#define PROP_ERROR_INVALID_TYPE     0x02000000L
#define PROP_ERROR_INVALID_LENGTH   0x03000000L

#define PROP_ERROR_READ_ONLY        0x11000000L
#define PROP_ERROR_WRITE_ONLY       0x12000000L

#define PROP_ERROR_COMMAND_INVALID  0x22000000L
#define PROP_ERROR_COMMAND_ERROR    0x23000000L

#define PROP_ERROR_ARG_MASK         0x0000FFFFL
#define PROP_ERROR_ARG(E)           ((E) & PROP_ERROR_ARG_MASK)

#define PROP_ERROR_CODE_MASK        0xFF000000L
#define PROP_ERROR_CODE(E)          ((E) & PROP_ERROR_CODE_MASK)

#define PROP_ERROR(E,A)             ((E) | ((A) & PROP_ERROR_ARG_MASK))
#define PROP_ERROR_OK_LENGTH(E)     (PROP_ERROR_CODE(E)? -1 : (int)PROP_ERROR_ARG(E))

typedef UInt32 PropertyError_t;

// ----------------------------------------------------------------------------

#define KVA_SAVE                    0x8000  // save to auxillary storage
#define KVA_HIDDEN                  0x4000  // hidden
#define KVA_READONLY                0x2000  // read only
#define KVA_WRITEONLY               0x1000  // write only (ie. command)
#define KVA_REFRESH                 0x0800  // needs refresh before display (not implemented)
#define KVA_CHANGED                 0x0001  // value changed
#define KVA_NONDEFAULT              0x0002  // value is not default

#define KVA_IS_SAVE(A)              (((A) & KVA_SAVE) != 0)
#define KVA_IS_READONLY(A)          (((A) & KVA_READONLY) != 0)
#define KVA_IS_WRITEONLY(A)         (((A) & KVA_WRITEONLY) != 0)
#define KVA_IS_HIDDEN(A)            (((A) & KVA_HIDDEN) != 0)
#define KVA_IS_REFRESH(A)           (((A) & KVA_REFRESH) != 0)
#define KVA_IS_NONDEFAULT(A)        (((A) & KVA_NONDEFAULT) != 0)
#define KVA_IS_CHANGED(A)           (((A) & KVA_CHANGED) != 0)

#define KVT_TYPE_MASK               0x0F80              // type mask
#define KVT_BOOLEAN                 KVT_UINT8           // utBool (0..1)
#define KVT_UINT8                   0x0200              // UInt8  (0..255)
#define KVT_UINT16                  0x0300              // UInt16 (0..65535)
#define KVT_UINT24                  0x0400              // UInt16 (0..372735) // not currently used
#define KVT_UINT32                  0x0500              // UInt32 (0..4294967295)
#define KVT_BINARY                  0x0800              // generic binary
#define KVT_STRING                  0x0900              // generic string
#define KVT_GPS                     0x0A00              // GPSOdometer_t
#define KVT_POINTER                 0x0C00				// pointer
#define KVT_COMMAND                 0x0D00				// command
#define KVT_TYPE(T)                 ((T) & KVT_TYPE_MASK)

#define KVT_DEC_MASK                0x000F           // decimal point mask (max 15 decimal places)
#define KVT_UINT_MASK               0x700            // everything about integer type
#define KVT_DEC(X)                  ((X) & KVT_DEC_MASK)

#define KVT_ATTR_MASK               0xF000              // attribute mask
#define KVT_SIGNED                  0x1000              // signed (UIntXX types only)
#define KVT_HEX                     0x2000              // hex (UIntXX types only)

#define KVT_INT16                   (KVT_UINT16 | KVT_SIGNED)
#define KVT_INT32                   (KVT_UINT32 | KVT_SIGNED)
#define KVT_HEX16                   (KVT_UINT16 | KVT_HEX)
#define KVT_HEX32                   (KVT_UINT32 | KVT_HEX)

#define KVT_IS_UINT8(T)             ((KVT_TYPE(T) == KVT_UINT8)? utTrue : utFalse)
#define KVT_IS_UINT16(T)            ((KVT_TYPE(T) == KVT_UINT16)? utTrue : utFalse)
#define KVT_IS_UINT24(T)            ((KVT_TYPE(T) == KVT_UINT24)? utTrue : utFalse)
#define KVT_IS_UINT32(T)            ((KVT_TYPE(T) == KVT_UINT32)? utTrue : utFalse)
#define KVT_IS_UINT(T)    (((KVT_TYPE(T)  & ~ KVT_UINT_MASK) == 0)? utTrue : utFalse)
#define KVT_UINT_SIZE(T)  (((T & KVT_UINT_MASK) >> 8) - 1)

#define KVT_IS_STRING(T)            ((KVT_TYPE(T) == KVT_STRING)? utTrue : utFalse)
#define KVT_IS_GPS(T)               ((KVT_TYPE(T) == KVT_GPS)? utTrue : utFalse)
#define KVT_IS_SIGNED(T)            ((((T) & KVT_SIGNED) != 0)? utTrue : utFalse)
#define KVT_IS_HEX(T)               ((((T) & KVT_HEX) != 0)? utTrue : utFalse)
#define KVT_IS_POINTER(T)           (KVT_TYPE(T) == KVT_POINTER)? utTrue : utFalse

#define PROP_REFRESH_TAG	1
#define PROP_REFRESH_TEMPERATURE	4
#define PROP_REFRESH_GPS	8

#define PROP_SAVE_0			1
#define PROP_SAVE_1			2
extern UInt32 property_refresh_flag;
typedef UInt16      Key_t;
typedef UInt16      KeyType_t;
typedef UInt16      KeyAttr_t;

typedef union {
	UInt8 	b[MAX_ID_SIZE];// small amounts of data
	UInt32 	i[MAX_ID_SIZE / 4];// boolean, [U][Int8|Int16|Int32]
	GPSOdometer_t	gps;                // gps (time, lat, lon)
	CommandError_t	(*cmd)(int pi, UInt16 key, const UInt8 *data, int dataLen);
	struct {
		UInt32	l;		// length of the data
		void	*b;		// pointer to real data
	} ptr;
} KeyData_t;

typedef struct {
    // The order of these elements must remain as-is (per static initialization)
    Key_t           key;                // [ 2] property key
    char            *name;              // [ 4] name
    KeyType_t       type;               // [ 2] type (this defines the size of maxNdx/lenNdx)
    KeyAttr_t       attr;               // [ 2] attributes
    UInt16          maxNdx;             // [ 2] maiximum index
    char            *dftInit;           // [ 4] ASCIIZ string initializer
    // These elements may be ordered as needed
    UInt16          lenNdx;             // [ 2] KeyData item index
    UInt16          dataSize;           // [ 2] number of bytes used in 'data'
    KeyData_t       data;               // [32]
} KeyValue_t;                           // [56]

// ----------------------------------------------------------------------------

// Name: 
//   Property table start-up initialization.
// Description:
//   This function must be called at startup to initialize the default property list
//   May be called at any time to reset the properties to their default values
// Return
//   void
void propInitialize(utBool forceReset);

// Name:
//   Initialize property key from string (char*).
// Description:
//   This function may be used to initialize properties to custom values following
//   the call to the initial 'propInitialize' function.
//   This function resets the 'modified' flag, so it should not be used for general
//   setting of values.
// Return:
//   utTrue if the key is valid, false otherwise.
utBool propInitFromString(Key_t key, const char *s);

// Name:
//   Print value of property to supplied string (char*)
// Description:
//   This function may be used to convert the KeyValue to a string which can later
//   be parsed by 'propInitFromString'.  This is useful for writing the contents
//   of the KeyValue_t property table to auxillary storage for later reloading.
// Return:
//   utTrue if the key is valid, false otherwise.
utBool propPrintToString(Key_t key, char *buf, int bufLen);

// Name:
//   Override 'ReadOnly' attribute on property
// Description:
//   This function allows overriding the read-only attribute on the specified
//   property key.
// Return:
//   utTrue if the key is valid, false otherwise.
utBool propSetReadOnly(Key_t key, utBool readOnly);

// Name:
//   Override 'Save' attribute on property
// Description:
//   This function allows overriding the save attribute on the specified
//   property key.
// Return:
//   utTrue if the key is valid, false otherwise.
utBool propSetSave(Key_t key, utBool save);

// Name:
//   Set alternate buffer for storage of property values
// Description:
//   This function allows specifying an alternate buffer to use for the storage
//   of data for the specified key.  
//   NOTE: This function does not verify the proper alignment of the specifiied buffer 
//   for type of data which will be stored.  The caller must insure that the buffer
void propSetNotifyFtn(PropertyRefresh_t mode, void (*ftn)(PropertyRefresh_t,Key_t,UInt8*,int));

// Name:
//   Set a callback function for execution of a command
// Description:
//   This function specifies the call-back function for the various 'command' type
//   properties.  The specified function will be called when the corresponding
//   command property is 'set'.
// Return:
//   utTrue if the key is valid, false otherwise.
utBool propSetCommandFtn(Key_t key, CommandError_t (*cmd)(int pi, Key_t key, const UInt8 *data, int dataLen));

// Name:
//   Return a pointer to the KeyValue_t entry
// Description:
//   These functions return a pointer to the KeyValue_t entry for the specified key
//   (propGetKeyValueEntry), or for the specified index (propGetKeyValueAt).
KeyValue_t *propGetKeyValueEntry(Key_t key);
KeyValue_t *propGetKeyValueEntryByName(const char *keyName);
// Name:
//   Get maximum number of indexed fields in property
// Description:
//   Returns the maximum number of indexed fields available in the property.
// Return:
//   The maximum number of indexed items in the specified property, or -1 if the
//   property key was not found.
int propSetValueCmd(int protoNdx, Packet_t *pkt, const UInt8 *data, UInt32 dataLen, UInt32 timestamp);
int propSetValueCmdTCP(int protoNdx, const UInt8 *data, UInt32 dataLen, UInt32 *error_code);
int propGetPropertyPacket(int protoNdx, Packet_t *pkt, UInt8 * args, UInt32 dataLen, UInt32 timestamp);
int propGetPropertyPacketTCP(int protoNdx, Packet_t *pkt, UInt8 * args, UInt32 dataLen);
PropertyError_t propGetValue(Key_t key, UInt8 *data, int dataLen);
// Name:
//   Get/Set integer value
// Description:
//   These functions are for getting/setting 32-bit integers from within code
//   These functions DO NOT obey the KVA_READONLY/KVA_WRITEONLY attributes
// Return:
//   propGetUInt32AtIndex & propGetUInt32 return the integer value of the property,
//   or the default 'dft' value if the key is not found, or the property value is
//   empty, or if an error occurs.  propSetUInt32AtIndex, propAddUInt32AtIndex,
//   propSetUInt32, and propAddUInt32 return utTrue if the operation was successful,
//   and utFalse otherwise.  propAddUInt32AtIndex returns the updated value back
//   into location pointed to by 'val'.
UInt32 propGetUInt32AtIndex(Key_t key, int ndx, UInt32 dft);
utBool propSetUInt32AtIndex(Key_t key, int ndx, UInt32 val);
UInt32 propGetUInt32(Key_t key, UInt32 dft);
utBool propSetUInt32(Key_t key, UInt32 val);
utBool propAddUInt32(Key_t key, UInt32 val);

// Name:
//   Get/Set boolean value
// Description:
//   These functions are for getting/setting boolean values from within code.
//   These are essentially wrappers around calls retrieving integers with a 
//   case to 'utBool'.  
//   These functions DO NOT obey the KVA_READONLY/KVA_WRITEONLY attributes
// Return:
//   These functions return utTrue if the operation was successful, utFalse otherwise.
utBool propGetBoolean(Key_t key, utBool dft);
utBool propSetBoolean(Key_t key, utBool val);

// Name:
//   Get/Set double value
// Description:
//   These functions are for getting/setting double values from within code.
//   These are essentially wrappers around calls retrieving integers, with a 
//   fixed decimal position and a case to a double.  
//   These functions DO NOT obey the KVA_READONLY/KVA_WRITEONLY attributes.
// Return:
//   propGetDoubleAtIndex & propGetDouble return the double value if this key
//   is valid, and the default 'dft' value otherwise.  propSetDoubleAtIndex,
//   propAddDoubleAtIndex, propSetDouble, and propAddDouble return utTrue if
//   the operation was successful, and utFalse otherwise.  propAddDoubleAtIndex
//   returns the updated value back into location pointed to by 'val'.
double propGetDouble(Key_t key, double dft);

// Name:
//   Get/Set string value
// Description:
//   These functions are for getting/setting string values from within code.
//   These functions DO NOT obey the KVA_READONLY/KVA_WRITEONLY attributes.
// Return:
//   propGetString returns the property string value if the key is valid,
//   or the default 'dft' value otherwise.  propSetString returns utTrue if
//   the operation was successful, utFalse otherwise.
const char *propGetString(Key_t key, const char *dft);
utBool propSetString(Key_t key, const char *val);

// Name:
//   Get/Set binary value
// Description:
//   These functions are for getting/setting binary values from within code.
//   These functions DO NOT obey the KVA_READONLY/KVA_WRITEONLY attributes.
// Return:
//   propGetBinary returns the property binary value if the key is valid,
//   or the default 'dft' value otherwise.  propSetBinary returns utTrue if
//   the operation was successful, utFalse otherwise.
const UInt8 *propGetBinary(Key_t key, const UInt8 *dft, UInt16 *maxLen, UInt16 *dtaLen);
utBool propSetBinary(Key_t key, const UInt8 *val, UInt16 dtaLen);
utBool propSetUInt32_a(Key_t key, const UInt32 *value, int len);

// Name:
//   Get/Set GPS value
// Description:
//   These functions are for getting/setting GPSOdometer_t values from within code.
//   These functions DO NOT obey the KVA_READONLY/KVA_WRITEONLY attributes.
// Return:
//   propGetGPS returns the property GPS value if the key is valid,
//   or the default 'dft' value otherwise.  propSetGPS returns utTrue if
//   the operation was successful, utFalse otherwise.
const GPSOdometer_t *propGetGPS(Key_t key, const GPSOdometer_t *dft);
utBool propSetGPS(Key_t key, const GPSOdometer_t *gps);

// Name:
//   Return true if properties have been changed since last save.
// Description:
//   This function indicates whether the current property values should be saved.
// Return:
//   true if at least one property has changed since last save, false otherwise.
utBool propHasChanged();

// Name:
//   Clear all property changed flags
// Description:
//   This function resets all property attributes to a "non-changed" state
void propClearChanged();

// Name:
//   Save <key>=<value> property list
// Description:
//   This function is for saving the contents of the property list to a file.
// Return:
//   true if save was successful
utBool propSaveProperties(const char *propFile, UInt32 method);

// Name:
//   Load <key>=<value> list
// Description:
//   This function is for loading the contents of the property list from a file.
// Return:
//   void
utBool propLoadProperties(const char *propFile, UInt32 method);

// ----------------------------------------------------------------------------
void propPrintAll(void);
void propExportAll(void);
utBool propSetPointerToString(Key_t key, char *str);
char * propGetPointerToString(Key_t key);
void propInitPointerFromString(Key_t key, char *str);
// ----------------------------------------------------------------------------

const char *propGetAccountID();
const char *propGetDeviceID(int protoNdx);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
