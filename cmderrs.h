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

#ifndef _CMDERRS_H
#define _CMDERRS_H
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------
// Reserved Command Error codes: (client to server) [0000 and E0-00 through FF-FF]
// ----------------------------------------------------------------------------

enum CommandErrors_enum {

// ----------------------------------------------------------------------------
// Command success

    COMMAND_OK                          = 0x0000,
    // Description:
    //      Command execution was successful (no error return to server)
    
    COMMAND_OK_ACK                      = 0x0001,
    // Description:
    //      Command execution was successful (Acknowledgement returned to server)

// ----------------------------------------------------------------------------
// Command argument errors
    
    COMMAND_ARGUMENTS                   = 0xF011,
    // Description:
    //      Insufficient/Invalid/Missing command arguments.  The command argument
    //      format was invalid, or missing critical values.

    COMMAND_INDEX                       = 0xF012,
    // Description:
    //      An index specified in the command arguments is invalid.  An 'index'
    //      specified in the command arguments was invalid or is out of range.

    COMMAND_STATUS                      = 0xF013,
    // Description:
    //      A status code specified in the command arguments is invalid.

    COMMAND_LENGTH                      = 0xF014,
    // Description:
    //      A length value specified in the command arguments is invalid.

    COMMAND_ID_NAME                     = 0xF015,
    // Description:
    //      An ID/Name/Filename specified in the command arguments is invalid.

    COMMAND_CHECKSUM                    = 0xF016,
    // Description:
    //      A checksum specified in the command arguments is invalid.

    COMMAND_OFFSET                      = 0xF017,
    // Description:
    //      An offset specified in the command arguments is invalid.

    COMMAND_OVERFLOW                    = 0xF021,
    // Description:
    //      More data was found in the payload than was expected.

// ----------------------------------------------------------------------------
// Command data value errors

    COMMAND_VALUE                       = 0xF100,
    // Description:
    //      A generic value specified in the command arguments is invalid.  
    //      The argument format is correct, but a specified value is invalid or
    //      is out of range.
    
    COMMAND_COUNT                       = 0xF102,
    // Description:
    //      A 'Count' value specified in the command arguments is invalid.
    
    COMMAND_TYPE                        = 0xF104,
    // Description:
    //      A 'Type' value specified in the command arguments is invalid.
    
    COMMAND_ACTION                      = 0xF106,
    // Description:
    //      An 'Action' value specified in the command arguments is invalid.

    COMMAND_ZONE_ID                     = 0xF111,
    // Description:
    //      A GeoZone ID specified in the command arguments is invalid.
    
    COMMAND_CORR_ID                     = 0xF116,
    // Description:
    //      A GeoCorridor ID specified in the command arguments is invalid.
    
    COMMAND_RADIUS                      = 0xF121,
    // Description:
    //      A 'Radius' value specified in the command arguments is invalid.
    
    COMMAND_LATLON                      = 0xF122,
    // Description:
    //      A 'Latitude/Longitude' value specified in the command arguments is invalid.

// ----------------------------------------------------------------------------
// Command data request errors
    
    COMMAND_UNAVAILABLE                 = 0xF201,
    // Description:
    //      The requested information is unavailable.  The command syntax and
    //      arguments are correct, but the requested resources are currently
    //      unavailable.

// ----------------------------------------------------------------------------
// Command execution errors
    
    COMMAND_EXECUTION                   = 0xF511,
    // Description:
    //      The client has determined that the execution of the command has failed
    //      (no specific reason)
    
    COMMAND_EXECUTION_MODE              = 0xF512,
    // Description:
    //      The client has determined that the execution of the command has failed
    //      due to an invalid mode.

    COMMAND_HARDWARE_FAILURE            = 0xF521,
    // Description:
    //      The client has determined that the execution of the command has failed
    //      due to hardware failure.
    
// ----------------------------------------------------------------------------
// Generic Command errors
// Create desired aliases for these to define specific custom errors

    COMMAND_ERROR_00                    = 0xFE00,
    COMMAND_ERROR_01                    = 0xFE01,
    COMMAND_ERROR_02                    = 0xFE02,
    COMMAND_ERROR_03                    = 0xFE03,
    COMMAND_ERROR_04                    = 0xFE04,
    COMMAND_ERROR_05                    = 0xFE05,
    COMMAND_ERROR_06                    = 0xFE06,
    COMMAND_ERROR_07                    = 0xFE07,

// ----------------------------------------------------------------------------
// Command execution errors

    COMMAND_FEATURE_NOT_SUPPORTED       = 0xFF01,
    // Description:
    //      A requested command feature is not supported.

};
typedef enum CommandErrors_enum CommandError_t;
    
// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
