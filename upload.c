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
// Description:
//  File upload support
// ---
// Change History:
//  2006/07/13  Martin D. Flynn
//      Initial Release
//  2007/01/28  Martin D. Flynn
//      WindowsCE port
// ----------------------------------------------------------------------------

#if defined(ENABLE_UPLOAD)
#include "defaults.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
//#include <unistd.h>
//#include <fcntl.h>
//#include <errno.h>

#include "log.h"

#include "stdtypes.h"
#include "utctools.h"
#include "strtools.h"
#include "checksum.h"
#include "io.h"

#include "propman.h"
#include "protocol.h"
#include "upload.h"

// ----------------------------------------------------------------------------

static long     uploadStartTime = 0L;
static char     uploadFile[UPLOAD_MAX_FILENAME_SIZE + 1];
static UInt8    *uploadData = (UInt8*)0;
static Int32    uploadRcd  = 0L;
static Int32    uploadSize = 0L;
static Int32    uploadAddr = 0L;

#if defined(TARGET_GUMSTIX)
// map filename
#define UPLOAD_INSTALL_DIR      "/install"
#define UPLOAD_DMTP_DIR         "/dmtp"
#define UPLOAD_DMTPD            "dmtpd"
static char *uploadFileMap[][2] = {
    { UPLOAD_DMTPD,             UPLOAD_DMTP_DIR "/" UPLOAD_DMTPD        }, // "/dmtp/dmtpd"
    { UPLOAD_DMTPD ".gz",       UPLOAD_DMTP_DIR "/" UPLOAD_DMTPD ".gz"  }, // "/dmtp/dmtpd.gz"
    { "S60" UPLOAD_DMTPD,       "/etc/init.d/S60" UPLOAD_DMTPD          }, // "/etc/init.d/S60dmtpd"
    { UPLOAD_DMTPD "_upgrade",  UPLOAD_DMTPD "_upgrade"                 }, // "dmtpd_upgrade"
    { 0, 0 }
};
#else
// leave filename as-is
#define UPLOAD_INSTALL_DIR        "/tmp/install"
static char *uploadFileMap[][2] = { { 0, 0 } };
#endif

// ----------------------------------------------------------------------------

/* return true if upload is currently in progress */
utBool uploadIsActive()
{
    return (uploadStartTime > 0L)? utTrue : utFalse;
}

/* return true if upload expired, or not in progress */
utBool uploadIsExpired()
{
    if (uploadStartTime <= 0L) {
        return utFalse;
    } else
    if ((uploadStartTime + UPLOAD_TIMEOUT_SEC) > utcGetTimeSec()) {
        return utFalse;
    } else {
        return utTrue;
    }
}

// ----------------------------------------------------------------------------

/* cancel current upload session */
void uploadCancel()
{
    
    /* clear start time */
    uploadStartTime = 0L;
    
    /* clear filename */
    *uploadFile = 0;
    
    /* clear len/addr */
    uploadRcd  = 0L;
    uploadSize = 0L;
    uploadAddr = 0L;
    
    /* free existing upload data */
    if (uploadData) {
        free(uploadData);
        uploadData = (UInt8*)0;
    }
    
}

// ----------------------------------------------------------------------------

/* process upload record */
static utBool _uploadProcessRecord(int protoNdx, const UInt8 *rcd, int rcdLen)
{
    Buffer_t bsrc, *src = binBuffer(&bsrc, (UInt8*)rcd, rcdLen, BUFFER_SOURCE);
    // File Name
    //   0:1 - Record Type [0x01]
    //   1:3 - File size
    //   4:X - File name
    // File Data
    //   0:1 - Record Type [0x02]
    //   1:3 - Data offset
    //   4:X - Data
    // End of data
    //   0:1 - Record Type [0x03]
    //   1:3 - File size
    //   4:2 - Fletcher Checksum
    
    /* type */
    UInt32 rcdType = 0L;
    binBufScanf(src, "%1x", &rcdType);
    
    /* file offset/length */
    Int32 lenAddr = 0L;
    binBufScanf(src, "%3x", &lenAddr);
    
    /* check record type */
    if (rcdType == UPLOAD_TYPE_FILE) {
        // file specifier
        logINFO(LOGSRC,"Upload filename ...");
        UInt8 *data = BUFFER_DATA(src);
        int dataLen = BUFFER_DATA_LENGTH(src);
        
        /* reset any previous upload attempts */
        uploadCancel();

        /* prechecks */
        if (lenAddr <= 0L) {
            // file length too small
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_LENGTH);
            return utFalse;
        } else
        if (lenAddr > UPLOAD_MAX_FILE_SIZE) {
            // file length too large
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_LENGTH);
            return utFalse;
        } else
        if (dataLen <= 0) {
            // file name too small
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_FILE_NAME);
            return utFalse;
        } else
        if (dataLen > UPLOAD_MAX_FILENAME_SIZE) {
            // file name too large
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_FILE_NAME);
            return utFalse;
        }

        /* filename */
        char tempName[UPLOAD_MAX_FILENAME_SIZE + 1], *destName = 0;
        strncpy(tempName, (char*)data, dataLen); // local copy
        tempName[dataLen] = 0;
        if (uploadFileMap && uploadFileMap[0][0]) {
            // map filename
            int d;
            for (d = 0; uploadFileMap[d][0]; d++) {
                if (strEquals(tempName, uploadFileMap[d][0])) {
                    destName = uploadFileMap[d][1];
                    break;
                }
            }
            if (!destName) {
                // Invalid filename
                protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_FILE_NAME);
                return utFalse;
            }
        } else {
            // leave filename as-is
            destName = tempName;
        }
        
        /* allocate space for uploaded file */
        uploadData = (UInt8*)malloc(lenAddr + 16); // [MALLOC] add some padding
        if (!uploadData) {
            logCRITICAL(LOGSRC,"OUT OF MEMORY!!");
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_LENGTH);
            return utFalse;
        }

        /* allocate space for upload */
        logINFO(LOGSRC,"Upload filename: %s ...", uploadFile);
        strcpy(uploadFile, destName);
        uploadRcd  = 0L;
        uploadSize = lenAddr;
        uploadStartTime = utcGetTimeSec();

        /* ok, so far */
        // should send ACK?
        //protocolQueueDiagnostic(protoNdx,"%2x%1x", (UInt32)DIAG_UPLOAD_ACK, (UInt32)rcdType);
        return utTrue;
        
    } else 
    if (rcdType == UPLOAD_TYPE_DATA) {
        // data
        //logINFO(LOGSRC,"Upload data ...");
        UInt8 *data = BUFFER_DATA(src);
        int dataLen = BUFFER_DATA_LENGTH(src);
        
        /* Simply ignore this packet if we are not uploading */
        // This allows the client to absorb these packets without causing possibly hundreds of
        // errors sent back to the server if a upload error has occurred.
        if (!uploadData || !(*uploadFile)) {
            // No file specification
            uploadCancel();
            return utTrue;
        }

        /* prechecks */
        if (lenAddr < uploadAddr) {
            // Data overlap
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_OFFSET_OVERLAP);
            return utFalse;
        } else
        if (lenAddr > uploadAddr) {
            // Data gap
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_OFFSET_GAP);
            return utFalse;
        } else
        if ((uploadAddr + BUFFER_DATA_LENGTH(src)) > uploadSize) {
            // data overflow
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_OFFSET_OVERFLOW);
            return utFalse;
        }
        
        /* cache */
        memcpy(uploadData + uploadAddr, data, dataLen);
        uploadAddr += dataLen;
        
        /* count this record */
        uploadRcd++;
        
        /* ok, so far */
        return utTrue;
       
    } else 
    if (rcdType == UPLOAD_TYPE_END) {
        // end of data
        logINFO(LOGSRC,"Upload EOD ...");
        
        /* Simply ignore this packet if we are not uploading */
        // This allows the client to absorb this packet without causeing possibly hundreds of
        // errors sent back to the server if a upload error has occurred.
        if (!uploadData || !(*uploadFile)) {
            // No file specification
            uploadCancel();
            return utTrue;
        }

        /* prechecks */
        if (lenAddr != uploadSize) {
            // invalid file length
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_LENGTH);
            return utFalse;
        } else
        if (uploadAddr != uploadSize) {
            // data hole
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_OFFSET_GAP);
            return utFalse;
        }
        
        /* check checksum */
        UInt32 cksumC0 = 0L, cksumC1 = 0L;
        binBufScanf(src, "%1x%1x", &cksumC0, &cksumC1);
        if ((cksumC0 != 0L) || (cksumC1 != 0L)) {
            ChecksumFletcher_t fcsTest = { { (UInt8)cksumC0, (UInt8)cksumC1 } }, fcsCalc;
            _cksumResetFletcher(&fcsCalc);
            _cksumCalcFletcher(&fcsCalc, uploadData, uploadSize);
            if (!_cksumEqualsFletcher(&fcsCalc, &fcsTest)) {
                // invalid checksum
                protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_CHECKSUM);
                return utFalse;
            }
        }
        
        /* get install file path */
        // eg. Install into "/install/<uploadFile>"
        char filepath[UPLOAD_MAX_FILENAME_SIZE * 2], *fp = filepath;
        sprintf(fp, UPLOAD_INSTALL_DIR); fp += strlen(fp);
        if (!strStartsWith(uploadFile,DIR_SEP)) {
            // append '/'
            sprintf(fp, "%s", DIR_SEP); fp += strlen(fp);
        }
        sprintf(fp, "%s", uploadFile);
        
        /* write file */
        logINFO(LOGSRC,"Upload writing file: %s ...", filepath);
#if defined(TARGET_WINCE)
        // for now, all directories must pre-exist
#else
        ioMakeDirs(filepath, utTrue); // make dirs
#endif
        long fileSize = ioWriteFile(filepath, uploadData, uploadSize);
        if (fileSize != uploadSize) {
            // write error
            protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_SAVE);
            return utFalse;
        }
        
        /* success */
        uploadCancel();
        // should send ACK?
        //protocolQueueDiagnostic(protoNdx,"%2x%1x", (UInt32)DIAG_UPLOAD_ACK, (UInt32)rcdType);
        return utTrue;
        
    } else {
        
        protocolQueueError(protoNdx,"%2x", (UInt32)ERROR_UPLOAD_TYPE);
        return utFalse;
        
    }

}

/* process upload record */
utBool uploadProcessRecord(int protoNdx, const UInt8 *rcd, int rcdLen)
{
    utBool ok = _uploadProcessRecord(protoNdx, rcd, rcdLen);
    if (!ok) {
        uploadCancel();
    }
    return ok;
}

// ----------------------------------------------------------------------------
#endif // defined(ENABLE_UPLOAD)
