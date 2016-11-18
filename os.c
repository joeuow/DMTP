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
//  OS platform specific utilities
// Notes:
//  - Most of the utilities in this module were designed for the GumStix.  They
//    may need to be modified to fit other platforms.
// ---
// Change History:
//  2006/05/10  Martin D. Flynn
//     -Initial release
// ----------------------------------------------------------------------------

#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#if defined(TARGET_GUMSTIX)
#include <linux/reboot.h>
#include <sys/reboot.h>
#endif // TARGET_GUMSTIX

#include <unistd.h>
//#include <mntent.h>
#include <sys/vfs.h>
#include <sys/utsname.h>

#include "log.h"
#include "os.h"

#include "stdtypes.h"
#include "strtools.h"
#include "utctools.h"
#include "io.h"
#include "comport.h" // needed for definition of "RFCOMM0"

// ----------------------------------------------------------------------------

#define SETHOSTNAME_REWRITE_ETC_HOSTNAME
#define SETHOSTNAME_REWRITE_ETC_HOSTS
#define SETHOSTNAME_RESET_BLUETOOTH

// ----------------------------------------------------------------------------

/* return current host name */
const char *osGetHostname(char *host, int hostLen)
{
	FILE *host_file = fopen("/etc/hostname", "r");
	int i = 0;
    if (host) {
        *host = 0;
//        gethostname(host, hostLen);
	fread(host, sizeof(char), hostLen, host_file);
	fclose(host_file);
	while(*(host+i++) != '\n');
	*(host+i-1) = '\0';
    }
    return host;
}

/* set current host name */
utBool osSetHostname(const char *s)
{
    // -----------------------------------------------------------------------------------------
    // rewrite "/etc/hostname"
    //   "%s"
    // rewrite "/etc/hosts"
    //   "127.0.0.1   localhost.localdomain localhost gumstix %s"
    // -----------------------------------------------------------------------------------------
    
    /* get old hostname */
    char oldHostname[64];
    osGetHostname(oldHostname, sizeof(oldHostname));
    logDEBUG(LOGSRC,"Current hostname: %s", oldHostname);

    /* check for default hostname request */
    if (!s || !*s || strEquals(s,"?")) {
        s = osGetUniqueID();
        if (!*s) {
            s = DEFAULT_HOSTNAME;
        }
    }

    /* validate hostname */
    int h = 0;
    char hostname[32];
    if (!isalpha(*s)) {
        // first character must be alpha
        hostname[h++] = 'T';
    }
    for (; *s && (h < sizeof(hostname) - 1); s++) {
        if (isdigit(*s) || isalpha(*s)) {
            hostname[h++] = *s; // only alphanumeric
        }
    }
    hostname[h++] = 0; // terminate
    
    /* change hostname */
    if (!*hostname) {
        // no specified hostname
        return utFalse;
    } else
    if (strEquals(hostname, oldHostname)) {
        // this hostname already set
        logINFO(LOGSRC,"Hostname already set to %s", hostname);
        return utTrue;
    } else
#if defined(TARGET_GUMSTIX)
    if (sethostname(hostname, strlen(hostname)) == 0) {
        // hostname change was successful
        char buff[128];
#if defined(SETHOSTNAME_REWRITE_ETC_HOSTNAME)
        /* rewrite "/etc/hostname" */
        sprintf(buff, "%s\n", hostname);
        ioWriteFile("/etc/hostname", buff, strlen(buff));
#endif // SETHOSTNAME_REWRITE_ETC_HOSTNAME
#if defined(SETHOSTNAME_REWRITE_ETC_HOSTS)
        /*rewrite "/etc/hosts" */
        sprintf(buff, "127.0.0.1 localhost.localdomain localhost gumstix %s\n", hostname);
        ioWriteFile("/etc/hosts", buff, strlen(buff));
#endif // SETHOSTNAME_REWRITE_ETC_HOSTS
#if defined(SETHOSTNAME_RESET_BLUETOOTH)
        osSetBluetoothName(hostname);
#endif // SETHOSTNAME_RESET_BLUETOOTH
        logINFO(LOGSRC,"New hostname: %s", hostname);
        return utTrue;
    }
#else // TARGET_GUMSTIX
    {
        /* sethostname omitted from non-GumStix platforms */
        logINFO(LOGSRC,"'osSetHostname' not supported on this platform");
    }
#endif // TARGET_GUMSTIX
    
    return utFalse;
}

// ----------------------------------------------------------------------------

/* execute command in separate process */
utBool osExec(char *cmd)
{
    FILE *cmdf = popen(cmd, "r"); // this may still send output to the console
    if (cmdf) {
        pclose(cmdf);
        return utTrue;
    } else {
        return utFalse;
    }
}

// ----------------------------------------------------------------------------

utBool osReboot()
{
#if defined(ENABLE_REBOOT)

    /* sync and sleep a few seconds (sleep may not be necessary) */
    sync();
    threadSleepMS(1000L); // wait a second

    /* start the watchdog timer */
    char modp[128];
    sprintf(modp, "( /sbin/modprobe sa1100_wdt margin=30; echo 1 >/dev/watchdog ) > /dev/null 2>&1");
    if (!osExec(modp)) {
        logERROR(LOGSRC,"Unable to start WatchDog timer");
    }
    
    /* try reboot */
    // This has been known to hang occasionally, thus the above watchdog
    reboot(LINUX_REBOOT_CMD_RESTART);
    threadSleepMS(35000L); // <-- should never reach here

    return utFalse; // not supported if we reach here
    
#else

    logERROR(LOGSRC,"'osReboot' not supported on this platform");
    return utFalse; // not supported

#endif
}


// ----------------------------------------------------------------------------

#define RFCOMM_GETTY    "/etc/bluetooth/rfcomm/rfcomm-getty"
#define SERIAL_GETTY    "/sbin/getty" /* -L ttyS0 115200 vt100 */

void osStartConsole(int port)
{
#if defined(TARGET_GUMSTIX)
    // It is assumed that all threads have been stopped!
    if (port == RFCOMM0) {
        char *argv[] = { RFCOMM_GETTY, (char*)0 };
        execv(RFCOMM_GETTY, argv);
    } else {
        char *argv[] = { SERIAL_GETTY, "-L", "ttyS0", "115200", "vt100", (char*)0 };
        execv(SERIAL_GETTY, argv);
    }
    osReboot(); // <-- should never reach here
#else // TARGET_GUMSTIX
    logWARNING(LOGSRC,"'osStartConsole' not supported on this platform");
#endif // TARGET_GUMSTIX
}

// ----------------------------------------------------------------------------

#define BLUETOOTH_PREFIX    'M' // bluetooth mac address

static Int8     osHasBluetoothChk = 0;
static Int64    osBluetoothMAC    = -1LL;  // 48 bit

/* return true if this platform supports bluetooth */
utBool osHasBluetooth()
{
    // GumStix devices without Bluetooth are shipped with Bluetooth support,
    // so this test may not return an accurate representation of Bluetooth support.
    if (osHasBluetoothChk == 0) {
        if (ioIsFile("/etc/init.d/S30bluetooth") && ioIsFile("/usr/sbin/hciconfig")) {
            osHasBluetoothChk = 1;
        } else {
            osHasBluetoothChk = -1;
        }
    }
    return (osHasBluetoothChk > 0)? utTrue : utFalse;
}

/* return a serial number based on the bluetooth MAC address */
const char *osGetBluetoothMacID(char *ser)
{
    Int64 bt = osGetBluetoothMac();
    sprintf(ser, "%c%012llX", BLUETOOTH_PREFIX, bt);
    return ser;
}

/* get bluetooth MAC address */
Int64 osGetBluetoothMac()
{
    // WARNING: This will return a valid Bluetooth MAC address iff the Bluetooth system
    //          has already been started!!!
    // Obtain from "hciconfig"
    // # /usr/sbin/hciconfig hci0 name                                                           
    //   hci0: Type: UART                                                              
    //         BD Address: 00:80:37:21:05:50 ACL MTU: 672:8  SCO MTU: 64:0             
    //         Name: 'M008037210550'                                                   
    // Or,
    //   Can't get device info: No such device
    // #
    if (osBluetoothMAC <= 0LL) {
        // hciconfig hci0 name | grep "BD Address" | sed 's/^\([^:]*\)//;s/: //;s/ACL.*$//;s/://g' 
        osBluetoothMAC = 0LL;
        if (osHasBluetooth()) {
            FILE *hciFile = popen("/usr/sbin/hciconfig hci0 name", "r");
            if (hciFile) {
                UInt8 data[1024];
                long len = ioReadStream(hciFile, data, sizeof(data) - 1);
                pclose(hciFile);
                if (len > 0) {
                    data[len] = 0; // terminate
                    //logDEBUG(LOGSRC,"Read %d bytes from 'hciconfig' command", len);
                    char *d = (char *)data;
                    while (*d) {
                        while (*d && isspace(*d)) { d++; } // skip leading spaces
                        if ((*d == 'B') && strStartsWithIgnoreCase(d, "BD Address:")) {
                            d += 11; // skip "BD Address:"
                            while (*d && isspace(*d)) { d++; } // skip spaces
                            char macHex[20], *m = macHex;
                            while (*d && !isspace(*d) && ((m - macHex) < (sizeof(macHex) - 1))) {
                                if (isdigit(*d) || isalpha(*d)) {
                                    *m++ = *d;
                                }
                                d++;
                            }
                            *m = 0;
                            //logDEBUG(LOGSRC,"Parsing MAC address: %s", macHex);
                            osBluetoothMAC = strParseHex64(macHex, 0LL);
                            break;
                        }
                        while (*d && (*d != '\n') && (*d != '\r')) { d++; }
                    } // while (*d)
                } else {
                    logINFO(LOGSRC,"No data read from 'hciconfig' command");
                }
            } else {
                logINFO(LOGSRC,"Failed to execute 'hciconfig' command");
            }
        }
    }
    return osBluetoothMAC;
}

/* set published bluetooth name */
utBool osSetBluetoothName(const char *name)
{
    if (name && *name && osHasBluetooth()) {
        // NOTE: Bluetooth must be also be running for this to be effective
        char buff[512];
        sprintf(buff, "/usr/sbin/hciconfig hci0 name %s", name);
        if (osExec(buff)) {
            return utTrue;
        }
    }
    logERROR(LOGSRC,"Unable to reset Bluetooth name: %s", name);
    return utFalse;
}

// ----------------------------------------------------------------------------

#define SERIAL_FILE_NAME    "serial.num"
#define CPUINFO_PATH        "/proc/cpuinfo"
#define SERIAL_PREFIX       'S' // serial number (flash "Protection Register" factory value)

static UInt64   osSerialNum = 0xFFFFFFFFFFFFFFFFLL;  // 64 bit
static char     osSerialStr[32];

static char _serialID[32] = { 0 };
const char *osGetSerialNumberID()
{
    if (!*_serialID) {
        // This compresses the 64 bit serial number into 12 hex digits
        // so that the resulting bluetooth name isn't unecessarily long.
        UInt64 sn = osGetSerialNumber();
        UInt64 ac = sn   & 0x0000FFFFFFFFFFFFLL;
        ac ^= (sn >> 24) & 0x000000FF00000000LL;
        ac ^= (sn >>  8) & 0x0000FF0000000000LL;
        sprintf(_serialID, "%c%012llX", SERIAL_PREFIX, ac);
    }
    return _serialID;
}

UInt64 osGetSerialNumber()
{
    // TODO: need to change this to return a byte array
    // IE "const int osGetSerialNumber(UInt8 *serial, int serialLen)"
    if (osSerialNum == 0xFFFFFFFFFFFFFFFFLL) {
        osSerialNum = 0LL;
#if defined(TARGET_GUMSTIX)
        /* read serial from "/proc/cpuinfo" */
        // "Serial : 200007009341250D"
        UInt8 buf[1024];
        long len = ioReadFile(CPUINFO_PATH, buf, sizeof(buf) - 1);
        if (len > 0L) {
            buf[len] = 0;
            UInt8 *b = buf;
            while (*b) {
                while (*b && isspace(*b)) { b++; } // skip leading spaces
                if ((*b == 'S') && strStartsWithIgnoreCase(b, "Serial")) {
                    while (*b && (*b != ':')) { b++; } // find ':'
                    if (*b) {
                        b++;
                        while (*b && isspace(*b)) { b++; } // skip spaces
                    }
                    osSerialNum = strParseHex64(b, 0LL);
                    break;
                }
                while (*b && (*b != '\n') && (*b != '\r')) { b++; }
            }
        } else {
            logERROR(LOGSRC,"Unable to read file: %s", CPUINFO_PATH);
        }
#else
        // arbitrary default
      //osSerialNum = 0x2000090093418502LL;
        osSerialNum = 0x0000000000000000LL;
#endif
    }
    return osSerialNum;
}

const char *osGetUniqueID()
{
    return _osGetUniqueID(utTrue);
}

const char *_osGetUniqueID(utBool useBluetooth)
{
    // Thread safe if called at least once to initialize before any other
    // threads are started.
    if (!*osSerialStr) {
    
        /* read serial number from file */
        char serFile[256];
        sprintf(serFile, "%s/%s", CONFIG_DIR, SERIAL_FILE_NAME);
        long len = ioReadFile(serFile, osSerialStr, sizeof(osSerialStr) - 1);
        if (len > 0L) {
            osSerialStr[len] = 0; // make sure it's terminated
            char *b = osSerialStr, *bs = osSerialStr;
            while (*b && isspace(*b)) { b++; } // scan for leading spaces
            while (*b && (isdigit(*b) || isalpha(*b))) { 
                *bs++ = toupper(*b);
                b++; 
            }
            *bs = 0; // terminate at end of valid characters
        }
        
        /* serial number not available from file, use systems serial number */
        if (!*osSerialStr) {
            if (useBluetooth && osHasBluetooth() && (osGetBluetoothMac() > 0LL)) {
                osGetBluetoothMacID(osSerialStr);
            } else
            if (osGetSerialNumber() > 0LL) {
                osGetSerialNumberID(osSerialStr);
            } else {
                // still can't get serial number, ignore ...
            }
        }
        
    }
    return osSerialStr;
}

// ----------------------------------------------------------------------------

#define K(N,S)  (Int32)((((Int64)(N) * (Int64)(S)) + 512LL) / 1024LL)

Int32 osGetDiskUsage(const char *mount, UInt32 *total, UInt32 *avail)
{
    struct statfs st;
    if (mount && (statfs(mount, &st) == 0)) {
        Int32 t = K(st.f_blocks, st.f_bsize);
        Int32 a = K(st.f_bavail, st.f_bsize); // non-superuser
        if (total) { *total = t; }
        if (avail) { *avail = a; }
        return a;
    } else {
        return -1L;
    }
}

// ----------------------------------------------------------------------------
static struct utsname mymachine;
static char osinfo[64];
char *osGetOSInfo()
{
	int err;
	char *result = NULL;

	err = uname(&mymachine);
	if (!err) {
		strncpy(osinfo, mymachine.sysname, 256);
		strcat(osinfo, " ");
		strncat(osinfo, mymachine.release, 256);
		result = osinfo;
	}
	return result;
}
