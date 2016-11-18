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

#ifndef _XPORT_SOCKET_H
#define _XPORT_SOCKET_H
#ifdef __cplusplus
extern "C" {
#endif

#include "transport.h"

// ----------------------------------------------------------------------------
/* transport initialization */
extern TransportFtns_t socketTCP_funcs;
extern TransportFtns_t socketUDP_funcs;

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
