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

#ifndef _PQUEUE_H
#define _PQUEUE_H
#ifdef __cplusplus
extern "C" {
#endif

#include "defaults.h"

#include "stdtypes.h"
#include "bintools.h"
#include "threads.h"

#include "packet.h"
// ----------------------------------------------------------------------------
#define PACKETS_PER_PAGE_EXPONENT 5
#define PACKET_INDEX_BITS	PACKETS_PER_PAGE_EXPONENT
#define PACKETS_PER_PAGE	(1 << PACKET_INDEX_BITS)
#define PACKET_INDEX_MASK	((1 << PACKET_INDEX_BITS) - 1)
#define PAGE_ARRAY_SIZE		EVENT_QUEUE_SIZE / PACKETS_PER_PAGE
#define PAGE_INDEX_BITS		(EVENT_QUEUE_SIZE_EXPONENT - PACKETS_PER_PAGE_EXPONENT)
#define PAGE_INDEX_MASK		((1 << PAGE_INDEX_BITS) - 1)
#define RESET_CLEANUP 1
#define PRESERVE_RESTORE_SIZE (EVENT_QUEUE_SIZE - PACKETS_PER_PAGE)

typedef struct {
    utBool              queOverwrite;
    utBool              expandable;
    Int32               queSize;        // total item count
    Int32               queFirst;       // first index (first valid packet if != queLast)
    Int32               queLast;        // last index (always points to invalid/unallocated packet)
    Packet_t		*queue[PAGE_ARRAY_SIZE];        // malloc'ed entries (queSize + 1)
    //Packet_t          *queue;       <-- (non-malloc'ed) pointer to pre-allocated array
#ifdef PQUEUE_THREAD_LOCK
    threadMutex_t       queMutex;
#endif
} PacketQueue_t;

typedef struct {
    PacketQueue_t       *pque;
    Int32               index; // must be signed (-1 means 'no index')
} PacketQueueIterator_t;

// 'malloc' is used to maintain entries in the queue
#define PacketQueue_DEFINE(N,S)     static PacketQueue_t N;
#define PacketQueue_INIT(N,S)       pqueInitQueue(&(N),(S))
// [obsolete] (the following assumes pre-allocated entry slots: 
//#define PacketQueue_STRUCT(P,S)     { utTrue, (S), 0, 0, (P) }
//#define PacketQueue_DEFINE(N,S)     static Packet_t _##N[(S)+1]; static PacketQueue_t N=PacketQueue_STRUCT(_##N,(S)+1);
//#define PacketQueue_INIT(N,S)       pqueInitQueue(&(N),(N).queue,(N).queSize)
// ----------------------------------------------------------------------------

void pqueInitQueue(PacketQueue_t *pq, int queSize);
void pqueEnableOverwrite(PacketQueue_t *pq, utBool overwrite);
Int32 pqueGetPacketCount(PacketQueue_t *pq);
utBool pqueHasPackets(PacketQueue_t *pq);
utBool pqueAddPacket(PacketQueue_t *pq, Packet_t *pkt);
utBool pqueHasUnsentPacket(PacketQueue_t *pq);
/*UInt32 pqueGetFirstSentSequence(PacketQueue_t *pq); */
UInt32 pqueGetLastSequence(PacketQueue_t *pq, time_t * timestamp);
utBool pqueDeleteSentPackets(PacketQueue_t *pq, UInt32 sequence, int total);
PacketPriority_t pqueGetHighestPriority(PacketQueue_t *pq);
void pqueResetQueue(PacketQueue_t *pq);
utBool pqueRestoreSentPacket(PacketQueue_t *pq);
utBool pqueIsPacketSent(Packet_t *pkt);
void pqueMarkPacketSent(Packet_t *pkt);
PacketQueueIterator_t *pqueGetIterator(PacketQueue_t *pq, PacketQueueIterator_t *i);
utBool pqueHasNextPacket(PacketQueueIterator_t *i);
Packet_t *pqueGetNextPacket(Packet_t *pktCopy, PacketQueueIterator_t *i);
void pqueReleaseQueue(PacketQueue_t *pq);
int pquePreserveQueue(PacketQueue_t *pq);
int pqueRestoreQueue(PacketQueue_t *pq);
void pqueResetPreserve(void);
void pqueUpdateTimestamp(PacketQueue_t *pq, long adjustment);
void pqueTuneTimestamp(PacketQueue_t *pq, long adjustment);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
