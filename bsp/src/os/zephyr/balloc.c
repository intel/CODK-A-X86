/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @defgroup os_zephyr Zephyr OS Abstraction Layer
 * @ingroup os
 * @{
 */

/**
 * \file malloc.c
 *
 * ZEPHYR OS abstraction / memory allocation services
 *
 * Functions are exported by os.h
 *
 */


#include <stdio.h>

/******* Framework headers : */
#include "os/os.h"          /* framework export definitions */
#include "os/os_types.h"    /* framework-specific types */
#include "zephyr/common.h"
#include "zephyr/os_config.h"
#include "infra/log.h"

#ifdef CONFIG_BALLOC_STATISTICS_TRACK_OWNER
#include "misc/printk.h"
#include <string.h>
#include "balloc_debug.h"
#endif

#ifdef CONFIG_BALLOC_STATISTICS
#include "zephyr/os_specific.h"   /* need _log function */
#endif


/**********************************************************
 ************** Extern variables   ************************
 **********************************************************/




/**********************************************************
 ************** Local definitions  ************************
 **********************************************************/
#define BITS_PER_UINT32 (sizeof(uint32_t) * 8)

/** Descriptor for a memory pool */
typedef struct {
    uint32_t* track;    /** block allocation tracker */
    uint32_t  start;    /** start address of the pool */
    uint32_t  end;      /** end address of the pool */
    uint16_t  count;    /** total number of blocks within the pool */
    uint16_t  size;     /** size of each memory block within the pool */
#ifdef CONFIG_BALLOC_STATISTICS
#ifdef CONFIG_BALLOC_STATISTICS_TRACK_OWNER
    uint32_t **owners;
#endif
    uint32_t  max;     /** maximum number of allocated blocks at the same time */
    uint32_t  cur;     /** current number of allocated blocks */
    uint32_t  sum;     /** Cumulative size in bytes */
    uint32_t  nbrs;    /** Cumulative block allocated */
#endif
}T_POOL_DESC;

/**********************************************************
 ************** Private variables  ************************
 **********************************************************/

#ifdef CONFIG_BALLOC_STATISTICS

/** Allocate the memory blocks and tracking variables for each pool */
#ifdef CONFIG_BALLOC_STATISTICS_TRACK_OWNER
#define DECLARE_MEMORY_POOL(index,size,count) \
    uint8_t  g_MemBlock_##index[count][size] __attribute__((aligned(4))); \
    uint32_t g_MemBlock_alloc_track_##index[count/BITS_PER_UINT32+1] = { 0 };\
    uint32_t *g_MemBlock_owners_##index[count] = { 0 };
#else
#define DECLARE_MEMORY_POOL(index,size,count) \
    uint8_t  g_MemBlock_##index[count][size] __attribute__((aligned(4))); \
    uint32_t g_MemBlock_alloc_track_##index[count/BITS_PER_UINT32+1] = { 0 };
#endif

#include "memory_pool_list.def"


/** Pool descriptor definition */
T_POOL_DESC g_MemPool[] =
{
#ifdef CONFIG_BALLOC_STATISTICS_TRACK_OWNER
#define DECLARE_MEMORY_POOL(index,size,count) \
{ \
/* T_POOL_DESC.track */  g_MemBlock_alloc_track_##index,\
/* T_POOL_DESC.start */  (uint32_t) g_MemBlock_##index, \
/* T_POOL_DESC.end */    (uint32_t) g_MemBlock_##index + count * size, \
/* T_POOL_DESC.count */  count, \
/* T_POOL_DESC.size */   size, \
/* T_POOL_DESC.owners */ g_MemBlock_owners_##index, \
/* T_POOL_DESC.max */    0, \
/* T_POOL_DESC.cur */    0, \
/* T_POOL_DESC.sum */    0, \
/* T_POOL_DESC.nbrs */   0 \
},
#else
#define DECLARE_MEMORY_POOL(index,size,count) \
{ \
/* T_POOL_DESC.track */  g_MemBlock_alloc_track_##index,\
/* T_POOL_DESC.start */  (uint32_t) g_MemBlock_##index, \
/* T_POOL_DESC.end */    (uint32_t) g_MemBlock_##index + count * size, \
/* T_POOL_DESC.count */  count, \
/* T_POOL_DESC.size */   size, \
/* T_POOL_DESC.max */    0, \
/* T_POOL_DESC.cur */    0, \
/* T_POOL_DESC.sum */    0, \
/* T_POOL_DESC.nbrs */   0 \
},
#endif

#include "memory_pool_list.def"
};


#else

/** Allocate the memory blocks and tracking variables for each pool */
#define DECLARE_MEMORY_POOL(index,size,count) \
    uint8_t  g_MemBlock_##index[count][size] ; \
    uint32_t g_MemBlock_alloc_track_##index[count/BITS_PER_UINT32+1] = { 0 }; \

#include "memory_pool_list.def"



/** Pool descriptor definition */
T_POOL_DESC g_MemPool [] =
{
#define DECLARE_MEMORY_POOL(index,size,count) \
{ \
/* T_POOL_DESC.track */  g_MemBlock_alloc_track_##index,\
/* T_POOL_DESC.start */  (uint32_t) g_MemBlock_##index, \
/* T_POOL_DESC.end */    (uint32_t) g_MemBlock_##index + count * size, \
/* T_POOL_DESC.count */  count, \
/* T_POOL_DESC.size */   size \
},

#include "memory_pool_list.def"
};



#endif


/** Number of memory pools */
#define NB_MEMORY_POOLS   (sizeof(g_MemPool) / sizeof(T_POOL_DESC))

/**********************************************************
 ************** Private functions  ************************
 **********************************************************/

/**
 * Return the next free block of a pool and
 *   mark it as reserved/allocated.
 *
 * @param pool index of the pool in g_MemPool
 *
 * @return allocated buffer or NULL if none is
 *   available
 */
static void* memblock_alloc (uint32_t pool)
{
    uint16_t block;
    uint32_t flags = interrupt_lock();

    for (block = 0; block < g_MemPool[pool].count; block++)
    {
        if (((g_MemPool[pool].track)[block/BITS_PER_UINT32] & 1 << (BITS_PER_UINT32 - 1 - (block%BITS_PER_UINT32))) == 0)
        {
            (g_MemPool[pool].track)[block/BITS_PER_UINT32] = (g_MemPool[pool].track)[block/BITS_PER_UINT32] | (1 << (BITS_PER_UINT32 - 1 - (block%BITS_PER_UINT32)));
#ifdef CONFIG_BALLOC_STATISTICS
            g_MemPool[pool].cur = g_MemPool[pool].cur +1;
#ifdef CONFIG_BALLOC_STATISTICS_TRACK_OWNER
            g_MemPool[pool].owners[block] = __builtin_return_address(0);
#endif
            if (g_MemPool[pool].cur > g_MemPool[pool].max)
                g_MemPool[pool].max = g_MemPool[pool].cur;
#endif
            interrupt_unlock(flags);
            return (void*) ( g_MemPool[pool].start + g_MemPool[pool].size * block ) ;
        }
    }
    interrupt_unlock(flags);
    return NULL;
}



/**
 * Free an allocated block from a pool.
 *
 * @param pool index of the pool in g_MemPool
 *
 * @param ptr points to the start of the block
 *     to free
 *
 */
static void memblock_free (uint32_t pool, void* ptr)
{
    uint16_t block ;
    uint32_t flags;

    block = ((uint32_t)ptr - g_MemPool[pool].start) / g_MemPool[pool].size ;
    if (block < g_MemPool[pool].count)
    {
        flags = interrupt_lock();
        (g_MemPool[pool].track)[block/BITS_PER_UINT32] &= ~(1 << (BITS_PER_UINT32-1-(block%BITS_PER_UINT32)));
        interrupt_unlock(flags);
#ifdef CONFIG_BALLOC_STATISTICS
        g_MemPool[pool].cur = g_MemPool[pool].cur - 1;
#endif
    }
#ifdef __DEBUG_OS_ABSTRACTION_BALLOC
    else
        _log ("ERR: memblock_free: ptr 0x%X is not within pool %d [0x%X , 0x%X]", ptr, pool, g_MemPool[pool].start, g_MemPool[pool].end);
#endif
}




/**
 * Test if a block is allocated.
 *
 * @param pool index of the pool in g_MemPool
 *
 * @param ptr points to the start of the block
 *
 * @return true if the block is allocated/reserved,
 *   false if the block is free
 *
 */
static bool memblock_used (uint32_t pool, void* ptr)
{
    uint16_t block ;
    block = ((uint32_t)ptr - g_MemPool[pool].start) / g_MemPool[pool].size ;
    if (block < g_MemPool[pool].count)
    {
        if ( ((g_MemPool[pool].track)[block/BITS_PER_UINT32] & (1 << (BITS_PER_UINT32 - 1 -(block%BITS_PER_UINT32)))) != 0 )
        {
            return true;
        }
    }
    return false;
}

/**********************************************************
 ************** Exported functions ************************
 **********************************************************/

/*----- Initialization  */

/**
 * Initialize the resources used by the framework's memory allocation services
 *
 * IMPORTANT : This function must be called during the initialization
 *             of the OS abstraction layer.
 *             This function shall only be called once after reset.
 */
void os_abstraction_init_malloc(void)
{
}

/**
 * Reserves a block of memory.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * This function returns a pointer on the start of
 * a reserved memory block whose size is equal or
 * larger than the requested size.
 *
 * The returned pointer shall be null if the function
 * fails.
 *
 * This function may panic if err is null and
 *  - size is null or bigger than allowed, or
 *  - there is not enough available memory
 *
 * @param size number of bytes to reserve
 *
 *
 * @param err execution status:
 *    E_OS_OK : block was successfully reserved
 *    E_OS_ERR : size is null
 *    E_OS_ERR_NO_MEMORY: there is not enough available
 *              memory
 *    E_OS_ERR_NOT_ALLOWED : size is bigger than the
 *         biggest block size defined in os_config.h
 *
 * @return pointer to the reserved memory block
 *    or null if no block is available
 */
void* balloc (uint32_t size, OS_ERR_TYPE* err)
{
    OS_ERR_TYPE localErr = E_OS_OK;
    void* buffer = NULL ;
    uint8_t poolIdx ;

    if (size > 0)
    {
        /* find the first block size greater or equal to requested size */
        poolIdx = 0;
        while ( poolIdx < NB_MEMORY_POOLS &&
               ( size > g_MemPool[poolIdx].size ))
        {
            poolIdx ++;
        }

        /* reserve the block */
        if ( poolIdx < NB_MEMORY_POOLS )
        {
#ifdef MALLOC_ALLOW_OUTCLASS
            /* loop until an available (maybe larger) block is found */
            do
            {
                if (size <= g_MemPool[poolIdx].size )
                { /* this condition may be false if pools are not sorted according to block size */
#endif
                buffer = memblock_alloc(poolIdx);
#ifdef CONFIG_BALLOC_STATISTICS
                if ( (buffer !=NULL) &&
                    ( (poolIdx == 0) || (size > g_MemPool[poolIdx-1].size) ))
                    {
                        g_MemPool[poolIdx].nbrs += 1;
                        g_MemPool[poolIdx].sum += size;
                    }
#endif
#ifdef MALLOC_ALLOW_OUTCLASS
                }

                if ( NULL == buffer )
                {
                    poolIdx ++;
                }
            }
            while ( (poolIdx < NB_MEMORY_POOLS ) && ( NULL == buffer ) );
#endif
            if ( NULL == buffer )
            {/* All blocks of relevant size are already reserved */
                pr_error(LOG_MODULE_MAIN, "Attempt to allocate %d bytes failed", size);
                localErr = E_OS_ERR_NO_MEMORY ;
            }
        }
        else
        { /* Configuration does not define blocks large enough for the requested size */
            localErr = E_OS_ERR_NOT_ALLOWED ;
        }
    }
    else
    { /* invalid function parameter */
        localErr = E_OS_ERR ;
    }

    /* set err or panic if err == NULL and localErr != E_OS_OK */
    error_management(err, localErr);

    return (buffer);
}

/**
 * Frees a block of memory.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * This function frees a memory block that was
 * reserved by malloc.
 *
 * The "buffer" parameter must point to the
 * start of the reserved block (i.e. it shall
 * be a pointer returned by malloc).
 *
 * @param buffer pointer returned by malloc
 *
 * @return execution status:
 *    E_OS_OK : block was successfully freed
 *    E_OS_ERR : "buffer" param did not match
 *        any reserved block
 */
OS_ERR_TYPE bfree(void* buffer)
{
    OS_ERR_TYPE err = E_OS_ERR;
    uint8_t poolIdx ;

    /* find which pool the buffer was allocated from */
    poolIdx = 0;
    while (( NULL != buffer ) && ( poolIdx < NB_MEMORY_POOLS ))
    {
        /* check if buffer is within g_MemPool[poolIdx] */
        if ( ( (uint32_t) buffer >= g_MemPool[poolIdx].start ) &&
             ( (uint32_t) buffer < g_MemPool[poolIdx].end ) )
        {
            if ( false != memblock_used (poolIdx, buffer))
            {
                memblock_free (poolIdx, buffer);
                err = E_OS_OK;
            }
            /* else: buffer is not marked as used, keep err = E_OS_ERR */
#ifdef __DEBUG_OS_ABSTRACTION_BALLOC
            else
                _log ("ERR: memory_free: buffer %p is already free\n", buffer);
#endif
            buffer = NULL; /* buffer was found in the pools, end the loop */
        }
        else
        {/* buffer does not belong to g_MemPool[poolIdx], go to the next one */
            poolIdx ++;
        }
    }
    return(err);
}


#ifdef CONFIG_BALLOC_STATISTICS

#ifdef CONFIG_BALLOC_STATISTICS_TRACK_OWNER
void print_pool(void)
{
    char tmp[128];
    uint32_t pool;
    uint32_t average;
    uint16_t block;
    uint8_t str_count;
    char *cur =tmp;

    for (pool=0; pool < NB_MEMORY_POOLS; pool++) {
        str_count=0;
        average=0;
        if (g_MemPool[pool].nbrs)
            average = g_MemPool[pool].sum/g_MemPool[pool].nbrs;
        snprintf(tmp, sizeof(tmp), "\npool %-4d bytes count:%-2d cur:%-2d max:%-2d avg: %-3d \n",
                g_MemPool[pool].size,
                g_MemPool[pool].count,
                g_MemPool[pool].cur,
                g_MemPool[pool].max ,
                average);
        printk(tmp);

        memset(tmp,0,sizeof(tmp));
        str_count=0;

        for (block = 0; block < g_MemPool[pool].count; block++) {
            if (((g_MemPool[pool].track)[block / BITS_PER_UINT32] & 1 << (BITS_PER_UINT32 - 1 - (block % BITS_PER_UINT32)))) {
                if (str_count == 0) {
                    cur=tmp;
                    printk(" owners: \n");
                }
                str_count++;
                snprintf(cur, 12, " 0x%08x", g_MemPool[pool].owners[block]);
                cur += 11; /* space + 0x + 8 digits */
                if(str_count % 8 == 0) {
                    printk("%s \n",tmp);
                    memset(tmp,0,sizeof(tmp));
                    cur=tmp;
                }
            }
        }
        if(str_count % 8 )
            printk("%s \n",tmp);
    }
    printk("\n");
}
#endif
#endif

/** @} */
