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

/*
 * common.h
 *
 * ZEPHYR OS abstraction / internal utilities
 *
 * Functions are defined by common.c
 *
 */

#ifndef __ZEPHYR_COMMON_
#define __ZEPHYR_COMMON_


/******* Framework headers : */
#include "os/os_types.h"    /* framework-specific types */



/******************/

/**********************************************************
 ************** Exported macros   *************************
 ********************************************************/
#define DISABLE_INT        interrupt_lock
#define RESTORE_INT(flags) interrupt_unlock (flags)


#define INT_FLAGS uint32_t
#define INT_SIZE (sizeof(unsigned int) * 8)


#define DECLARE_BLK_ALLOC(var, type, count) \
type var##_elements[count]; \
unsigned int var##_track_alloc[count/INT_SIZE+1] = { 0 }; \
unsigned int var##_alloc_max = 0; \
unsigned int var##_alloc_cur = 0; \
type * var##_alloc(void) { \
    int i; \
    INT_FLAGS flags = DISABLE_INT(); \
    for ( i = 0; i < count; i++) { \
        int wi = i/INT_SIZE; \
        int bi = INT_SIZE - 1 - (i%INT_SIZE); \
        if ((var##_track_alloc[wi] & 1 << bi) == 0) { \
            var##_track_alloc[wi] |= 1 << (bi); \
            var##_alloc_cur++; \
            if (var##_alloc_cur > var##_alloc_max) var##_alloc_max = var##_alloc_cur;\
            RESTORE_INT(flags); \
            return &var##_elements[i]; \
        } \
    }\
    RESTORE_INT(flags); \
    return NULL; \
} \
void var##_free(type * ptr) { \
   int index =  ptr - var##_elements; \
   INT_FLAGS flags; \
   if (index < count) { \
       flags = DISABLE_INT(); \
       if ((var##_track_alloc[index/INT_SIZE] & 1 << (INT_SIZE - 1 -(index%INT_SIZE))) == 0) { \
           _log("Already free"); \
           return; \
       } \
       var##_track_alloc[index/INT_SIZE] &= ~(1 << (INT_SIZE-1-(index%INT_SIZE))); \
       RESTORE_INT(flags); \
       var##_alloc_cur--; \
   } \
} \
bool var##_used(type * ptr) { \
   int index =  ptr - var##_elements; \
   if ( (index >= 0) && (index<count) ) { \
       if ((var##_track_alloc[index/INT_SIZE] & (1 << (INT_SIZE - 1 -(index%INT_SIZE)))) != 0) { \
           return true; \
       } \
   } \
   return false; \
}





/* Macros for managing resource usage -- TODO: remove and use DECLARE_BLK_ALLOC when sync.c is updated */
#define IS_OBJ_RESERVED(pool,idx)         (( (pool) & (1 << (idx)) ) != 0 )
#define RESERVE_OBJ(pool,idx)             (pool) |= (1 << (idx))
#define UNRESERVE_OBJ(pool,idx)           (pool) &= ~(1 << (idx))




/**********************************************************
 ************** Exported definitions  *********************
 ********************************************************/

/**********************************************************
 ************** Exported variables   **********************
 ********************************************************/

/**********************************************************
 ************** Exported functions ************************
 ********************************************************/


/**
  * \brief Lock access to a pool of synchronization objects
  *
  * This function manages the concurrent accesses to the pools
  * of semaphores and mutexes.
  *
  * This function is only used when creating and deleting semaphores or mutexes.
  *
  * Rationale:
  *  This function does not manage concurrent accesses to the pools when running
  *  from an ISR context. ( Creating a semaphore or a mutex in an ISR does not
  *  seem a valid use-case anyway ).
  */
extern void _PoolLock ( void );

 /**
  * \brief Unlock access to a pool of synchronization objects
  *
  * This function manages the concurrent accesses to the pools
  * of semaphores and mutexes.
  *
  * This function is only used when creating and deleting semaphores or mutexes.
  *
  * Rationale:
  *  This function does not manage concurrent accesses to the pools when running
  *  from an ISR context. ( Creating a semaphore or a mutex in an ISR does not
  *  seem a valid use-case anyway ).
  *
  */
extern  void _PoolUnlock ( void );

 /**
  * \brief Initialize private variables
  *
  * IMPORTANT : this function must be called during the initialization
  *             of the OS abstraction layer.
  *             this function shall only be called once after reset, otherwise
  *             it may cause the take/lock and give/unlock services to fail
  */
extern void framework_init_common (void);


/**
 * \brief Copy error code to caller's variable, or panic if caller did not specify
 * an error variable
 *
 * \param [out] err : pointer to caller's error variable - may be _NULL
 * \param [in] localErr : error detected by the function
 *
 */
extern void error_management (OS_ERR_TYPE* err, OS_ERR_TYPE localErr);


/**
 * \brief Convert VxMicro's semaphore API return value to OS_ERR_TYPE error code
 *
 * \param [in] vxErr : error code returned by the microkernel's semaphore functions
 *
 * \return framework standardized error code
 */
extern OS_ERR_TYPE _VxErrToOsErr ( int vxErr );




#endif /* __ZEPHYR_COMMON_ */
