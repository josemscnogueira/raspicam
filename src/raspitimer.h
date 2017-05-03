/**
 * @file   raspitimer.h
 * @author Jose Nogueira, josenogueira@biosurfit.com
 * @date   May 2017
 * @brief  Private implementation of camera in still mode
 *         Uses picture port
 *
 * (extensive explanation)
 */

#ifndef _RASPI_TIMER_H_
#define _RASPI_TIMER_H_


/**
 * Include files
 */
#include <sys/time.h>


/**
 * Typedefs
 */
typedef struct
{
    struct timeval time_init;
} TIMER_usecCtx_t;


/**
 * Initialize TIMER_usecCtx_t timer
 * @param ctx timer component
 */
void TIMER_usecStart(TIMER_usecCtx_t* ctx)
{
    gettimeofday(&ctx->time_init, NULL);
}

/**
 * Get elapsed time from timer
 * @param  ctx timer
 * @return     elapsed time in usecs
 */
unsigned int TIMER_usecElapsedUs(TIMER_usecCtx_t* ctx)
{
    /* get current time */
    struct timeval time_now;
    gettimeofday(&time_now, NULL);

    /* compute diff */
    return (1000000 * (time_now.tv_sec - ctx->time_init.tv_sec) + time_now.tv_usec - ctx->time_init.tv_usec);
}

#endif
