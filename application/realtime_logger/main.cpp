/**
 * @file realtime_logger
 * @author S. Dalla Gasperina (stefano.dallagasperina@austin.utexas.edu)
 * @brief Real-time logger
 * @version v1.0
 * @date 2024-07-12
 *
 * @copyright Copyright (c) 2024
 *
 */

/******************************************************************************************
 * INCLUDES
 *****************************************************************************************/
#include "research_interface.h"
#include "plog/Log.h"
#include "plog/Appenders/ColorConsoleAppender.h"
#include "csv_handling.h"
#include "log_manager.h"

#include <sys/mman.h>
#include "time_spec_operation.h"

using namespace std;

/******************************************************************************************
 * DEFINES
 *****************************************************************************************/

#define DEFAULT_LOOP_TIME_NS 1000000L
#define DEFAULT_APP_DURATION_COUNTS 10000
#define ALLOWED_LOOPTIME_OVERFLOW_NS 100000L

/******************************************************************************************
*   GET SCHED PARAMETERS
******************************************************************************************/

static void getinfo ()
{
    struct sched_param param;
    int policy;

    sched_getparam(0, &param);
    printf("Priority of the process: %d\n\r", param.sched_priority);

    pthread_getschedparam(pthread_self(), &policy, &param);
    printf("Priority of the thread: %d, current policy is: %d\n\r",
              param.sched_priority, policy);
}

/******************************************************************************************
*   LATENCY TRICK
******************************************************************************************/

static void set_latency_target(void)
{
    struct stat s;
    int err;
    int latency_target_value = 0;

    errno = 0;
    err = stat("/dev/cpu_dma_latency", &s);
    if (err == -1) {
        printf("WARN: stat /dev/cpu_dma_latency failed");
        return;
    }

    errno = 0;
    int latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1) {
        printf("WARN: open /dev/cpu_dma_latency");
        return;
    }

    errno = 0;
    err = write(latency_target_fd, &latency_target_value, 4);
    if (err < 1) {
        printf("# error setting cpu_dma_latency to %d!", latency_target_value);
        close(latency_target_fd);
        return;
    }
    printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
}


/******************************************************************************************
 * REAL-TIME LOOP
 *****************************************************************************************/

void* main_loop(void* arg){

    struct timespec t_start, t_now, t_next,t_period,t_result,t_overflow; // timespec variable to handle timing
    unsigned long int loop_count = 0;   // counter for cycle loops

    // Lock memory to disable paging
    if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        perror("mlockall");
    }

    set_latency_target();

    getinfo();

    // Calculate the time period for the execution of this task
    t_period.tv_sec = 0;
    t_period.tv_nsec = DEFAULT_LOOP_TIME_NS;

    // Get starting time
    clock_gettime(CLOCK_MONOTONIC, &t_start);
    clock_gettime( CLOCK_MONOTONIC, &t_now);
    t_next = t_now;

    PLOGW << "Research Interface started";

    // Init Research Interface
    harmony::ResearchInterface info;

    // Init Logger
    csv_log_file log_manager;

    if (!info.init()) {
        PLOGE << "Research Interface failed to initialize!";
        return nullptr;
    }

    log_manager.create_file(logger::create_header());

    /* Cyclic Loop */
    while(loop_count<DEFAULT_APP_DURATION_COUNTS){

        // Compute next cycle deadline
        timespec_add_nsec(&t_next,&t_next,DEFAULT_LOOP_TIME_NS);

        // Other options to increment
//        TIMESPEC_INCREMENT ( t_next, t_period );

        // Debug cycle at 1 Hz
        if(loop_count%1000==0){

            // Compute time since start
            timespec_sub(&t_result,&t_now,&t_start);
            PLOGI << "RT Clock: " << (double) (timespec_to_nsec(&t_result)/1e9) << endl;

            // Run my cyclic task
            // Do work here
            log_manager.log_data(logger::create_dataline(loop_count,0));

        }


        loop_count++;

        // Sleep until the next execution time
        clock_nanosleep ( CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, nullptr );
        // Get time after sleep
        clock_gettime ( CLOCK_MONOTONIC, &t_now);
        // Compute overflow time
        timespec_sub(&t_overflow,&t_now,&t_next);
        // If overflow is too big, print overrun
        if(t_overflow.tv_nsec > ALLOWED_LOOPTIME_OVERFLOW_NS)
        {
            PLOGE << "RT Overflow: " << (double)t_overflow.tv_nsec/1000 << endl;
        }

    }

    PLOGI << "Research Interface stopped";

}

/******************************************************************************************
 * MAIN
 *****************************************************************************************/
int main() {

    static plog::RollingFileAppender<plog::CsvFormatter> fileAppender("log/harmony_interface_log.csv", 80000, 10); // Create the 1st appender.
    plog::init(plog::info, &fileAppender);

    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.
    plog::get()->addAppender(&consoleAppender);

    // Scheduler variables
    int policy;
    struct sched_param prio;
    pthread_attr_t attr;

    // pthreads variables
    pthread_t rt_loop;

    /********************/
    /* REAL-TIME THREAD */
    /********************/

    // Set explicit attributes for thread creation
    pthread_attr_init( &attr);
    pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);

    // Set scheduler policy
    policy = SCHED_RR;
    pthread_attr_setschedpolicy( &attr, policy);

    // Set scheduler priority
    // A static priority value is assigned to each process and scheduling depends on this static priority.
    // Priority range should be btw 1 and 99
    prio.sched_priority = 50 ;
    pthread_attr_setschedparam(&attr,&prio);

    if ( pthread_create(&rt_loop, &attr, main_loop, nullptr ) ){
        PLOGE <<  "Research Interface real-time loop" ;
        return 1;
    }
    // Wait for threads to finish
    pthread_join(rt_loop,NULL);

    return 0;
}
