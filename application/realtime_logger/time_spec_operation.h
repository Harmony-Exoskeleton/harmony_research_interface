#ifndef TIME_SPEC_OPERATION_H
#define TIME_SPEC_OPERATION_H

#include <stdint.h>
#include <time.h>

#define NSEC_PER_SEC 1000000000

#define TIMESPEC_INCREMENT(A,B) /* A += B */ \
    do {                                   \
    (A).tv_sec += (B).tv_sec;          \
    (A).tv_nsec += (B).tv_nsec;        \
    if ( (A).tv_nsec >= 1000000000 ) { \
    (A).tv_sec++;                  \
    (A).tv_nsec -= 1000000000;     \
    }                                  \
    } while (0)

#define TIMESPEC_DECREMENT(A,B) /* A -= B */ \
    do {                                   \
    (A).tv_sec -= (B).tv_sec;          \
    (A).tv_nsec -= (B).tv_nsec;        \
    if ( (A).tv_nsec < 0 ) {           \
    (A).tv_sec--;                  \
    (A).tv_nsec += 1000000000;     \
    }                                  \
    } while (0)

/* Subtract timespecs
 *
 * \param r[out] result: a - b
 * \param a[in] operand
 * \param b[in] operand
 */

static inline void
timespec_sub(struct timespec *r,
         const struct timespec *a, const struct timespec *b)
{
    r->tv_sec = a->tv_sec - b->tv_sec;
    r->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (r->tv_nsec < 0) {
        r->tv_sec--;
        r->tv_nsec += NSEC_PER_SEC;
    }
}

/* Add a nanosecond value to a timespec
 *
 * \param r[out] result: a + b
 * \param a[in] base operand as timespec
 * \param b[in] operand in nanoseconds
 */
static inline void
timespec_add_nsec(struct timespec *r, const struct timespec *a, int64_t b)
{
    r->tv_sec = a->tv_sec + (b / NSEC_PER_SEC);
    r->tv_nsec = a->tv_nsec + (b % NSEC_PER_SEC);

    if (r->tv_nsec >= NSEC_PER_SEC) {
        r->tv_sec++;
        r->tv_nsec -= NSEC_PER_SEC;
    } else if (r->tv_nsec < 0) {
        r->tv_sec--;
        r->tv_nsec += NSEC_PER_SEC;
    }
}

/* Add a millisecond value to a timespec
 *
 * \param r[out] result: a + b
 * \param a[in] base operand as timespec
 * \param b[in] operand in milliseconds
 */
static inline void
timespec_add_msec(struct timespec *r, const struct timespec *a, int64_t b)
{
    timespec_add_nsec(r, a, b * 1000000);
}

/* Convert timespec to nanoseconds
 *
 * \param a timespec
 * \return nanoseconds
 */
static inline int64_t
timespec_to_nsec(const struct timespec *a)
{
    return (int64_t)a->tv_sec * NSEC_PER_SEC + a->tv_nsec;
}

/* Subtract timespecs and return result in nanoseconds
 *
 * \param a[in] operand
 * \param b[in] operand
 * \return to_nanoseconds(a - b)
 */
static inline int64_t
timespec_sub_to_nsec(const struct timespec *a, const struct timespec *b)
{
    struct timespec r;
    timespec_sub(&r, a, b);
    return timespec_to_nsec(&r);
}

/* Convert timespec to milliseconds
 *
 * \param a timespec
 * \return milliseconds
 *
 * Rounding to integer milliseconds happens always down (floor()).
 */
static inline int64_t
timespec_to_msec(const struct timespec *a)
{
    return (int64_t)a->tv_sec * 1000 + a->tv_nsec / 1000000;
}

/* Subtract timespecs and return result in milliseconds
 *
 * \param a[in] operand
 * \param b[in] operand
 * \return to_milliseconds(a - b)
 */
static inline int64_t
timespec_sub_to_msec(const struct timespec *a, const struct timespec *b)
{
    return timespec_sub_to_nsec(a, b) / 1000000;
}

/* Convert timespec to microseconds
 *
 * \param a timespec
 * \return microseconds
 *
 * Rounding to integer microseconds happens always down (floor()).
 */
static inline int64_t
timespec_to_usec(const struct timespec *a)
{
    return (int64_t)a->tv_sec * 1000000 + a->tv_nsec / 1000;
}

/* Convert nanoseconds to timespec
 *
 * \param a timespec
 * \param b nanoseconds
 */
static inline void
timespec_from_nsec(struct timespec *a, int64_t b)
{
    a->tv_sec = b / NSEC_PER_SEC;
    a->tv_nsec = b % NSEC_PER_SEC;
}

/* Convert microseconds to timespec
 *
 * \param a timespec
 * \param b microseconds
 */
static inline void
timespec_from_usec(struct timespec *a, int64_t b)
{
    timespec_from_nsec(a, b * 1000);
}

/* Convert milliseconds to timespec
 *
 * \param a timespec
 * \param b milliseconds
 */
static inline void
timespec_from_msec(struct timespec *a, int64_t b)
{
    timespec_from_nsec(a, b * 1000000);
}

/* Convert protocol data to timespec
 *
 * \param a[out] timespec
 * \param tv_sec_hi the high bytes of seconds part
 * \param tv_sec_lo the low bytes of seconds part
 * \param tv_nsec the nanoseconds part
 */
static inline void
timespec_from_proto(struct timespec *a, uint32_t tv_sec_hi,
                    uint32_t tv_sec_lo, uint32_t tv_nsec)
{
    a->tv_sec = ((uint64_t)tv_sec_hi << 32) + tv_sec_lo;
    a->tv_nsec = tv_nsec;
}

/* Check if a timespec is zero
 *
 * \param a timespec
 * \return whether the timespec is zero
 */
static inline bool
timespec_is_zero(const struct timespec *a)
{
    return a->tv_sec == 0 && a->tv_nsec == 0;
}

/* Check if two timespecs are equal
 *
 * \param a[in] timespec to check
 * \param b[in] timespec to check
 * \return whether timespecs a and b are equal
 */
static inline bool
timespec_eq(const struct timespec *a, const struct timespec *b)
{
    return a->tv_sec == b->tv_sec &&
           a->tv_nsec == b->tv_nsec;
}

#endif // TIME_SPEC_OPERATION_H

