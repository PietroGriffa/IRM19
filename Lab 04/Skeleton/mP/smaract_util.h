// $Id: smaract_util.h,v 1.1 2009/12/15 12:14:22 bradleyk Exp $

#ifndef SMARACT_UTIL_H
#define SMARACT_UTIL_H

#include <math.h>
#include <sys/time.h>
#include <time.h>

/// Convert radians to degrees
inline double rtod(double r)
{
  return r * 180.0 / M_PI;
}

/// Convert degrees to radians
inline double dtor(double r)
{
  return r * M_PI / 180.0;
}

/// Normalize angle to domain -pi, pi
inline double normalize(double z)
{
  return atan2(sin(z), cos(z));
}

// round a value to a power of 10 (similar to the matlab function)
//inline double roundn(double x, int n)
//{
   //Compute the exponential factors for rounding at specified
   //power of 10.  Ensure that n is an integer.
  //double factors = pow(10, -n);
    //Set the significant digits for the input data
  //return round(x * factors) / factors;
//}

static struct timeval start_time = {0,0};

// tic and toc functions work together to measure elapsed time.
// tic saves the current time that TOC uses later to measure
// the elapsed time. The sequence of commands:
//
//           tic();
//           operations
//           toc();
//
// measures the amount of time MATLAB takes to complete the one
// or more operations specified here by "operations" and displays
// the time in seconds.
inline void
tic()
{
  gettimeofday(&start_time, NULL);
}

inline double
toc(int restart)
{
  struct timeval now;
  gettimeofday(&now, NULL);

  double elapsed_time = now.tv_sec - start_time.tv_sec +
                        (now.tv_usec - start_time.tv_usec)/1e6;

  if (0 != restart)
    tic();

  return elapsed_time;
}

#endif
