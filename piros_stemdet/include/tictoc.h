#ifndef TICTOC_H
#define TICTOC_H

#include <sys/time.h>
class TicToc {
 public:
  void tic() { gettimeofday(&_start, 0); }

  double toc() {
    gettimeofday(&_stop, 0);
    // calculate duration
    // seconds
    _duration = _stop.tv_sec - _start.tv_sec;
    // microseconds
    _duration += (_stop.tv_usec - _start.tv_usec) / 1e6;
    return _duration;
  }

  double duration() { return _duration; }
  void reset() {
    _start.tv_sec = 0;
    _start.tv_usec = 0;

    _stop.tv_sec = 0;
    _stop.tv_usec = 0;

    _duration = 0.0;
  }

 private:
  timeval _start;
  timeval _stop;
  // duration:
  double _duration;
};

#endif  // TICTOC_H