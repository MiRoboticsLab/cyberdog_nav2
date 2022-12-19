
#ifndef ALGORITHM_MANAGER__TIMMER_HPP_
#define ALGORITHM_MANAGER__TIMMER_HPP_

#include <chrono>

namespace cyberdog
{
namespace algorithm
{

class Timer 
{
 public:
  Timer();

  void Start();
  void Restart();
  void Pause();
  void Resume();
  void Reset();

  double ElapsedMicroSeconds() const;
  double ElapsedSeconds() const;
  double ElapsedMinutes() const;
  double ElapsedHours() const;

 private:
  bool started_;
  bool paused_;
  std::chrono::high_resolution_clock::time_point start_time_;
  std::chrono::high_resolution_clock::time_point pause_time_;
};

}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__TIMMER_HPP_
