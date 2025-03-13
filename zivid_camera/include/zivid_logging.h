#pragma once

#include <ros/console.h>

#include <thread>

namespace zivid_logging
{
  template <typename F>
  class ScopeExit final
  {
    static_assert(std::is_invocable_v<F>);
    static_assert(std::is_nothrow_move_constructible_v<F>);

  public:
    explicit ScopeExit(F&& fn)
      : m_fn(std::move(fn))
    {
    }

    ~ScopeExit()
    {
      if (!m_released)
      {
        m_fn();
      }
    }

    ScopeExit(ScopeExit&& other) noexcept
      : m_fn(std::move(other.m_fn))
        , m_released(other.m_released)
    {
      other.release();
    }

    ScopeExit& operator=(ScopeExit&&) = delete;

    void release()
    {
      m_released = true;
    }

  private:
    F m_fn;
    bool m_released = false;
  };
}

#define ZIVIDROS_LOG_ENTRY() \
    const auto current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); \
    ROS_INFO_STREAM("ENTER:" << __func__ << " threadid=" << std::this_thread::get_id() << " time=" << std::put_time(std::localtime(&current_time), "%c %Z")); \
    auto logOnExitAlso = zivid_logging::ScopeExit([funname=std::string{__func__}]() { \
      const auto exit_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); \
      ROS_INFO_STREAM("EXIT:" << funname << " threadid=" << std::this_thread::get_id() << " time=" << std::put_time(std::localtime(&exit_time), "%c %Z")); \
    });
