#pragma once
#include "world.h"

// TODO: Use std::is_debugger_present when support arrives
#ifndef NDEBUG
#include <cstdio>

#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <sys/sysctl.h>
#include <sys/types.h>
#include <unistd.h>
#elif defined(__linux__)
#include <fstream>
#include <string>
#endif

inline bool is_debugger_present_impl()
{
#if defined(_WIN32)
    return ::IsDebuggerPresent() != 0;

#elif defined(__APPLE__)
    int mib[4] = {CTL_KERN, KERN_PROC, KERN_PROC_PID, getpid()};
    struct kinfo_proc info{};
    size_t size = sizeof(info);

    if (sysctl(mib, 4, &info, &size, nullptr, 0) == -1) return false;

    return (info.kp_proc.p_flag & P_TRACED) != 0;

#elif defined(__linux__)
    std::ifstream status("/proc/self/status");
    if (!status.is_open()) return false;

    std::string line;
    while (std::getline(status, line))
    {
        if (line.starts_with("TracerPid:"))
        {
            std::size_t pos = line.find_first_not_of(" \t", 10);
            if (pos == std::string::npos) return false;
            return line[pos] != '0';
        }
    }
    return false;

#else
    // Unsupported platform — assume no debugger
    return false;
#endif
}

#include <iostream>
#include <print>

inline bool is_debugger_present()
{
    static bool present = []()
    {
        if (is_debugger_present_impl())
        {
            std::println(std::cerr,
                         "Warning: Debugger detected. Limiting timestep to prevent instability.\n");
            return true;
        }
        return false;
    }();
    return present;
}
#endif

namespace physkit
{
class fixed_accum
{
public:
    fixed_accum(quantity<si::second> fixed_dt) : M_dt(fixed_dt) {}

    void update(quantity<si::second> dt)
    {
#ifndef NDEBUG
        if (is_debugger_present()) dt = std::min(dt, 0.1 * si::second);
#endif
        M_accum += dt;
    }

    [[nodiscard]] bool should_step()
    {
        if (M_accum >= M_dt)
        {
            M_accum -= M_dt;
            return true;
        }
        return false;
    }

    [[nodiscard]] auto step_size() const { return M_dt; }
    [[nodiscard]] auto accum() const { return M_accum; }
    [[nodiscard]] auto alpha() const { return M_accum / M_dt; }

private:
    quantity<si::second> M_dt;
    quantity<si::second> M_accum = 0 * si::second;
};

class stepper
{
public:
    stepper() : M_accum(1.0 / 60.0 * si::second), M_w{} {}
    stepper(world_base &w, quantity<si::second> dt) : M_accum(dt), M_w(&w) {}

    void update(quantity<si::second> dt)
    {
        M_accum.update(dt);
        while (M_accum.should_step()) M_w->step(M_accum.step_size());
    }

    [[nodiscard]] auto accum() const { return M_accum; }
    [[nodiscard]] auto step_size() const { return M_accum.step_size(); }
    [[nodiscard]] auto alpha() const { return M_accum.alpha(); }

    // resets accumulator and sets world
    void reset(world_base &w, quantity<si::second> dt)
    {
        M_w = &w;
        M_accum = fixed_accum{dt};
    }

private:
    fixed_accum M_accum;
    world_base *M_w;
};

} // namespace physkit