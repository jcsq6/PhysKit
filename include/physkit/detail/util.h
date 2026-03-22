#pragma once

// TODO: Use std::is_debugger_present when support arrives
#ifndef NDEBUG

#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include "macros.h"
#include <cstdio>
#include <iostream>
#include <print>
#if defined(__linux__)
#include <fstream>
#include <string>
#endif
#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <sys/sysctl.h>
#include <sys/types.h>
#include <unistd.h>
#endif
#endif

namespace physkit::detail
{
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

PHYSKIT_EXPORT inline bool is_debugger_present()
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

template <typename... T> class passkey
{
    friend T...;
    passkey() = default;
};

} // namespace physkit::detail
#endif