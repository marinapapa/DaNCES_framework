#pragma once

// get the path to the current binary
// Hanno 2022

#include <filesystem>

#if defined _WIN32
# include <Windows.h>
#elif defined __APPLE__
# include <sys/syslimits.h>
#endif


namespace exe_path {

  inline std::filesystem::path get() {
#ifdef _WIN32
    char buf[MAX_PATH];
    GetModuleFileNameA(NULL, buf, sizeof(buf));
    return std::filesystem::canonical(buf).parent_path();
#elif defined __Apple__
    char path[PATH_MAX + 1];
    uint32_t size = sizeof(path);
    _NSGetExecutablePath(path, &size);
    return std::filesystem::canonical(path);
#else // Linux
    return std::filesystem::canonical("/proc/self/exe").parent_path();
#endif
  }

}
