

#include "logger_util.h"

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/utsname.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>

#include "glog/logging.h"



static int32_t g_main_thread_pid = getpid();

int32_t GetMainThreadPid() { return g_main_thread_pid; }

bool PidHasChanged() {
  int32_t pid = getpid();
  if (g_main_thread_pid == pid) {
    return false;
  }
  g_main_thread_pid = pid;
  return true;
}

