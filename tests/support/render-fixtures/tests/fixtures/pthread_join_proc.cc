// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>

static void *worker(void *arg) {
  auto *tid = static_cast<long *>(arg);
  if (prctl(PR_SET_NAME, "gzjoin-probe", 0, 0, 0) != 0) std::abort();
  *tid = syscall(SYS_gettid);
  return nullptr;
}
int main() {
  int visible = 0;
  for (int i = 0; i < 50000; ++i) {
    long tid = 0;
    pthread_t thread;
    if (pthread_create(&thread, nullptr, worker, &tid) != 0 || pthread_join(thread, nullptr) != 0) std::abort();
    char path[128];
    std::snprintf(path, sizeof(path), "/proc/self/task/%ld/comm", tid);
    if (FILE *file = std::fopen(path, "r")) {
      char name[64] = {};
      const bool read = std::fgets(name, sizeof(name), file) != nullptr;
      std::fclose(file);
      if (read && std::strcmp(name, "gzjoin-probe\n") == 0) {
        ++visible;
        if (visible <= 5) {
          errno = 0;
          const long signal = syscall(SYS_tgkill, getpid(), tid, 0);
          const int error = errno;
          std::snprintf(path, sizeof(path), "/proc/self/task/%ld/status", tid);
          FILE *status = std::fopen(path, "r");
          char row[256] = {};
          if (status) {
            while (std::fgets(row, sizeof(row), status) && std::strncmp(row, "State:", 6) != 0) {}
            std::fclose(status);
          }
          std::printf("joined thread visible: tid=%ld signal=%ld errno=%d %s", tid, signal, error, row);
        }
      }
    }
  }
  std::printf("joined thread still visible %d/50000\n", visible);
}
