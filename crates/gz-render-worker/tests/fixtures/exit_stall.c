/* Copyright (C) 2026 Open Source Robotics Foundation */
/* SPDX-License-Identifier: Apache-2.0 */
/* Test-only native exit stall, not a GPU fault or a Gazebo reference. */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

__attribute__((destructor)) static void stall_native_exit(void)
{
  const char *path = getenv("GZ_RENDER_NATIVE_EXIT_MARKER");
  if (!path || !*path)
    return;
  int fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0600);
  if (fd < 0)
    _exit(124);
  char message[80];
  int length = snprintf(message, sizeof(message), "native_exit_entered pid=%ld\n", (long)getpid());
  if (length <= 0 || length >= (int)sizeof(message) || write(fd, message, (size_t)length) != length)
    _exit(125);
  close(fd);
  for (;;)
    pause();
}
