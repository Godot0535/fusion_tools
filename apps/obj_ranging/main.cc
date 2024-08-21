#include "apps/obj_ranging/include/obj_ranging.h"
#include <glog/logging.h>

int main(int argc, char const *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::INFO);
  obj_ranging::ObjRanging obj_ranging_app;
  while (obj_ranging_app.Run());
  return 0;
}
