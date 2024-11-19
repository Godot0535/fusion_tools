#include "apps/infrared_ranging/include/infrared_ranging.h"
#include <glog/logging.h>

int main(int argc, char const *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::INFO);
  infrared_ranging::InfraredRanging infrared_ranging_app;
  while (infrared_ranging_app.Run());
  return 0;
}
