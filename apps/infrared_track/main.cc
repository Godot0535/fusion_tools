#include "apps/infrared_track/include/infrared_track.h"

int main(int argc, char const *argv[]) {
  infrared_track::InfraredTrack infrared_track_app;
  while (infrared_track_app.Run());
  return 0;
}
