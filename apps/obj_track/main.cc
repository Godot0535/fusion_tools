#include "apps/obj_track/include/obj_track.h"

int main(int argc, char const *argv[]) {
  obj_track::ObjTrack obj_track_app;
  while (obj_track_app.Run());
  return 0;
}
