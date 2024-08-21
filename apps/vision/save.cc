#include "apps/vision/include/vision.h"

int main(int argc, char const *argv[])
{
  vision::Vision vision_app;
  while(vision_app.SaveVideo());
  return 0;
}