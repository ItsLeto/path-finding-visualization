#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

// Override base class with your custom functionality
class PathFinding : public olc::PixelGameEngine {
  public:
    PathFinding() {
        sAppName.assign("PathFinding");
    }

  public:
    bool OnUserCreate() override {

        return true;
    }

    bool OnUserUpdate(float elapsed_time) override {

        return true;
    }
};

int main() {
    PathFinding demo;
    if (demo.Construct(256, 240, 4, 4)) {
        demo.Start();
    }
    return 0;
}
