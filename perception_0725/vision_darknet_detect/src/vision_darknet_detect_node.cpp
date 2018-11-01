
#include "vision_darknet_detect.h"

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "vision_darknet_detect");

    Yolo3DetectorNode app;

    app.Run();

    return 0;
}
