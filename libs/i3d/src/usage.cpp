#include "usage.h"
#include "logger.h"

void usage::prompt(const int& code)
{
    switch (code) {
    case (ABOUT):
        LOG(INFO) << "-- 3DINTACT is currently unstable and should only be "
                     "used for academic purposes!";
        LOG(INFO) << "-- press ESC to exit";
        break;
    case (USAGE):
        LOG(INFO) << "INSTRUCTIONS:" << std::endl;
        LOG(INFO) << "-- press ENTER to take images" << std::endl;
        LOG(INFO) << "-- to at least 20 chessboard images" << std::endl;
        LOG(INFO) << "-- press ESCAPE to exit capture mode" << std::endl;
        LOG(INFO) << "-- once calibration is done, see output directory for "
                     "calibration file"
                  << std::endl;
        break;
    case (CALIBRATING):
        LOG(INFO) << "-- computing calibration parameters" << std::endl;
        break;
    case (MORE_IMAGES_REQUIRED):
        LOG(INFO) << "-- take more chessboard images" << std::endl;
        break;
    case (SAVING_PARAMETERS):
        LOG(INFO) << "-- saving calibration parameters to disk" << std::endl;
        break;

    case (LOADING_CALIBRATION_PARAMETERS):
        LOG(INFO) << "-- loading calibration parameters from disk" << std::endl;
        break;
    case (FINDING_ARUCO_MARKERS):
        LOG(INFO) << "-- searching for aruco markers" << std::endl;
        break;
    default:
        LOG(INFO) << "-- well now, wasn't expecting that one bit" << std::endl;
        break;
    }
}
