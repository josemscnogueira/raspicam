#include <iostream>
#include <fstream>
#include <cstdlib>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"


#include "raspitimer.h"
#include "raspicam_still.h"
#include "raspicam_raw_parser.h"




int main (int argc, char *argv[])
{
    raspicam::RaspiCam_Still cam;
                            //  cam.setImageEncoding(raspicam::RASPICAM_ENCODING_BMP);
                             cam.setupRawMode(5);
                             cam.open();
    size_t          length = cam.getImageBufferSize(); // Header + Image Data + Padding
    int             rc_b   = 0;
    unsigned char   data[length];
    unsigned char*  data_raw = NULL;

    std::cout << "[DEBUG] Data length is " << length             << " bytes\n";
    std::cout << "[DEBUG] Data length is " << length/1024        << "Kbytes\n";
    std::cout << "[DEBUG] Data length is " << length/(1024*1024) << "Mbytes\n";

    for (size_t idx = 0; idx < 10; ++idx)
    {
        TIMER_usecCtx_t rtimer;
        TIMER_usecStart(&rtimer);

        rc_b = cam.captureFrameRaw(data, length, &data_raw);

        std::cout << "Took frame in " << TIMER_usecElapsedUs(&rtimer) / 1000.0 << "ms\n";

        if (rc_b < 0)
        {
            std::cerr << "Error in capture\n";;
            return -1;
        }
    }

    uint16_t* data_depadded = NULL;

    if (data_raw != NULL)
    {
        cv::Mat image_raw = raspicam::load_bcrm_image_raw(data_raw, rc_b);

        // cv::imshow("Final image", image_raw);
        cv::imwrite("image_raw.jpg", image_raw);
    }

    free(data_depadded);

    return 0;
}
