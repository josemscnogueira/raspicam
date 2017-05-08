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
                             cam.setImageEncoding(raspicam::RASPICAM_ENCODING_BMP);
                             cam.setRawMode(true);
                             cam.setSensorMode(5);
                             cam.setResolution(32,18);
                             cam.open();
    size_t          length = cam.getImageBufferSize(); // Header + Image Data + Padding
    size_t          offset = 0;
    int             rc_b   = 0;
    unsigned char   data[length];

    std::cout << "[DEBUG] Data length is " << length             << " bytes\n";
    std::cout << "[DEBUG] Data length is " << length/1024        << "Kbytes\n";
    std::cout << "[DEBUG] Data length is " << length/(1024*1024) << "Mbytes\n";

    for (size_t idx = 0; idx < 10; ++idx)
    {
        TIMER_usecCtx_t rtimer;
        TIMER_usecStart(&rtimer);

        rc_b = cam.captureFrame(data, length, offset);

        std::cout << "Took frame in " << TIMER_usecElapsedUs(&rtimer) / 1000.0 << "ms\n";

        if (rc_b < 0)
        {
            std::cerr << "Error in capture\n";;
            return -1;
        }

        std::cout << "[DEBUG] Data length is " << (rc_b - offset) << "  bytes\n";
    }

    std::cout << "Saving picture.rawpacked\n";
    std::ofstream file("picture.bmp", std::ios::binary);
                  file.write((char*)(data), rc_b);

    uint16_t* data_depadded = NULL;
    size_t    data_depadded_size = 0;

    std::cout << "offset = " << offset << std::endl;
    std::cout << "rc_b   = " << rc_b   << std::endl;
    std::cout << "rc_b - offset = " << rc_b - offset   << std::endl;

    cv::Mat image_raw = raspicam::load_bcrm_image_raw(data + offset, rc_b - offset);

    cv::imshow("Final image", image_raw);
    cv::imwrite("image_raw.jpg", image_raw);

    // std::cout << "data_depadded_size = " << data_depadded_size << "\n";
    // std::cout << "Saving picture.raw (" << data_depadded_size*sizeof(uint16_t) << " bytes)\n";
    // std::ofstream file2("picture.raw", std::ios::binary);
    //               file2.write((char*)data_depadded, data_depadded_size*sizeof(uint16_t));

    free(data_depadded);

    return 0;
}
