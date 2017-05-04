/**
 * @file   raspicam_raw_parser.cpp
 * @author Jose Nogueira, josenogueira@biosurfit.com
 * @date   May 2017
 * @brief  Raspistill raw image parser
 *
 * (extensive explanation)
 */

/**
 * Include files
 */
#include "raspicam_raw_parser.h"

#include <cstring>
#include <iostream>


/**
 * Definitions
 */
#define BCRM_HEADER_SIZE           0x8000
#define BCRM_HEADER_ADDRESS_START  0x00B0

// Values taken from https://github.com/raspberrypi/userland/blob/master/interface/vctypes/vc_image_types.h
#define BRCM_FORMAT_BAYER              33
#define BRCM_BAYER_RAW10                3


namespace raspicam
{

/**
 *
 */
Type_BRCMHeader parse_brcm_header(uint8_t*      data,
                                  const size_t  length)
{
    // Check data length size
    if (length < BCRM_HEADER_SIZE)
    {
        std::cerr << "Data provided does not have the necessary length for a Broadcom Header.\n";
        return Type_BRCMHeader();
    }

    // Check for broadcom header tag
    if ( memcmp(data, "BRCM", 4) != 0 )
    {
        std::cerr << "Data provided does not check for Broadcom Header tag.\n";
        return Type_BRCMHeader();
    }

    // Copy header from data
    Type_BRCMHeader image_header;
    memcpy(&image_header, data+BCRM_HEADER_ADDRESS_START, sizeof(Type_BRCMHeader));

    // Verify Broadcom image format
    if ( (image_header.format       != BRCM_FORMAT_BAYER) ||
         (image_header.bayer_format != BRCM_BAYER_RAW10 )   )
    {
        memset(&image_header, 0, sizeof(image_header));
    }

    return image_header;
}


/**
 *
 */
void depad_image_raw(uint8_t*      data_in   ,
                     const size_t  length_in ,
                     uint16_t**    data_out  ,
                     size_t&       length_out)
{
    // Parse Header
    Type_BRCMHeader image_header;
                    image_header = parse_brcm_header(data_in, length_in);
                    length_out   = 0;

    // Verify that image_header was correctly parsed
    if (image_header.width == 0) return;

    // (from dcraw https://github.com/6by9/RPiTest)
    const unsigned short image_width  = image_header.width;
    const unsigned short image_height = image_header.height;
    // Define depad algorithm variables
    // (x * 5) + 3 >> 2 is the same as ceil(x*5.0/4.0) but faster (10bit pixel and 8bit packed format)
    //   + align 32 ----> 0x1F is 31
    const unsigned short depad_stride = ((((((image_header.width + image_header.padding_right)*5)+3)>>2) + 0x1F)&(~0x1F));
    // const unsigned short depad_height = image_height + image_header.padding_down;
    // const unsigned short depad_width  = (length_in - BCRM_HEADER_SIZE) / (depad_height);
    // const          short depad_order  = 0x4d4d;

    // Reserve memory for output image
    length_out  = image_width * image_height;
    (*data_out) = (uint16_t*) malloc(length_out);

    // Unpack 10bit values into 16bit data
    unsigned char* data_cursor = data_in + BCRM_HEADER_SIZE;
    for (size_t row = 0; row < image_height; ++row, data_cursor += depad_stride)
    {
        unsigned char* dp = data_cursor; // Set of 4 pixels in 5 bytes
        for (size_t col = 0; col < image_width; col += 4, dp += 5)
        {
            for (size_t idx = 0; idx < 4; ++idx)
            {
                (*data_out)[(row)*image_width+(col)+idx] = (uint16_t(dp[idx]) << 2) | (uint16_t(dp[4]) >> (idx << 1) & 3);
            }
        }
    }
}

}
