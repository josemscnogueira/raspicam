/**
 * @file   raspicam_raw_parser.h
 * @author Jose Nogueira, josenogueira@biosurfit.com
 * @date   May 2017
 * @brief  Raspistill raw image parser
 *
 * (extensive explanation)
 */

#ifndef _RASPICAM_RAW_PARSER_H_
#define _RASPICAM_RAW_PARSER_H_

/**
 * Include files
 */
#include <cstdint>
#include <stdio.h>


/**
 * Enums
 */
typedef enum Enum_BRCM_BayerOrder
{
    RGGB = 0,
    GBRG = 1,
    BGGR = 2,
    GRGB = 3,
} Enum_BRCM_BayerOrder;


namespace raspicam
{
    /**
     * Typedefs
     */
    typedef struct Type_BRCMHeader
    {
        uint8_t   name[32];
        uint16_t  width;
        uint16_t  height;
        uint16_t  padding_right;
        uint16_t  padding_down;
        uint32_t  dummy[6];
        uint16_t  transform;
        uint16_t  format;
        uint8_t   bayer_order;
        uint8_t   bayer_format;
    } Type_BRCMHeader;

    Type_BRCMHeader parse_brcm_header(uint8_t*      data  ,
                                      const size_t  length);

    void            depad_image_raw(  uint8_t*      data_in   ,
                                      const size_t  length_in ,
                                      uint16_t**    data_out  ,
                                      size_t&       length_out);



} // End of namespace raspicam


#endif
