/**********************************************************
 Software developed by AVA ( Ava Group of the University of Cordoba, ava  at uco dot es)
 Main author Rafael Munoz Salinas (rmsalinas at uco dot es)
 This software is released under BSD license as expressed below
-------------------------------------------------------------------
Copyright (c) 2013, AVA ( Ava Group University of Cordoba, ava  at uco dot es)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:

   This product includes software developed by the Ava group of the University of Cordoba.

4. Neither the name of the University nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AVA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL AVA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************/

#include "private_still_impl.h"

#include "interfaces/mmal/mmal_buffer.h"
#include "interfaces/mmal/util/mmal_default_components.h"
#include "interfaces/mmal/util/mmal_util.h"
#include "interfaces/mmal/util/mmal_util_params.h"

#include <fstream>
#include <iostream>
#include <semaphore.h>

#define PREVIEW_DEFAULT_WIDTH   640
#define PREVIEW_DEFAULT_HEIGHT  480


namespace raspicam
{
namespace _private
{

/**
 * Typedefs
 */
typedef struct
{
    Private_Impl_Still*  handle_camera;
    MMAL_POOL_T*         encoder_pool;
    imageTakenCallback   callback_image;
    sem_t*               mutex;
    uchar*               data;
    uint                 buffer_position;
    uint                 offset_starting;
    uint                 offset;
    uint                 length;
} RASPICAM_USERDATA;


/**
 * Auxiliary functions
 */
/**
 * Convert a MMAL status return value to a simple boolean of success
 * ALso displays a fault if code is not success
 *
 * @param status The error code to convert
 * @return 0 if status is success, 1 otherwise
 */
int mmal_status_to_int(MMAL_STATUS_T status)
{
    if (status == MMAL_SUCCESS) return 0;
    else
    {
        switch (status)
        {
            case MMAL_ENOMEM   : std::cerr << "Out of memory\n"                                          ; break;
            case MMAL_ENOSPC   : std::cerr << "Out of resources (other than memory)\n"                   ; break;
            case MMAL_EINVAL   : std::cerr << "Argument is invalid\n"                                    ; break;
            case MMAL_ENOSYS   : std::cerr << "Function not implemented\n"                               ; break;
            case MMAL_ENOENT   : std::cerr << "No such file or directory\n"                              ; break;
            case MMAL_ENXIO    : std::cerr << "No such device or address\n"                              ; break;
            case MMAL_EIO      : std::cerr << "I/O error\n"                                              ; break;
            case MMAL_ESPIPE   : std::cerr << "Illegal seek\n"                                           ; break;
            case MMAL_ECORRUPT : std::cerr << "Data is corrupt \attention FIXME: not POSIX\n"            ; break;
            case MMAL_ENOTREADY: std::cerr << "Component is not ready \attention FIXME: not POSIX\n"     ; break;
            case MMAL_ECONFIG  : std::cerr << "Component is not configured \attention FIXME: not POSIX\n"; break;
            case MMAL_EISCONN  : std::cerr << "Port is already connected\n"                              ; break;
            case MMAL_ENOTCONN : std::cerr << "Port is disconnected\n"                                   ; break;
            case MMAL_EAGAIN   : std::cerr << "Resource temporarily unavailable. Try again later\n"      ; break;
            case MMAL_EFAULT   : std::cerr << "Bad address\n"                                            ; break;
            default            : std::cerr << "Unknown status error\n"                                   ; break;
        }

        return 1;
    }
}


/**
 * Camera callbacks
 */
static void camera_control_callback(MMAL_PORT_T*          port  ,
                                    MMAL_BUFFER_HEADER_T* buffer)
{
    if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED)
    {
        MMAL_EVENT_PARAMETER_CHANGED_T* param = (MMAL_EVENT_PARAMETER_CHANGED_T*) buffer->data;

        switch (param->hdr.id)
        {
            case MMAL_PARAMETER_CAMERA_SETTINGS:
            {
                MMAL_PARAMETER_CAMERA_SETTINGS_T* settings = (MMAL_PARAMETER_CAMERA_SETTINGS_T*) param;

                std::cerr << "Exposure now "   << settings->exposure
                          << ", analog gain "  << settings->analog_gain.num  << "/" << settings->analog_gain.den
                          << ", digital gain " << settings->digital_gain.num << "/" << settings->digital_gain.den << '\n';

                std::cerr << "AWB R=" << settings->awb_red_gain.num   << "/" << settings->awb_red_gain.den
                          <<    " B=" << settings->awb_blue_gain.num  << "/" << settings->awb_blue_gain.den << '\n';
                break;
            }
        }
    }
    else if (buffer->cmd == MMAL_EVENT_ERROR)
    {
        std::cerr << "No data received from sensor. Check all connections, including the Sunny one on the camera board.\n";
    }
    else
    {
        std::cerr << "Received unexpected camera control callback event, " << buffer->cmd << '\n';
    }

    mmal_buffer_header_release(buffer);
}


/**
 * Camera callbacks
 */
static void encoder_buffer_callback(MMAL_PORT_T*          port  ,
                                    MMAL_BUFFER_HEADER_T* buffer)
{
    bool               flag_complete = false;
    RASPICAM_USERDATA* userdata      = (RASPICAM_USERDATA*) port->userdata;

    if ( (userdata                != NULL) &&
         (userdata->handle_camera != NULL)   )
    {
        if (buffer->length && userdata->data)
        {
            if (userdata->length < buffer->length)
            {
                flag_complete = true;
                std::cerr << "Unable to write buffer user memory - aborting\n";
            }
            else
            {
                // Get data from buffer
                mmal_buffer_header_mem_lock(buffer);
                memcpy(userdata->data, buffer->data, buffer->length);
                mmal_buffer_header_mem_unlock(buffer);
            }
        }

        // Update flag of completion
        if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END           |
                             MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED) )
        {
            flag_complete = true;
        }
    }
    else
    {
        std::cerr << "Received a encoder buffer callback with no userdata\n";
    }

    mmal_buffer_header_release(buffer);

    // Send new buffer to port
    if (port->is_enabled)
    {
        MMAL_STATUS_T          status     = MMAL_SUCCESS;
        MMAL_BUFFER_HEADER_T*  new_buffer = mmal_queue_get(userdata->encoder_pool->queue);

        if (new_buffer)
        {
            status = mmal_port_send_buffer(port, new_buffer);
        }
        if (!new_buffer || (status != MMAL_SUCCESS) )
        {
            std::cerr << "Unable to return a buffer to the encoder port\n";
        }
    }

    if (flag_complete)
    {
        if (userdata->mutex == NULL)
        {
            userdata->callback_image(userdata->data                              ,
                                     userdata->offset_starting                   ,
                                     userdata->length - userdata->offset_starting);
        }
        else
        {
            sem_post(userdata->mutex);
        }
    }
}


/**************************************************************************************************
 *   Private_Impl_Still Code                                                                      *
 **************************************************************************************************/
const std::string Private_Impl_Still::API_NAME = "PRIVATE_IMPL_RaspiCamStill";


/**
 *
 */
Private_Impl_Still::Private_Impl_Still(void)
{
    // Camera initialized flag
    _is_initialized      = false;
    _camera_name         = "";
    _raw_mode            = false;

    // MMAL Components pointers
    _camera              = NULL;
    _encoder             = NULL;

    // MMAL Connection pointers
    _encoder_connection  = NULL;

    // MMAL Pool pointers
    _encoder_pool        = NULL;

    // MMAL ports pointers
    _port_camera         = NULL;
    _port_encoder_input  = NULL;
    _port_encoder_output = NULL;

    this->setDefaults();
}


/**
 *
 */
void Private_Impl_Still::getSensorInfo(void)
{
    MMAL_COMPONENT_T*   camera_info;
    MMAL_STATUS_T       status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);

    if (status == MMAL_SUCCESS)
    {
        // Deliberately undersize to check firmware veresion
        MMAL_PARAMETER_CAMERA_INFO_T camera_info_param;
                                     camera_info_param.hdr.id   = MMAL_PARAMETER_CAMERA_INFO;
                                     camera_info_param.hdr.size = sizeof(camera_info_param)-4;

        status = mmal_port_parameter_get(camera_info->control, &camera_info_param.hdr);

        // Checks with newer firmware versions
        if (status != MMAL_SUCCESS)
        {
            // Running on newer firmware
            camera_info_param.hdr.size = sizeof(camera_info_param);

            status = mmal_port_parameter_get(camera_info->control, &camera_info_param.hdr);
            if ( (status == MMAL_SUCCESS) && (camera_info_param.num_cameras > uint(_camera_idx)) )
            {
                // Restore to maxiumum resolution
                if ( (_width  == 0) ||
                     (_height == 0)   )
                {
                    _width  = camera_info_param.cameras[_camera_idx].max_width;
                    _height = camera_info_param.cameras[_camera_idx].max_height;
                }

                // Restore camera name
                _camera_name = camera_info_param.cameras[_camera_idx].camera_name;
                _camera_name.resize(MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);

                std::cout << "Recognized sensor " << _camera_name << '\n';
                std::cout << "    -> width  ⁼ "   << _width       << '\n';
                std::cout << "    -> height ⁼ "   << _height      << '\n';
            }
        }

        mmal_component_destroy(camera_info);
    }

    // Failed to get sensor information, assume Sony IMX219
    if ( (_width  == 0) ||
         (_height == 0)   )
    {
        _width  = 3280;
        _height = 2464;
    }
}


/**
 *
 */
void Private_Impl_Still::setDefaults(void)
{
    _width                 = 0;
    _height                = 0;
    _rotation              = 0;
    _brightness            = 50;
    _quality               = 85;
    _iso                   = 100;
    _sharpness             = 0;
    _contrast              = 0;
    _saturation            = 0;
    _video_stabilization   = 0;

    _sensor_mode           = 0;
    _camera_idx            = 0;

    _exposure_mode         = RASPICAM_EXPOSURE_AUTO;
    _exposure_compensation = 0;
    _exposure_metering     = RASPICAM_METERING_AVERAGE;
    _shutter_speed         = 0;

    _awb                   = RASPICAM_AWB_AUTO;
    _awb_gains_r           = 1.0;
    _awb_gains_b           = 1.0;

    _flip_horizontal       = false;
    _flip_vertical         = false;

    _encoding              = RASPICAM_ENCODING_BMP;
	_image_fx              = RASPICAM_IMAGE_EFFECT_NONE;
    _drc                   = MMAL_PARAMETER_DRC_STRENGTH_OFF;

    // Get sensor information (resolution and name)
    this->getSensorInfo();

    _settings_changed      = true;
}


/**
 *
 */
int Private_Impl_Still::commitParameters(void)
{
    if (!_settings_changed) return MMAL_SUCCESS;

    int status  = commitSaturation();
        status += commitSharpness();
        status += commitContrast();
        status += commitBrightness();
        status += commitISO();
        status += commitVideoStabilization();
        status += commitExposureCompensation();
        status += commitExposureMode();
        status += commitExposureMetering();
        status += commitAWB();
        status += commitAWBGains();
        status += commitImageFX();
        status += commitColorFX();
        status += commitRotation();
        status += commitFlips();
        status += commitROI();
        status += commitShutterSpeed();
        status += commitDRC();

    // Set encoder encoding
    if (_port_encoder_output != NULL)
    {
        _port_encoder_output->format->encoding = this->convertEncoding(_encoding);

        status += mmal_status_to_int(mmal_port_format_commit(_port_encoder_output));
    }

    _settings_changed = false;

    return status;
}


/**
 *
 */
MMAL_STATUS_T Private_Impl_Still::connectPorts(MMAL_PORT_T*        port_output,
                                               MMAL_PORT_T*        port_input ,
                                               MMAL_CONNECTION_T** connection )
{
    MMAL_STATUS_T status = mmal_connection_create(connection                                 ,
                                                  port_output                                ,
                                                  port_input                                 ,
                                                  (MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT) );
    if (status == MMAL_SUCCESS)
    {
        status = mmal_connection_enable(*connection);

        if (status != MMAL_SUCCESS) mmal_connection_destroy(*connection);
    }

    return status;
}


/**
 *
 */
int Private_Impl_Still::createCamera(void)
{
    std::cout << "Creating camera..." << '\n';

    // Verify that _camera == NULL (not initialized)
    if (_camera)
    {
        std::cerr << API_NAME << ": Camera seems to be already created.\n";
        this->destroyCamera();
        return -1;
    }

    // Create camera component
    MMAL_STATUS_T status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &_camera);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to create camera component.\n";
        this->destroyCamera();
        return -1;
    }

    // Select camera
    MMAL_PARAMETER_INT32_T camera_idx = { {MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_idx)}, _camera_idx};

    status = mmal_port_parameter_set(_camera->control, &camera_idx.hdr);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to select camera.\n";
        this->destroyCamera();
        return -1;
    }

    // Verify output ports
    if (_camera->output_num < 3)
    {
        std::cerr << API_NAME << ": Camera doesn't have enough output ports.\n";
        this->destroyCamera();
        return -1;
    }

    // Configure sensor mode
    status = mmal_port_parameter_set_uint32(_camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, _sensor_mode);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Could not set sensor mode.\n";
        this->destroyCamera();
        return -1;
    }

    // Define camera port
    _port_camera               = _camera->output[MMAL_CAMERA_CAPTURE_PORT];
    MMAL_PORT_T* port_preview  = _camera->output[MMAL_CAMERA_PREVIEW_PORT];
    MMAL_PORT_T* port_video    = _camera->output[MMAL_CAMERA_VIDEO_PORT];

    // Enable the camera, and tell it its control callback function
    status = mmal_port_enable(_camera->control, camera_control_callback);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Unable to enable control port.\n";
        this->destroyCamera();
        return -1;
    }

    // Setup the camera configuration
    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cfg_camera =
        {
            {MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cfg_camera)},
            .max_stills_w                          = _width,
            .max_stills_h                          = _height,
            .stills_yuv422                         = 0,
            .one_shot_stills                       = 1,
            .max_preview_video_w                   = PREVIEW_DEFAULT_WIDTH,
            .max_preview_video_h                   = PREVIEW_DEFAULT_HEIGHT,
            .num_preview_video_frames              = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume                   = 0,
            .use_stc_timestamp                     = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };

        status = mmal_port_parameter_set(_camera->control, &cfg_camera.hdr);
        if (status != MMAL_SUCCESS)
        {
            std::cerr << API_NAME << ": Failed to set camera still parameters.\n";
            this->destroyCamera();
            return -1;
        }
    }

    // Commit configuration parameters
    if (this->commitParameters() != 0)
    {
        std::cerr << API_NAME << ": Failed to commit camera settings.\n";
        this->destroyCamera();
        return -1;
    }

    // Set preview port format (?? FIXME necesseary ??)
    MMAL_ES_FORMAT_T* format = port_preview->format;
                      format->encoding                 = MMAL_ENCODING_OPAQUE;
                      format->encoding_variant         = MMAL_ENCODING_I420;
                      format->es->video.width          = VCOS_ALIGN_UP(PREVIEW_DEFAULT_WIDTH , 32);
                      format->es->video.height         = VCOS_ALIGN_UP(PREVIEW_DEFAULT_HEIGHT, 16);
                      format->es->video.crop.x         = 0;
                      format->es->video.crop.y         = 0;
                      format->es->video.crop.width     = PREVIEW_DEFAULT_WIDTH;
                      format->es->video.crop.height    = PREVIEW_DEFAULT_HEIGHT;
                      format->es->video.frame_rate.num = PREVIEW_FRAME_RATE_NUM;
                      format->es->video.frame_rate.den = PREVIEW_FRAME_RATE_DEN;
    status = mmal_port_format_commit(port_preview);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Camera preview format couldn't be set.\n";
        this->destroyCamera();
        return -1;
    }

    // Set the same format on the video port (which we don't use here)
    mmal_format_full_copy(port_video->format, format);
    status = mmal_port_format_commit(port_video);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Camera video format couldn't be set.\n";
        this->destroyCamera();
        return -1;
    }

    // Ensure there are enough buffers to avoid dropping frames
    if (port_video->buffer_num > VIDEO_OUTPUT_BUFFERS_NUM)
    {
        port_video->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
    }

    // Set format for camera port
    format = _port_camera->format;
    format->encoding                 = MMAL_ENCODING_OPAQUE;
    format->es->video.width          = VCOS_ALIGN_UP(_width, 32);
    format->es->video.height         = VCOS_ALIGN_UP(_height,16);
    format->es->video.crop.x         = 0;
    format->es->video.crop.y         = 0;
    format->es->video.crop.width     = _width;
    format->es->video.crop.height    = _height;
    format->es->video.frame_rate.num = STILLS_FRAME_RATE_NUM;
    format->es->video.frame_rate.den = STILLS_FRAME_RATE_DEN;

    status = mmal_port_format_commit(_port_camera);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Camera still port format couldn't be set.\n";
        this->destroyCamera();
        return -1;
    }

    /* Ensure there are enough buffers to avoid dropping frames */
    if (_port_camera->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    {
        _port_camera->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
    }

    // Enable camera component
    status = mmal_component_enable(_camera);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Camera component couldn't be enabled.\n";
        this->destroyCamera();
        return -1;
    }

    std::cout << "Creating camera...DONE" << '\n';

    return 0;
}


/**
 *
 */
void Private_Impl_Still::destroyCamera(void)
{
    if (_camera) mmal_component_destroy(_camera);
    _camera = NULL;
}


/**
 *
 */
int Private_Impl_Still::createEncoder(void)
{
    std::cout << "Creating encoder..." << '\n';

    // Create default image encoder
    MMAL_STATUS_T status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &_encoder);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Could not create encoder component.\n";
        this->destroyEncoder();
        return -1;
    }

    // Verify encoder input and output ports
    if ( (_encoder->input_num < 1) || (_encoder->output_num < 1) )
    {
        std::cerr << API_NAME << ": Encoder does not have input/output ports.\n";
        this->destroyEncoder();
        return -1;
    }

    // Copy encoder port references
    _port_encoder_input  = _encoder->input[0];
    _port_encoder_output = _encoder->output[0];

    // Force encoder ports to have the same format
    mmal_format_copy(_port_encoder_output->format, _port_encoder_input->format);

    // Specify output format
    _port_encoder_output->format->encoding = this->convertEncoding(_encoding);

    // Set buffer size
    _port_encoder_output->buffer_size = _port_encoder_output->buffer_size_recommended;
    if (_port_encoder_output->buffer_size < _port_encoder_output->buffer_size_min)
        _port_encoder_output->buffer_size = _port_encoder_output->buffer_size_min;

    _port_encoder_output->buffer_num = _port_encoder_output->buffer_num_recommended;
    if (_port_encoder_output->buffer_num < _port_encoder_output->buffer_num_min)
        _port_encoder_output->buffer_num = _port_encoder_output->buffer_num_min;

    // Commit the port changes to the output port
    status = mmal_port_format_commit(_port_encoder_output);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Unable to set format on encoder output port.\n";
        this->destroyEncoder();
        return -1;
    }

    // Set the JPEG quality level
    status = mmal_port_parameter_set_uint32(_port_encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, _quality);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Unable to set JPEG quality.\n";
        this->destroyEncoder();
        return -1;
    }

    // Set the JPEG restart interval
    status = mmal_port_parameter_set_uint32(_port_encoder_output, MMAL_PARAMETER_JPEG_RESTART_INTERVAL, 0);
    if (0 && status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Unable to set JPEG restart interval.\n";
        this->destroyEncoder();
        return -1;
    }

    // Set up any required thumbnail
    {
        MMAL_PARAMETER_THUMBNAIL_CONFIG_T param_thumb = { {MMAL_PARAMETER_THUMBNAIL_CONFIGURATION, sizeof(param_thumb)},
                                                          .enable  =  1,
                                                          .width   = 64,
                                                          .height  = 48,
                                                          .quality = 35};

        status = mmal_port_parameter_set(_encoder->control, &param_thumb.hdr);
        if (status != MMAL_SUCCESS)
        {
            std::cerr << API_NAME << ": Thumbnail configuration coult not be set.\n";
            this->destroyEncoder();
            return -1;
        }
    }

    // Enable encoder component
    status = mmal_component_enable(_encoder);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Unable to enable video encoder component.\n";
        this->destroyEncoder();
        return -1;
    }

    // Create pool of buffer headers for the output port to consume
    _encoder_pool = mmal_port_pool_create(_port_encoder_output, _port_encoder_output->buffer_num, _port_encoder_output->buffer_size);
    if (!_encoder_pool)
    {
        std::cerr << API_NAME << ": Failed to create buffer header pool for encoder output port.\n";
        this->destroyEncoder();
        return -1;
    }

    std::cout << "Creating encoder...DONE" << '\n';

    return 0;
}


/**
 *
 */
void Private_Impl_Still::destroyEncoder(void)
{
    if (_encoder) mmal_component_destroy(_encoder);
    _encoder = NULL;
}


/**
 *
 */
int Private_Impl_Still::initialize(void)
{
    if (_is_initialized) return 0;

    // Create camera MMAL structure
    int status = this->createCamera();
    if (status != 0)
    {
        std::cerr << API_NAME << ": Failed to create camera component.\n";
        return -1;
    }

    // Create encoder MMAL structure
    status = this->createEncoder();
    if (status != 0)
    {
        std::cerr << API_NAME << ": Failed to create encoder component.\n";
        this->destroyCamera();
        return -1;
    }

    // Refresh port referenecs
    _port_camera         = _camera->output[MMAL_CAMERA_CAPTURE_PORT];
    _port_encoder_input  = _encoder->input[0];
    _port_encoder_output = _encoder->output[0];

    // Connect encoder ports
    status = this->connectPorts(_port_camera, _port_encoder_input, &_encoder_connection);
    if (status != 0)
    {
        std::cerr << API_NAME << ": Failed to connect camera port to encoder input.\n";
        this->release();
        return -1;
    }

    _is_initialized = true;
    return 0;
}


/**
 *
 */
int Private_Impl_Still::release(void)
{
    if (!_is_initialized) return 0;

    // Disable all our ports that are not handled by connections
    if (_port_encoder_output && _port_encoder_output->is_enabled) mmal_port_disable(_port_encoder_output);
    std::cout << "release() | camera port disabled\n";

    // Destroy encoder connection
    if (_encoder_connection) mmal_connection_destroy(_encoder_connection);
    std::cout << "release() | encoder connection destroyed\n";

    if (_encoder) mmal_component_disable(_encoder);
    std::cout << "release() | encoder component disabled\n";

    if (_camera) mmal_component_disable(_camera);
    std::cout << "release() | camera component disabled\n";

    // Get rid of any port buffers first
    if (_encoder_pool) mmal_port_pool_destroy(_encoder->output[0], _encoder_pool);
    std::cout << "release() | encoder pool destroyed\n";

    if (_encoder) mmal_component_destroy(_encoder);
    _encoder = NULL;
    std::cout << "release() | encoder component destroyed\n";

    if (_camera) mmal_component_destroy(_camera);
    _camera = NULL;
    std::cout << "release() | camera component destroyed\n";

    _is_initialized = false;
    return 0;
}


/**
 *
 */
bool Private_Impl_Still::takePicture(uchar* data_preallocated, uint length)
{
    int status = this->initialize();

    if (status) return status;

    // Init semaphore
    sem_t     mutex;
    sem_init(&mutex, 0, 0);

    RASPICAM_USERDATA* userdata = new RASPICAM_USERDATA();
                       userdata->handle_camera   = this;
                       userdata->encoder_pool    = _encoder_pool;
                       userdata->mutex           = &mutex;
                       userdata->data            = data_preallocated;
                       userdata->buffer_position = 0;
                       userdata->offset          = 0;
                       userdata->offset_starting = 0;
                       userdata->length          = length;
                       userdata->callback_image  = NULL;

    _port_encoder_output->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;

    if ( (status = this->startCapture()) != 0 )
    {
        delete userdata;
        return false;
    }

    // Semaphore barrier
    sem_wait(   &mutex);
    sem_destroy(&mutex);

    this->stopCapture();
    delete userdata;
    return true;
}


/**
 *
 */
size_t Private_Impl_Still::getImageBufferSize(void) const
{
    // For bmp images only!!! oversize the buffer so to fit BMP images
    return _width * _height * 3 + 54;
}


/**
 *
 */
int Private_Impl_Still::startCapture(imageTakenCallback  callback_user    ,
                                     uchar*              data_preallocated,
                                     uint                offset           ,
                                     uint                length           )
{
    RASPICAM_USERDATA* userdata = new RASPICAM_USERDATA();
                       userdata->handle_camera   = this;
                       userdata->encoder_pool    = _encoder_pool;
                       userdata->mutex           = NULL;
                       userdata->data            = data_preallocated;
                       userdata->buffer_position = 0;
                       userdata->offset          = offset;
                       userdata->offset_starting = offset;
                       userdata->length          = length;
                       userdata->callback_image  = callback_user;

    _port_encoder_output->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;

    return this->startCapture();
}


/**
 *
 */
int Private_Impl_Still::startCapture(void)
{
    // Veirfy if camera port is active
    if (!_port_camera)
    {
        std::cerr << API_NAME << ": Camera power was not active for capture.\n";
        return -1;
    }

    // Enable raw mode capture if specified
    if (_raw_mode)
    {
        if (mmal_port_parameter_set_boolean(_port_camera, MMAL_PARAMETER_ENABLE_RAW_CAPTURE, 1) != MMAL_SUCCESS)
        {
            std::cerr << API_NAME << ": RAW was requested, but failed to enable.\n";
            return -1;
        }
    }

    // There is a possibility that shutter needs to be set each loop. (raspistill code)
    if (mmal_port_parameter_set_uint32(_camera->control, MMAL_PARAMETER_SHUTTER_SPEED, _shutter_speed) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Unable to set shutter speed.\n";
        return -1;
    }

    // If the parameters were changed and this function wasn't called, it will be called here
    // However if the parameters weren't changed, the function won't do anything - it will return right away
    this->commitParameters();

    if (_port_encoder_output->is_enabled)
    {
        std::cerr << API_NAME << ": Could not enable encoder output port. Try waiting longer before attempting to take another picture.\n";
        return -1;
    }

    if (mmal_port_enable(_port_encoder_output, encoder_buffer_callback) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Could not enable encoder output port.\n";
        return -1;
    }

    const int queue_size = mmal_queue_length(_encoder_pool->queue);

    for (int idx = 0; idx < queue_size; ++idx )
    {
        MMAL_BUFFER_HEADER_T* buffer = mmal_queue_get(_encoder_pool->queue);

        if (!buffer)
        {
            std::cerr << API_NAME << ": Could not get buffer (#" << idx << ") from pool queue.\n";
        }

        if (mmal_port_send_buffer(_port_encoder_output, buffer) != MMAL_SUCCESS)
        {
            std::cerr << API_NAME << ": Could not send a buffer (#" << idx << ") to encoder output port.\n";
        }
    }

    if (mmal_port_parameter_set_boolean(_port_camera, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to start capture.\n";
        return -1;
    }

    return 0;
}


/**
 *
 */
void Private_Impl_Still::stopCapture(void)
{
    if (!_port_encoder_output->is_enabled) return;

    if (mmal_port_disable(_port_encoder_output))
    {
        delete (RASPICAM_USERDATA*) _port_encoder_output->userdata;
    }
}


/**
 *
 */
void Private_Impl_Still::setWidth(const uint width)
{
    _width            = width;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setHeight(const uint height)
{
    _height = height;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setCaptureSize(const uint width ,
                                        const uint height)
{
    this->setWidth( width );
    this->setHeight(height);
}


/**
 *
 */
void Private_Impl_Still::setBrightness(const uint brightness)
{
    if (brightness > 100) _brightness = 100;
    else                  _brightness = brightness;

    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setQuality(const uint quality)
{
    if (quality > 100) _quality = 100;
    else               _quality = quality;

    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setRotation(int rotation)
{
    while (rotation <    0) rotation += 360;
    if (   rotation >= 360) rotation  = rotation % 360;

    _rotation         = rotation;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setISO(const int iso)
{
    _iso              = iso;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setSharpness (const int sharpness)
{
    if      (sharpness < -100) _sharpness = -100;
    else if (sharpness >  100) _sharpness =  100;
    else                       _sharpness = sharpness;

    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setContrast(const int contrast)
{
    if      (contrast < -100) _contrast = -100;
    else if (contrast >  100) _contrast =  100;
    else                      _contrast = contrast;

    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setSaturation (const int saturation)
{
    if      (saturation < -100) _saturation  = -100;
    else if (saturation >  100) _saturation  =  100;
    else                        _saturation = saturation;

    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setEncoding(const RASPICAM_ENCODING encoding)
{
    _encoding         = encoding;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setExposure(const RASPICAM_EXPOSURE exposure)
{
    _exposure_mode    = exposure;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setAWB(const RASPICAM_AWB awb)
{
    _awb              = awb;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setImageEffect(const RASPICAM_IMAGE_EFFECT image_fx)
{
    _image_fx         = image_fx;
	_image_fx         = RASPICAM_IMAGE_EFFECT_NONE;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setMetering(const RASPICAM_METERING metering)
{
    _exposure_metering = metering;
    _settings_changed  = true;
}


/**
 *
 */
void Private_Impl_Still::setHorizontalFlip(const bool flip)
{
    _flip_horizontal  = flip;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setVerticalFlip(const bool flip)
{
    _flip_vertical    = flip;
    _settings_changed = true;
}


/**
 *
 */
int Private_Impl_Still::commitBrightness(void)
{
    if (!_camera) return 1;

    const int param = _brightness;
    if ( (param < 0) || (param > 100) )
    {
        std::cerr << API_NAME << "Invalid brightness value.\n";
        return 1;
    }

    MMAL_RATIONAL_T value  = {param, 100};
    MMAL_STATUS_T   status = mmal_port_parameter_set_rational(_camera->control, MMAL_PARAMETER_BRIGHTNESS, value);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set brightness parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitISO(void)
{
    if (!_camera) return 1;

    MMAL_STATUS_T status = mmal_port_parameter_set_uint32(_camera->control, MMAL_PARAMETER_ISO, _iso);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set ISO parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitVideoStabilization(void)
{
    if (!_camera) return 1;

    const bool    value  = _video_stabilization != 0 ? true : false;
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(_camera->control, MMAL_PARAMETER_VIDEO_STABILISATION, value);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set video stabilization parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitExposureCompensation(void)
{
    if (!_camera) return 1;

    MMAL_STATUS_T status = mmal_port_parameter_set_int32(_camera->control, MMAL_PARAMETER_EXPOSURE_COMP, _exposure_compensation);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set exposure compensation parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitExposureMode(void)
{
    if (!_camera) return 1;

    MMAL_PARAMETER_EXPOSUREMODE_T exp_mode = { {MMAL_PARAMETER_EXPOSURE_MODE,sizeof(exp_mode)}, this->convertExposure(_exposure_mode) };

    MMAL_STATUS_T status = mmal_port_parameter_set(_camera->control, &exp_mode.hdr);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set exposure mode parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitExposureMetering(void)
{
    if (!_camera) return 1;

    MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param  = { {MMAL_PARAMETER_EXP_METERING_MODE,sizeof(param)}, this->convertMetering(_exposure_metering)};
    MMAL_STATUS_T                         status = mmal_port_parameter_set(_camera->control, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set exposure metering parameter.\n";
    }

   return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitSharpness(void)
{
    if (!_camera) return 1;

    const int param = _sharpness;
    if ( (param < -100) || (param > 100) )
    {
        std::cerr << API_NAME << "Invalid saturation value.\n";
        return 1;
    }

    MMAL_RATIONAL_T value  = {param, 100};
    MMAL_STATUS_T   status = mmal_port_parameter_set_rational(_camera->control, MMAL_PARAMETER_SHARPNESS, value);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set sharpness parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitContrast(void)
{
    if (!_camera) return 1;

    const int param = _contrast;
    if ( (param < -100) || (param > 100) )
    {
        std::cerr << API_NAME << "Invalid saturation value.\n";
        return 1;
    }

    MMAL_RATIONAL_T value  = {param, 100};
    MMAL_STATUS_T   status = mmal_port_parameter_set_rational(_camera->control, MMAL_PARAMETER_CONTRAST, value);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set contrast parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitSaturation(void)
{
    if (!_camera) return 1;

    const int param = _saturation;
    if ( (param < -100) || (param > 100) )
    {
        std::cerr << API_NAME << "Invalid saturation value.\n";
        return 1;
    }

    MMAL_RATIONAL_T value  = {param, 100};
    MMAL_STATUS_T   status = mmal_port_parameter_set_rational(_camera->control, MMAL_PARAMETER_SATURATION, value);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set saturation parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitAWB(void)
{
    if (!_camera) return 1;

    MMAL_PARAMETER_AWBMODE_T param  = { {MMAL_PARAMETER_AWB_MODE,sizeof(param)}, this->convertAWB(_awb) };
    MMAL_STATUS_T            status = mmal_port_parameter_set(_camera->control, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set AWB mode parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitAWBGains(void)
{
    if (!_camera) return 1;

    MMAL_PARAMETER_AWB_GAINS_T param = { {MMAL_PARAMETER_CUSTOM_AWB_GAINS,sizeof(param)}, {0,0}, {0,0}};
                               param.r_gain.num = (unsigned int)(_awb_gains_r * 0x10000);
                               param.b_gain.num = (unsigned int)(_awb_gains_b * 0x10000);
                               param.r_gain.den = 0x10000;
                               param.b_gain.den = 0x10000;

    MMAL_STATUS_T              status = mmal_port_parameter_set(_camera->control, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set AWB gains parameters.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitImageFX(void)
{
    if (!_camera) return 1;

    MMAL_PARAMETER_IMAGEFX_T param  = { {MMAL_PARAMETER_IMAGE_EFFECT,sizeof(param)}, this->convertImageEffect(_image_fx) };
    MMAL_STATUS_T            status = mmal_port_parameter_set(_camera->control, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set image effect parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitColorFX(void)
{
    if (!_camera) return 1;

    MMAL_PARAMETER_COLOURFX_T param        = { {MMAL_PARAMETER_COLOUR_EFFECT,sizeof(param)}, 0, 0, 0 };
                              param.enable = 0;
                              param.u      = 128;
                              param.v      = 128;
    MMAL_STATUS_T             status       = mmal_port_parameter_set(_camera->control, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set color effect parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitRotation(void)
{
    if (!_camera                ) return 1;
    if ( _camera->output_num < 3) return 1;

    const int param = int((_rotation % 360) / 90) * 90;

    MMAL_STATUS_T               status = mmal_port_parameter_set_int32(_camera->output[0], MMAL_PARAMETER_ROTATION, param);
    if (status == MMAL_SUCCESS) status = mmal_port_parameter_set_int32(_camera->output[1], MMAL_PARAMETER_ROTATION, param);
    if (status == MMAL_SUCCESS) status = mmal_port_parameter_set_int32(_camera->output[2], MMAL_PARAMETER_ROTATION, param);

    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set rotation parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitFlips(void)
{
    if (!_camera                ) return 1;
    if ( _camera->output_num < 3) return 1;

    MMAL_PARAMETER_MIRROR_T param = { {MMAL_PARAMETER_MIRROR, sizeof(MMAL_PARAMETER_MIRROR_T)}, MMAL_PARAM_MIRROR_NONE};

    if      ( _flip_horizontal && _flip_vertical) param.value = MMAL_PARAM_MIRROR_BOTH;
    else if ( _flip_horizontal                  ) param.value = MMAL_PARAM_MIRROR_HORIZONTAL;
    else if (                     _flip_vertical) param.value = MMAL_PARAM_MIRROR_VERTICAL;

    MMAL_STATUS_T               status = mmal_port_parameter_set(_camera->output[0], &param.hdr);
    if (status == MMAL_SUCCESS) status = mmal_port_parameter_set(_camera->output[1], &param.hdr);
    if (status == MMAL_SUCCESS) status = mmal_port_parameter_set(_camera->output[2], &param.hdr);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set image flip parameter.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitROI(void)
{
    if (!_camera) return 1;

    MMAL_PARAMETER_INPUT_CROP_T param             = { {MMAL_PARAMETER_INPUT_CROP, sizeof(param)} };
                                param.rect.x      = (0x10000 * 0);
                                param.rect.y      = (0x10000 * 0);
                                param.rect.width  = (0x10000 * 1);
                                param.rect.height = (0x10000 * 1);

    MMAL_STATUS_T status = mmal_port_parameter_set(_camera->control, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set ROI parameters.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
int Private_Impl_Still::commitShutterSpeed(void)
{
    if (!_camera) return 1;

    MMAL_STATUS_T status = mmal_port_parameter_set_uint32(_camera->control, MMAL_PARAMETER_SHUTTER_SPEED, _shutter_speed);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set shutter speed.\n";
    }

    return mmal_status_to_int(status);
}



/**
 *
 */
int Private_Impl_Still::commitDRC(void)
{
    if (!_camera) return 1;

    MMAL_PARAMETER_DRC_T param  = { {MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, _drc };
    MMAL_STATUS_T        status = mmal_port_parameter_set(_camera->control, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set dynamic range compression.\n";
    }

    return mmal_status_to_int(status);
}


/**
 *
 */
MMAL_FOURCC_T Private_Impl_Still::convertEncoding(const RASPICAM_ENCODING encoding)
{
    switch (encoding)
    {
        case RASPICAM_ENCODING_JPEG: return MMAL_ENCODING_JPEG;
        case RASPICAM_ENCODING_BMP : return MMAL_ENCODING_BMP;
        case RASPICAM_ENCODING_GIF : return MMAL_ENCODING_GIF;
        case RASPICAM_ENCODING_PNG : return MMAL_ENCODING_PNG;
        case RASPICAM_ENCODING_RGB : return MMAL_ENCODING_BMP;
        default                    : return -1;
    }
}


/**
 *
 */
MMAL_PARAM_EXPOSUREMETERINGMODE_T Private_Impl_Still::convertMetering(const RASPICAM_METERING metering)
{
    switch (metering)
    {
        case RASPICAM_METERING_AVERAGE: return MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
        case RASPICAM_METERING_SPOT   : return MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT;
        case RASPICAM_METERING_BACKLIT: return MMAL_PARAM_EXPOSUREMETERINGMODE_BACKLIT;
        case RASPICAM_METERING_MATRIX : return MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX;
        default                       : return MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
    }
}


/**
 *
 */
MMAL_PARAM_EXPOSUREMODE_T Private_Impl_Still::convertExposure(const RASPICAM_EXPOSURE exposure)
{
    switch (exposure)
    {
        case RASPICAM_EXPOSURE_OFF         : return MMAL_PARAM_EXPOSUREMODE_OFF;
        case RASPICAM_EXPOSURE_AUTO        : return MMAL_PARAM_EXPOSUREMODE_AUTO;
        case RASPICAM_EXPOSURE_NIGHT       : return MMAL_PARAM_EXPOSUREMODE_NIGHT;
        case RASPICAM_EXPOSURE_NIGHTPREVIEW: return MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW;
        case RASPICAM_EXPOSURE_BACKLIGHT   : return MMAL_PARAM_EXPOSUREMODE_BACKLIGHT;
        case RASPICAM_EXPOSURE_SPOTLIGHT   : return MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT;
        case RASPICAM_EXPOSURE_SPORTS      : return MMAL_PARAM_EXPOSUREMODE_SPORTS;
        case RASPICAM_EXPOSURE_SNOW        : return MMAL_PARAM_EXPOSUREMODE_SNOW;
        case RASPICAM_EXPOSURE_BEACH       : return MMAL_PARAM_EXPOSUREMODE_BEACH;
        case RASPICAM_EXPOSURE_VERYLONG    : return MMAL_PARAM_EXPOSUREMODE_VERYLONG;
        case RASPICAM_EXPOSURE_FIXEDFPS    : return MMAL_PARAM_EXPOSUREMODE_FIXEDFPS;
        case RASPICAM_EXPOSURE_ANTISHAKE   : return MMAL_PARAM_EXPOSUREMODE_ANTISHAKE;
        case RASPICAM_EXPOSURE_FIREWORKS   : return MMAL_PARAM_EXPOSUREMODE_FIREWORKS;
        default                            : return MMAL_PARAM_EXPOSUREMODE_AUTO;
    }
}


/**
 *
 */
MMAL_PARAM_AWBMODE_T Private_Impl_Still::convertAWB(const RASPICAM_AWB awb)
{
    switch (awb)
    {
        case RASPICAM_AWB_OFF         : return MMAL_PARAM_AWBMODE_OFF;
        case RASPICAM_AWB_AUTO        : return MMAL_PARAM_AWBMODE_AUTO;
        case RASPICAM_AWB_SUNLIGHT    : return MMAL_PARAM_AWBMODE_SUNLIGHT;
        case RASPICAM_AWB_CLOUDY      : return MMAL_PARAM_AWBMODE_CLOUDY;
        case RASPICAM_AWB_SHADE       : return MMAL_PARAM_AWBMODE_SHADE;
        case RASPICAM_AWB_TUNGSTEN    : return MMAL_PARAM_AWBMODE_TUNGSTEN;
        case RASPICAM_AWB_FLUORESCENT : return MMAL_PARAM_AWBMODE_FLUORESCENT;
        case RASPICAM_AWB_INCANDESCENT: return MMAL_PARAM_AWBMODE_INCANDESCENT;
        case RASPICAM_AWB_FLASH       : return MMAL_PARAM_AWBMODE_FLASH;
        case RASPICAM_AWB_HORIZON     : return MMAL_PARAM_AWBMODE_HORIZON;
        default                       : return MMAL_PARAM_AWBMODE_AUTO;
    }
}


/**
 *
 */
MMAL_PARAM_IMAGEFX_T Private_Impl_Still::convertImageEffect(const RASPICAM_IMAGE_EFFECT image_fx)
{
    switch (image_fx)
    {
        case RASPICAM_IMAGE_EFFECT_NONE        : return MMAL_PARAM_IMAGEFX_NONE;
        case RASPICAM_IMAGE_EFFECT_NEGATIVE    : return MMAL_PARAM_IMAGEFX_NEGATIVE;
        case RASPICAM_IMAGE_EFFECT_SOLARIZE    : return MMAL_PARAM_IMAGEFX_SOLARIZE;
        case RASPICAM_IMAGE_EFFECT_SKETCH      : return MMAL_PARAM_IMAGEFX_SKETCH;
        case RASPICAM_IMAGE_EFFECT_DENOISE     : return MMAL_PARAM_IMAGEFX_DENOISE;
        case RASPICAM_IMAGE_EFFECT_EMBOSS      : return MMAL_PARAM_IMAGEFX_EMBOSS;
        case RASPICAM_IMAGE_EFFECT_OILPAINT    : return MMAL_PARAM_IMAGEFX_OILPAINT;
        case RASPICAM_IMAGE_EFFECT_HATCH       : return MMAL_PARAM_IMAGEFX_HATCH;
        case RASPICAM_IMAGE_EFFECT_GPEN        : return MMAL_PARAM_IMAGEFX_GPEN;
        case RASPICAM_IMAGE_EFFECT_PASTEL      : return MMAL_PARAM_IMAGEFX_PASTEL;
        case RASPICAM_IMAGE_EFFECT_WATERCOLOR  : return MMAL_PARAM_IMAGEFX_WATERCOLOUR;
        case RASPICAM_IMAGE_EFFECT_FILM        : return MMAL_PARAM_IMAGEFX_FILM;
        case RASPICAM_IMAGE_EFFECT_BLUR        : return MMAL_PARAM_IMAGEFX_BLUR;
        case RASPICAM_IMAGE_EFFECT_SATURATION  : return MMAL_PARAM_IMAGEFX_SATURATION;
        case RASPICAM_IMAGE_EFFECT_COLORSWAP   : return MMAL_PARAM_IMAGEFX_COLOURSWAP;
        case RASPICAM_IMAGE_EFFECT_WASHEDOUT   : return MMAL_PARAM_IMAGEFX_WASHEDOUT;
        case RASPICAM_IMAGE_EFFECT_POSTERISE   : return MMAL_PARAM_IMAGEFX_POSTERISE;
        case RASPICAM_IMAGE_EFFECT_COLORPOINT  : return MMAL_PARAM_IMAGEFX_COLOURPOINT;
        case RASPICAM_IMAGE_EFFECT_COLORBALANCE: return MMAL_PARAM_IMAGEFX_COLOURBALANCE;
        case RASPICAM_IMAGE_EFFECT_CARTOON     : return MMAL_PARAM_IMAGEFX_CARTOON;
        default                                : return MMAL_PARAM_IMAGEFX_NONE;
    }
}


/**
 * Returns an id of the camera. We assume the camera id is the one of the raspberry
 * the id is obtained using raspberry serial number obtained in /proc/cpuinfo
 */
std::string Private_Impl_Still::getId(void) const
{
    char serial[1024];
         serial[   0] = '\0';

    std::ifstream file("/proc/cpuinfo");

    if (!file)
    {
        std::cerr << __FILE__ << " " << __LINE__ << ":" << __func__ << "Could not read /proc/cpuinfo\n";
        return serial;
    }

    // Read lines until find serial
    bool found = false;

    while (!file.eof() && !found)
    {
        char         aux[  100];
        char         line[1024];
        file.getline(line,1024);

        std::string str(line);

        if (str.find("Serial") != std::string::npos)
        {
            if (sscanf(line,"%s : %s", aux, serial) != 2)
            {
                std::cerr << __FILE__ << " " << __LINE__ << ":" << __func__ << "Error parsing /proc/cpuinfo\n";
            }
            else
            {
                found=true;
            }
        }
    }
    return serial;
}


} // End of namespace _private
} // End of namespace raspicam
