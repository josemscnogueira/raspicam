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
}


/**
 * Camera callbacks
 */
static void buffer_callback(MMAL_PORT_T*          port  ,
                            MMAL_BUFFER_HEADER_T* buffer)
{
    RASPICAM_USERDATA* userdata = (RASPICAM_USERDATA*) port->userdata;

    if ( (userdata                != NULL) &&
         (userdata->handle_camera != NULL)   )
    {
        const uint flags = buffer->flags;

        mmal_buffer_header_mem_lock(buffer);
        {
            for (size_t idx = 0; idx < buffer->length; ++idx, ++userdata->buffer_position)
            {
                if (userdata->offset >= userdata->length)
                {
                    std::cerr << userdata->handle_camera->API_NAME << ": Buffer provided was too small! Failed to copy data into buffer." << std::endl;

                    userdata->handle_camera = NULL;
                    break;
                }
                else
                {
                    if (userdata->handle_camera->getEncoding() == RASPICAM_ENCODING_RGB)
                    {
                        // Determines if the byte is an RGB value
                        if (userdata->buffer_position >= 54)
                        {
                            userdata->data[userdata->offset] = buffer->data[idx];
                            ++userdata->offset;
                        }
                    }
                    else
                    {
                        userdata->data[userdata->offset] = buffer->data[idx];
                        ++userdata->offset;
                    }
                }
            }
        }
        mmal_buffer_header_mem_unlock(buffer);

        const uint END_FLAG = flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | /* bitwise_or */
                                       MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED);


        if (END_FLAG != 0)
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

    mmal_buffer_header_release(buffer);

    if (port->is_enabled)
    {
        MMAL_BUFFER_HEADER_T* new_buffer = mmal_queue_get(userdata->encoder_pool->queue);

        if (new_buffer) mmal_port_send_buffer(port, new_buffer);
    }
}


const std::string Private_Impl_Still::API_NAME = "PRIVATE_IMPL_RaspiCamStill";

/**
 *
 */
Private_Impl_Still::Private_Impl_Still(void)
{
    // Camera initialized flag
    _is_initialized      = false;
    _camera_name         = "";

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
    MMAL_STATUS_T       status;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);
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
            }
        }
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

    _encoding              = RASPICAM_ENCODING_JPEG;
	_image_fx              = RASPICAM_IMAGE_EFFECT_NONE;

    // Get sensor information (resolution and name)
    this->getSensorInfo();

    _settings_changed      = true;
}



/**
 *
 */
void Private_Impl_Still::commitParameters(void)
{
    if (!_settings_changed) return;

    commitSharpness();
    commitContrast();
    commitBrightness();
    commitQuality();
    commitSaturation();
    commitISO();
    commitExposure();
    commitMetering();
    commitAWB();
    commitImageEffect();
    commitRotation();
    commitFlips();

    // Set Video Stabilization
    if (mmal_port_parameter_set_boolean(_camera->control, MMAL_PARAMETER_VIDEO_STABILISATION, 0) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set video stabilization parameter." << std::endl;
    }

    // Set Exposure Compensation
    if (mmal_port_parameter_set_int32(_camera->control, MMAL_PARAMETER_EXPOSURE_COMP, 0) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set exposure compensation parameter." << std::endl;
    }

    // Set Color Efects
    MMAL_PARAMETER_COLOURFX_T colfx        = { {MMAL_PARAMETER_COLOUR_EFFECT, sizeof(colfx) }, 0, 0, 0 };
                              colfx.enable =   0;
                              colfx.u      = 128;
                              colfx.v      = 128;
    if (mmal_port_parameter_set(_camera->control, &colfx.hdr) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set color effects parameter." << std::endl;
    }

    // Set ROI
    MMAL_PARAMETER_INPUT_CROP_T crop = { {MMAL_PARAMETER_INPUT_CROP, sizeof(MMAL_PARAMETER_INPUT_CROP_T)} };
                                crop.rect.x      = (65536 * 0);
                                crop.rect.y      = (65536 * 0);
                                crop.rect.width  = (65536 * 1);
                                crop.rect.height = (65536 * 1);
    if (mmal_port_parameter_set(_camera->control, &crop.hdr) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set ROI parameter." << std::endl;
    }

    // Set encoder encoding
    if (_port_encoder_output != NULL)
    {
        _port_encoder_output->format->encoding = this->convertEncoding(_encoding);

        mmal_port_format_commit(_port_encoder_output);
    }

    _settings_changed = false;
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
    if (!_camera->output_num)
    {
        std::cerr << API_NAME << ": Camera doesn't have output ports.\n";
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
    _port_camera = _camera->output[MMAL_CAMERA_CAPTURE_PORT];

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
            .max_preview_video_w                   = _width,
            .max_preview_video_h                   = _height,
            .num_preview_video_frames              = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume                   = 0,
            .use_stc_timestamp                     = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };

        mmal_port_parameter_set(_camera->control, &cfg_camera.hdr);
    }

    // Commit configuration parameters
    this->commitParameters();


    // Change fps range if necessary, according to shutter_speed // FIXME
    if (_shutter_speed > 6e6)
    {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = { {MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                                 { 50, 1000}                                  ,
                                                 {166, 1000}                                  };
        mmal_port_parameter_set(_port_camera, &fps_range.hdr);
    }
    else if (_shutter_speed > 1e6)
    {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = { {MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                                 {167, 1000}                                  ,
                                                 {999, 1000}                                  };
        mmal_port_parameter_set(_port_camera, &fps_range.hdr);
    }

    // Set format for camera port
    MMAL_ES_FORMAT_T* format = _port_camera->format;
                      format->encoding                 = MMAL_ENCODING_OPAQUE;
                      format->es->video.width          = VCOS_ALIGN_UP( _width,32);
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
        std::cerr << API_NAME << ": Camera port format couldn't be set.\n";
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
    // Create camera component
    MMAL_STATUS_T status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &_encoder);

    // Create default image encoder
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Could not create encoder component.\n";
        this->destroyEncoder();
        return -1;
    }

    // Verify encoder input and output ports
    if (!_encoder->input_num || !_encoder->output_num)
    {
        std::cerr << API_NAME << ": Encoder does not have input/output ports.\n";
        this->destroyEncoder();
        return -1;
    }

    // Copy encoder port references
    _port_encoder_input  = _encoder->input[ 0];
    _port_encoder_output = _encoder->output[0];

    // Force encoder ports to have the same format
    mmal_format_copy(_port_encoder_output->format, _port_encoder_input->format);

    // Specify output format
    _port_encoder_output->format->encoding = _encoding;

    // Set buffer size
    if (_port_encoder_output->buffer_size < _port_encoder_output->buffer_size_min)
        _port_encoder_output->buffer_size = _port_encoder_output->buffer_size_min;
    else
        _port_encoder_output->buffer_size = _port_encoder_output->buffer_size_recommended;


    if (_port_encoder_output->buffer_num < _port_encoder_output->buffer_num_min)
        _port_encoder_output->buffer_num = _port_encoder_output->buffer_num_min;
    else
        _port_encoder_output->buffer_num = _port_encoder_output->buffer_num_recommended;

    // Commit the port changes to the output port
    status = mmal_port_format_commit(_port_encoder_output);
    if (status != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Could not set format on encoder output port.\n";
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

    // Set up any required thumbnail
    {
        MMAL_PARAMETER_THUMBNAIL_CONFIG_T param_thumb = { {MMAL_PARAMETER_THUMBNAIL_CONFIGURATION, sizeof(param_thumb)},
                                                           0, 0, 0, 0};

        status = mmal_port_parameter_set(_encoder->control, &param_thumb.hdr);
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
void Private_Impl_Still::disablePorts(void)
{
    std::cout << "Disabling ports..." << std::endl;

    if (_port_camera         && _port_camera        ->is_enabled)  mmal_port_disable(_port_camera        );
    if (_port_encoder_input  && _port_encoder_input ->is_enabled)  mmal_port_disable(_port_encoder_input );

    if (_port_encoder_output && _port_encoder_output->is_enabled)
    {
        std::cout << "_port_encoder_output as enabled!!!" << std::endl;

        if (mmal_port_disable(_port_encoder_output))
        {
            delete (RASPICAM_USERDATA*) _port_encoder_output->userdata;
        }
    }

    std::cout << "Disabling ports...DONE" << std::endl;
}


/**
 *
 */
void Private_Impl_Still::disableComponents(void)
{
    std::cout << "disableComponents..." << std::endl;
    if (_encoder) mmal_component_disable(_encoder);
    std::cout << "disableComponents...ENCODER" << std::endl;
    if (_camera ) mmal_component_disable(_camera);
    std::cout << "disableComponents...CAMERA" << std::endl;
    std::cout << "disableComponents...DONE" << std::endl;
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
    // If the parameters were changed and this function wasn't called, it will be called here
    // However if the parameters weren't changed, the function won't do anything - it will return right away
    commitParameters();

    if (_port_encoder_output->is_enabled)
    {
        std::cerr << API_NAME << ": Could not enable encoder output port. Try waiting longer before attempting to take another picture." << std::endl;
        return -1;
    }

    if (mmal_port_enable(_port_encoder_output, buffer_callback) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Could not enable encoder output port." << std::endl;
        return -1;
    }

    const int queue_size = mmal_queue_length(_encoder_pool->queue);

    for (int idx = 0; idx < queue_size; ++idx )
    {
        MMAL_BUFFER_HEADER_T* buffer = mmal_queue_get(_encoder_pool->queue);

        if (!buffer)
        {
            std::cerr << API_NAME << ": Could not get buffer (#" << idx << ") from pool queue." << std::endl;
        }

        if (mmal_port_send_buffer(_port_encoder_output, buffer) != MMAL_SUCCESS)
        {
            std::cerr << API_NAME << ": Could not send a buffer (#" << idx << ") to encoder output port." << std::endl;
        }
    }

    if (mmal_port_parameter_set_boolean(_port_camera, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to start capture." << std::endl;
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
void Private_Impl_Still::commitBrightness(void)
{
    if (mmal_port_parameter_set_rational(_camera->control, MMAL_PARAMETER_BRIGHTNESS, (MMAL_RATIONAL_T) {int(_brightness), 100}) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set brightness parameter." << std::endl;
    }
}


/**
 *
 */
void Private_Impl_Still::commitQuality(void)
{
    if (_port_encoder_output != NULL)
    {
        if (mmal_port_parameter_set_uint32(_port_encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, _quality) != MMAL_SUCCESS)
        {
            std::cerr << API_NAME << ": Failed to set JPEG quality parameter." << std::endl;
        }
    }
}

/**
 *
 */
void Private_Impl_Still::commitRotation(void)
{
    const int rotation = int(_rotation / 90) * 90;

    if ( (mmal_port_parameter_set_int32(_camera->output[0], MMAL_PARAMETER_ROTATION, rotation) != MMAL_SUCCESS) ||
         (mmal_port_parameter_set_int32(_camera->output[1], MMAL_PARAMETER_ROTATION, rotation) != MMAL_SUCCESS) ||
         (mmal_port_parameter_set_int32(_camera->output[2], MMAL_PARAMETER_ROTATION, rotation) != MMAL_SUCCESS)   )
    {
        std::cerr << API_NAME << ": Failed to set rotation parameter." << std::endl;
    }
}


/**
 *
 */
void Private_Impl_Still::commitISO(void)
{
    if (mmal_port_parameter_set_uint32(_camera->control, MMAL_PARAMETER_ISO, _iso) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set ISO parameter." << std::endl;
    }
}


/**
 *
 */
void Private_Impl_Still::commitSharpness(void)
{
    if (mmal_port_parameter_set_rational(_camera->control, MMAL_PARAMETER_SHARPNESS, (MMAL_RATIONAL_T) {_sharpness, 100}) != MMAL_SUCCESS )
    {
        std::cerr << API_NAME << ": Failed to set sharpness parameter." << std::endl;
    }
}


/**
 *
 */
void Private_Impl_Still::commitContrast(void)
{
    if (mmal_port_parameter_set_rational(_camera->control, MMAL_PARAMETER_CONTRAST, (MMAL_RATIONAL_T) {_contrast, 100}) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set contrast parameter." << std::endl;
    }
}


/**
 *
 */
void Private_Impl_Still::commitSaturation(void)
{
    if (mmal_port_parameter_set_rational(_camera->control, MMAL_PARAMETER_SATURATION, (MMAL_RATIONAL_T) {_saturation, 100}) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set saturation parameter." << std::endl;
    }
}


/**
 *
 */
void Private_Impl_Still::commitExposure(void)
{
    MMAL_PARAMETER_EXPOSUREMODE_T exp_mode = { {MMAL_PARAMETER_EXPOSURE_MODE,sizeof(exp_mode)}, this->convertExposure(_exposure_mode) };

    if (mmal_port_parameter_set(_camera->control, &exp_mode.hdr) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set exposure parameter." << std::endl;
    }
}


/**
 *
 */
void Private_Impl_Still::commitAWB(void)
{
    MMAL_PARAMETER_AWBMODE_T param = { {MMAL_PARAMETER_AWB_MODE,sizeof(param)}, this->convertAWB(_awb) };

    if (mmal_port_parameter_set(_camera->control, &param.hdr) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set AWB parameter." << std::endl;
    }
}


/**
 *
 */
void Private_Impl_Still::commitImageEffect(void)
{
    MMAL_PARAMETER_IMAGEFX_T image_fx = { {MMAL_PARAMETER_IMAGE_EFFECT,sizeof(image_fx)}, this->convertImageEffect(_image_fx) };

    if (mmal_port_parameter_set(_camera->control, &image_fx.hdr) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set image effect parameter." << std::endl;
    }
}


/**
 *
 */
void Private_Impl_Still::commitMetering(void)
{
    MMAL_PARAMETER_EXPOSUREMETERINGMODE_T meter_mode = { {MMAL_PARAMETER_EXP_METERING_MODE, sizeof(meter_mode)}, this->convertMetering(_exposure_metering) };

    if (mmal_port_parameter_set(_camera->control, &meter_mode.hdr) != MMAL_SUCCESS)
    {
        std::cerr << API_NAME << ": Failed to set metering parameter." << std::endl;
    }
}


/**
 *
 */
void Private_Impl_Still::commitFlips(void)
{
    MMAL_PARAMETER_MIRROR_T mirror = { {MMAL_PARAMETER_MIRROR, sizeof(MMAL_PARAMETER_MIRROR_T)}, MMAL_PARAM_MIRROR_NONE};

    if      ( _flip_horizontal && _flip_vertical) mirror.value = MMAL_PARAM_MIRROR_BOTH;
    else if ( _flip_horizontal                  ) mirror.value = MMAL_PARAM_MIRROR_HORIZONTAL;
    else if (                     _flip_vertical) mirror.value = MMAL_PARAM_MIRROR_VERTICAL;

    if ( (mmal_port_parameter_set(_camera->output[0], &mirror.hdr) != MMAL_SUCCESS) ||
         (mmal_port_parameter_set(_camera->output[1], &mirror.hdr) != MMAL_SUCCESS) ||
         (mmal_port_parameter_set(_camera->output[2], &mirror.hdr)                )   )
    {
        std::cerr << API_NAME << ": Failed to set horizontal/vertical flip parameter." << std::endl;
    }
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
        std::cerr << __FILE__ << " " << __LINE__ << ":" << __func__ << "Could not read /proc/cpuinfo" << std::endl;
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
                std::cerr << __FILE__ << " " << __LINE__ << ":" << __func__ << "Error parsing /proc/cpuinfo" << std::endl;
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
