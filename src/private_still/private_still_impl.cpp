/**
 * @file   private_still_impl.cpp
 * @author Jose Nogueira, josenogueira@biosurfit.com
 * @date   May 2017
 * @brief  Private implementation of camera in still mode
 *         Uses picture port
 *
 * (extensive explanation)
 */


/**
 * Include files
 */
#include "private_still_impl.h"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <semaphore.h>

#include "interfaces/mmal/mmal_buffer.h"
#include "interfaces/mmal/mmal_format.h"
#include "interfaces/mmal/util/mmal_default_components.h"
#include "interfaces/mmal/util/mmal_util.h"
#include "interfaces/mmal/util/mmal_util_params.h"

#include "raspitimer.h"


/**
 * Definitions
 */
#define PREVIEW_DEFAULT_WIDTH   640
#define PREVIEW_DEFAULT_HEIGHT  480

#define IMX219_SENSORMODES        8


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
    ImageTakenCallback   callback_image;
    sem_t*               mutex;
    uchar*               data;
    size_t               data_offset;
    size_t               image_offset;
    size_t               length;
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
        if ( (buffer->length > 0) && (userdata->data != NULL) )
        {
            // Ensure userdata memory is sufficient to store buffer data
            if ( userdata->length < (buffer->length + userdata->data_offset) )
            {
                flag_complete = true;
                std::cerr << "Unable to write buffer user memory - " << buffer->length + userdata->data_offset << " > " << userdata->length << '\n';
            }
            else
            {
                // Get data from buffer
                mmal_buffer_header_mem_lock(buffer);
                memcpy(userdata->data+userdata->data_offset, buffer->data, buffer->length);
                userdata->data_offset += buffer->length;
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
            userdata->callback_image(userdata->data                           ,
                                     userdata->image_offset                   ,
                                     userdata->length - userdata->image_offset);
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

const uint Private_Impl_Still::IMX219_RAWOFFSET[] =
{
    10270208, // Sensor mode 0
     2678784, //             1
    10270208, //             2
    10270208, //             3
     2628608, //             4
     1963008, //             5
     1233920, //             6
      445440, //             7
};

const uint Private_Impl_Still::IMX219_RESOLUTIONS[][2] =
{
    {3280, 2464}, // Sensor mode 0
    {1920, 1080}, //             1
    {3280, 2464}, //             2
    {3280, 2464}, //             3
    {1640, 1232}, //             4
    {1640,  922}, //             5
    {1280,  720}, //             6
    { 640,  480}, //             7
};


/**
 *
 */
Private_Impl_Still::Private_Impl_Still(void)
{
    // Camera initialized flag
    _is_initialized      = false;
    _is_capturing        = false;

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

    _sensor_name         = "";

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
                _sensor_name = camera_info_param.cameras[_camera_idx].camera_name;
                std::transform(_sensor_name.begin(), _sensor_name.end(), _sensor_name.begin(), ::tolower);

                std::cout << "Recognized sensor " << _sensor_name << '\n';
            }
        }

        mmal_component_destroy(camera_info);
    }

    // Failed to get sensor information, assume Sony IMX219
    if ( (_width  == 0) ||
         (_height == 0)   )
    {
        _width  = this->IMX219_RESOLUTIONS[0][0];
        _height = this->IMX219_RESOLUTIONS[0][1];
    }
}


/**
 *
 */
void Private_Impl_Still::setDefaults(void)
{
    _camera_idx            =     0;
    _raw_mode              = false;
    _sensor_mode           =     0;
    _sensor_name           =    "";
    _width                 =     0;
    _height                =     0;
    _iso                   =   100;
    _brightness            =    50;
    _sharpness             =     0;
    _contrast              =     0;
    _saturation            =     0;
    _video_stabilization   =     0;

    _exposure_mode         = RASPICAM_EXPOSURE_AUTO;
    _exposure_compensation =     0;
    _exposure_metering     = RASPICAM_METERING_AVERAGE;
    _shutter_speed         =     0;

    _awb_mode              = RASPICAM_AWB_AUTO;
    _awb_gains_r           =     1.0;
    _awb_gains_b           =     1.0;

    _rotation              =     0;
    _flip_horizontal       = false;
    _flip_vertical         = false;

    _image_encoding        = RASPICAM_ENCODING_JPEG;
    _image_quality         =    85;
    _image_fx              = RASPICAM_IMAGE_EFFECT_NONE;
    _drc                   = RASPICAM_DRC_OFF;

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
        _port_encoder_output->format->encoding = this->convertEncoding(_image_encoding);

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
    _port_encoder_output->format->encoding = this->convertEncoding(_image_encoding);

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
    status = mmal_port_parameter_set_uint32(_port_encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, _image_quality);
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
int Private_Impl_Still::initialize(const long int sleep_ready_ms)
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

    // Sleep, camera must be ready before taking photos
    if (sleep_ready_ms > 0) usleep(sleep_ready_ms * 1000);

    return 0;
}


/**
 *
 */
int Private_Impl_Still::release(void)
{
    if (!_is_initialized) return 0;

    if (_is_capturing) this->stopCapture();

    // Disable all our ports that are not handled by connections
    if (_port_encoder_output && _port_encoder_output->is_enabled) mmal_port_disable(_port_encoder_output);
    std::cerr << API_NAME << ": camera port disabled\n";

    // Destroy encoder connection
    if (_encoder_connection) mmal_connection_destroy(_encoder_connection);
    std::cerr << API_NAME << ":  encoder connection destroyed\n";

    if (_encoder) mmal_component_disable(_encoder);
    std::cerr << API_NAME << ":  encoder component disabled\n";

    if (_camera) mmal_component_disable(_camera);
    std::cerr << API_NAME << ":  camera component disabled\n";

    // Get rid of any port buffers first
    if (_encoder_pool) mmal_port_pool_destroy(_encoder->output[0], _encoder_pool);
    std::cerr << API_NAME << ":  encoder pool destroyed\n";

    if (_encoder) mmal_component_destroy(_encoder);
    _encoder = NULL;
    std::cerr << API_NAME << ":  encoder component destroyed\n";

    if (_camera) mmal_component_destroy(_camera);
    _camera = NULL;
    std::cerr << API_NAME << ":  camera component destroyed\n";

    _is_initialized = false;
    return 0;
}


/**
 *
 */
int Private_Impl_Still::takePicture(uchar* data, size_t length, size_t& offset, const bool single)
{
    std::cerr << API_NAME << ": taking picture...\n";
    if (_raw_mode) offset = 0;

    int status = this->initialize();
    if (status != 0)
    {
        std::cerr << API_NAME << ": Failed to take picture | camera was not initialized.\n";
        return (status < 0 ? status : -1);
    }

    // Enable capturing flag
    _is_capturing = true;

    // Init semaphore
    sem_t     mutex;
    sem_init(&mutex, 0, 0);

    RASPICAM_USERDATA* userdata = new RASPICAM_USERDATA();
                       userdata->handle_camera   = this;
                       userdata->encoder_pool    = _encoder_pool;
                       userdata->mutex           = &mutex;
                       userdata->data            = data;
                       userdata->data_offset     = 0;
                       userdata->image_offset    = 0;
                       userdata->length          = length;
                       userdata->callback_image  = NULL;

    _port_encoder_output->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;

    if ( (status = this->startCapture()) != 0 )
    {
        sem_destroy(&mutex);
        delete userdata;

        return -1;
    }

    // Semaphore barrier
    sem_wait(   &mutex);
    sem_destroy(&mutex);

    // Calculate image offset, in case we are taking raw images
    const size_t data_offset = userdata->data_offset;
    if (_raw_mode && (data_offset > this->IMX219_RAWOFFSET[_sensor_mode]) )
    {
        offset = data_offset - this->IMX219_RAWOFFSET[_sensor_mode];
    }
    delete userdata;

    // Disable capture
    if (single) this->stopCapture();

    std::cerr << API_NAME << ": taking picture...DONE\n";

    return data_offset;
}


/**
 *
 */
int Private_Impl_Still::takePictureRaw(uchar* data, size_t length, uchar** data_raw_ptr, const bool single)
{
    if (!_raw_mode) this->setRawMode(true);

    size_t     offset_raw = 0;
    int        rc_bytes   = this->takePicture(data, length, offset_raw, single);

    const int  length_raw = rc_bytes - offset_raw;

    if (length_raw == int(this->IMX219_RAWOFFSET[_sensor_mode]))
    {
        (*data_raw_ptr)  = data + offset_raw;
        rc_bytes        -= offset_raw;
    }
    else
    {
        (*data_raw_ptr) = NULL;
        rc_bytes        = -1;
    }

    return rc_bytes;
}


/**
 *
 */
size_t Private_Impl_Still::getImageBufferSize(void) const
{
    size_t n = (_width * _height * 3) + 54; // Image size * channels + bmp header

    if ( _raw_mode && (_sensor_name == "imx219") )
    {
        n += this->IMX219_RAWOFFSET[_sensor_mode] + 1;
    }

    return n;
}


/**
 *
 */
int Private_Impl_Still::startCapture(ImageTakenCallback  callback_user    ,
                                     uchar*              data_preallocated,
                                     uint                offset           ,
                                     uint                length           )
{
    RASPICAM_USERDATA* userdata = new RASPICAM_USERDATA();
                       userdata->handle_camera   = this;
                       userdata->encoder_pool    = _encoder_pool;
                       userdata->mutex           = NULL;
                       userdata->data            = data_preallocated;
                       userdata->data_offset     = 0;
                       userdata->image_offset    = offset;
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

    // If the parameters were changed and this function wasn't called, it will be called here
    // However if the parameters weren't changed, the function won't do anything - it will return right away
    // Commit configuration parameters
    if (this->commitParameters() != 0)
    {
        std::cerr << API_NAME << ": Failed to commit camera settings.\n";
        return -1;
    }

    if (!_port_encoder_output->is_enabled)
    {
        // Enable encoder output port
        if (mmal_port_enable(_port_encoder_output, encoder_buffer_callback) != MMAL_SUCCESS)
        {
            std::cerr << API_NAME << ": Could not enable encoder output port.\n";
            return -1;
        }
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

    // Start capture
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
    _is_capturing = false;

    if (!_port_encoder_output->is_enabled) return;

    if (mmal_port_disable(_port_encoder_output))
    {
        delete (RASPICAM_USERDATA*) _port_encoder_output->userdata;
    }
}


/**
 *
 */
void Private_Impl_Still::setSensorMode(const int value)
{
    if ( (_sensor_name == "imx219"         ) &&
         (_sensor_mode < IMX219_SENSORMODES)   )
    {
        _sensor_mode      = value;
        this->setResolution(this->IMX219_RESOLUTIONS[_sensor_mode][0],
                            this->IMX219_RESOLUTIONS[_sensor_mode][1]);

        _settings_changed = true;
    }
}


/**
 *
 */
void Private_Impl_Still::setRawMode(const bool active)
{
    _raw_mode         = active;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setResolution( const uint width ,
                                        const uint height)
{
    _width            = width;
    _height           = height;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setBrightness(const uint value)
{
    if (value > 100) _brightness = 100;
    else             _brightness = value;

    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setImageQuality(const uint value)
{
    if (value > 100) _image_quality = 100;
    else             _image_quality = value;

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
void Private_Impl_Still::setShutterSpeed(const int value)
{
    _shutter_speed    = value;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setISO(const int value)
{
    _iso              = value;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setSharpness (const int value)
{
    if      (value < -100) _sharpness = -100;
    else if (value >  100) _sharpness =  100;
    else                   _sharpness = value;

    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setContrast(const int value)
{
    if      (value < -100) _contrast = -100;
    else if (value >  100) _contrast =  100;
    else                   _contrast = value;

    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setSaturation (const int value)
{
    if      (value < -100) _saturation  = -100;
    else if (value >  100) _saturation  =  100;
    else                   _saturation = value;

    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setVideoStabilization(const int value)
{
    if (value > 0) _video_stabilization = 1;
    else           _video_stabilization = 0;

    _settings_changed      = true;
}


/**
 *
 */
void Private_Impl_Still::setImageEncoding(const RASPICAM_ENCODING encoding)
{
    _image_encoding   = encoding;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setExposureMode(const RASPICAM_EXPOSURE exposure)
{
    _exposure_mode    = exposure;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setExposureCompensation(const int value)
{
    _exposure_compensation = value;
    _settings_changed      = true;
}


/**
 *
 */
void Private_Impl_Still::setExposureMetering(const RASPICAM_METERING metering)
{
    _exposure_metering = metering;
    _settings_changed  = true;
}


/**
 *
 */
void Private_Impl_Still::setAWBMode(const RASPICAM_AWB awb)
{
    _awb_mode         = awb;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setAWBGains(const float gain_r, const float gain_b)
{
    _awb_gains_r = gain_r;
    _awb_gains_b = gain_b;
}


/**
 *
 */
void Private_Impl_Still::setImageEffect(const RASPICAM_IMAGE_EFFECT image_fx)
{
    _image_fx         = image_fx;
    _settings_changed = true;
}


/**
 *
 */
void Private_Impl_Still::setDRC(const RASPICAM_DRC drc)
{
    _drc              = drc;
    _settings_changed = true;
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

    MMAL_PARAMETER_AWBMODE_T param  = { {MMAL_PARAMETER_AWB_MODE,sizeof(param)}, this->convertAWB(_awb_mode) };
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

    MMAL_PARAMETER_DRC_T param  = { {MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, this->convertDRC(_drc) };
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
 *
 */
MMAL_PARAMETER_DRC_STRENGTH_T Private_Impl_Still::convertDRC(const RASPICAM_DRC drc)
{
    switch (drc)
    {
        case RASPICAM_DRC_OFF    : return MMAL_PARAMETER_DRC_STRENGTH_OFF;
        case RASPICAM_DRC_LOW    : return  MMAL_PARAMETER_DRC_STRENGTH_LOW;
        case RASPICAM_DRC_MEDIUM : return  MMAL_PARAMETER_DRC_STRENGTH_MEDIUM;
        case RASPICAM_DRC_HIGH   : return MMAL_PARAMETER_DRC_STRENGTH_HIGH;
        case RASPICAM_DRC_MAX    : return MMAL_PARAMETER_DRC_STRENGTH_MAX;
        default                  : return MMAL_PARAMETER_DRC_STRENGTH_OFF;
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
