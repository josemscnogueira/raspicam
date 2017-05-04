/**
 * @file   private_still_impl.h
 * @author Jose Nogueira, josenogueira@biosurfit.com
 * @date   May 2017
 * @brief  Private implementation of camera in still mode
 *         Uses picture port
 *
 * (extensive explanation)
 */

#ifndef _Private_RaspiCam_STILL_IMPL_H
#define _Private_RaspiCam_STILL_IMPL_H

/**
 * Include files
 */
#include "raspicamtypes.h"
#include "interfaces/mmal/mmal.h"
#include "interfaces/mmal/util/mmal_connection.h"

#include <string>
#include <utility>


/**
 * Defines
 */
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT   1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Frames rates of 0 implies variable, but denominator needs to be 1 to prevent div by 0
#define STILLS_FRAME_RATE_NUM    0
#define STILLS_FRAME_RATE_DEN    1
#define PREVIEW_FRAME_RATE_NUM   0
#define PREVIEW_FRAME_RATE_DEN   1

#define VIDEO_OUTPUT_BUFFERS_NUM 3

namespace raspicam
{
    namespace _private
    {
        /**
         * Typedefs
         */
        typedef unsigned char uchar;
        typedef unsigned int  uint;

        typedef void (*ImageTakenCallback) (uchar* data, uint image_offset, uint length);

        class Private_Impl_Still
        {
        private:
            // Attributes
            bool                       _is_initialized;
                // MMAL Attributes
            MMAL_COMPONENT_T*          _camera;             // Pointer to the camera component
            MMAL_COMPONENT_T*          _encoder;            // Pointer to the encoder component

            MMAL_CONNECTION_T*         _encoder_connection; // Connection from the camera to the encoder

            MMAL_POOL_T*               _encoder_pool;       // Pointer to the pool of buffers used by encoder output port

            MMAL_PORT_T*               _port_camera;
            MMAL_PORT_T*               _port_encoder_input;
            MMAL_PORT_T*               _port_encoder_output;

                // Camera+Sensor Settings
            int                        _camera_idx;         // Only compute module supports idx > 0
            bool                       _raw_mode;
            std::string                _sensor_name;
            int                        _sensor_mode;
            uint                       _width;
            uint                       _height;
            int                        _iso;
            uint                       _brightness;         // 0 to 100
            int                        _sharpness;          // -100 to 100
            int                        _contrast;           // -100 to 100
            int                        _saturation;         // -100 to 100
            int                        _video_stabilization;

            RASPICAM_EXPOSURE          _exposure_mode;
            int                        _exposure_compensation;
            RASPICAM_METERING          _exposure_metering;
            int                        _shutter_speed;

            RASPICAM_AWB               _awb_mode;
            float                      _awb_gains_r;
            float                      _awb_gains_b;

            uint                       _rotation;           // 0 to 359
            bool                       _flip_horizontal;
            bool                       _flip_vertical;

            RASPICAM_ENCODING          _image_encoding;
            uint                       _image_quality;      // 0 to 100
			RASPICAM_IMAGE_EFFECT      _image_fx;
            RASPICAM_DRC               _drc;

                // Other atributes
            bool                       _settings_changed;

            // Methods
                // MMAL methods
            static MMAL_FOURCC_T                     convertEncoding(   RASPICAM_ENCODING     encoding);
            static MMAL_PARAM_EXPOSUREMETERINGMODE_T convertMetering(   RASPICAM_METERING     metering);
            static MMAL_PARAM_EXPOSUREMODE_T         convertExposure(   RASPICAM_EXPOSURE     exposure);
            static MMAL_PARAM_AWBMODE_T              convertAWB(        RASPICAM_AWB          awb     );
            static MMAL_PARAM_IMAGEFX_T              convertImageEffect(RASPICAM_IMAGE_EFFECT image_fx);
            static MMAL_PARAMETER_DRC_STRENGTH_T     convertDRC(        RASPICAM_DRC          drc     );
                // Camera settings
            int                                      commitSaturation(          void);
            int                                      commitSharpness(           void);
            int                                      commitContrast(            void);
            int                                      commitBrightness(          void);
            int                                      commitISO(                 void);
            int                                      commitVideoStabilization(  void);
            int                                      commitExposureCompensation(void);
            int                                      commitExposureMode(        void);
            int                                      commitExposureMetering(    void);
            int                                      commitAWB(                 void);
            int                                      commitAWBGains(            void);
            int                                      commitImageFX(             void);
            int                                      commitColorFX(             void);
            int                                      commitRotation(            void);
            int                                      commitFlips(               void);
            int                                      commitROI(                 void);
            int                                      commitShutterSpeed(        void);
            int                                      commitDRC(                 void);
                // Camera interface
            void                                     getSensorInfo(             void);

            int                                      startCapture(              void);
            int                                      createCamera(              void);
            int                                      createEncoder(             void);
            void                                     disableComponents(         void);
            void                                     destroyCamera(             void);
            void                                     destroyEncoder(            void);
            void                                     disablePorts(              void);
            void                                     setDefaults(               void);
            static MMAL_STATUS_T                     connectPorts(MMAL_PORT_T*          port_output,
                                                                  MMAL_PORT_T*          port_input ,
                                                                  MMAL_CONNECTION_T**   connection );

        public:
            static const std::string API_NAME;

            static const uint         IMX219_RAWOFFSET[];
            static const uint         IMX219_RESOLUTIONS[][2];

            // Constructor
            Private_Impl_Still(void);

            // Destructor
            ~Private_Impl_Still(void) { this->release(); };

            int    initialize(        void);
            int    release(           void);
            int    commitParameters(  void);

            int    startCapture(      ImageTakenCallback          callback_user    ,
                                      uchar*                      data_preallocated,
                                      uint                        offset           ,
                                      uint                        length           );
            void   stopCapture(       void);
            int    takePicture(       uchar*                      data_preallocated,
                                      uint                        length           );

            size_t getImageBufferSize(void) const;

            // Setters
            void   setRawMode(             const bool                  value                     );
            void   setSensorMode(          const int                   value                     );
            void   setResolution(          const uint                  width, const uint   height);
            void   setSaturation(          const int                   value                     );
            void   setSharpness(           const int                   value                     );
            void   setContrast(            const int                   value                     );
            void   setBrightness(          const uint                  value                     );
            void   setISO(                 const int                   value                     );
            void   setVideoStabilization(  const int                   value                     );
            void   setExposureCompensation(const int                   value                     );
            void   setExposureMode(        const RASPICAM_EXPOSURE     mode                      );
            void   setExposureMetering(    const RASPICAM_METERING     metering                  );
            void   setShutterSpeed(        const int                   value                     );
            void   setAWBMode(             const RASPICAM_AWB          awb                       );
            void   setAWBGains(            const float                 gain_r, const float gain_b);
            void   setRotation(            int                         value                     );
            void   setHorizontalFlip(      const bool                  flip                      );
            void   setVerticalFlip(        const bool                  flip                      );
            void   setImageEncoding(       const RASPICAM_ENCODING     encoding                  );
            void   setImageQuality(        const uint                  value                     );
            void   setImageEffect(         const RASPICAM_IMAGE_EFFECT image_fx                  );
            void   setDRC(                 const RASPICAM_DRC          drc                       );

            bool                   getRawMode(             void) const { return _raw_mode;                                         };
            int                    getSensorMode(          void) const { return _sensor_mode;                                      };
            std::pair<uint,uint>   getResolution(          void) const { return std::pair<uint,uint>(_width,_height);              };
            int                    getSaturation(          void) const { return _saturation;                                       };
            int                    getSharpness(           void) const { return _sharpness;                                        };
            int                    getContrast(            void) const { return _contrast;                                         };
            uint                   getBrightness(          void) const { return _brightness;                                       };
            int                    getISO(                 void) const { return _iso;                                              };
            int                    getVideoStabilization(  void) const { return _video_stabilization;                              };
            int                    getExposureCompensation(void) const { return _exposure_compensation;                            };
            RASPICAM_EXPOSURE      getExposureMode(        void) const { return _exposure_mode;                                    };
            RASPICAM_METERING      getExposureMetering(    void) const { return _exposure_metering;                                };
            int                    getShutterSpeed(        void) const { return _shutter_speed;                                    };
            RASPICAM_AWB           getAWBMode(             void) const { return _awb_mode;                                         };
            std::pair<float,float> getAWBGains(            void) const { return std::pair<float,float>(_awb_gains_r,_awb_gains_b); };
            int                    getRotation(            void) const { return _rotation;                                         };
            bool                   getHorizontalFlip(      void) const { return _flip_horizontal;                                  };
            bool                   getVerticalFlip(        void) const { return _flip_vertical;                                    };
            RASPICAM_ENCODING      getImageEncoding(       void) const { return _image_encoding;                                   };
            uint                   getImageQuality(        void) const { return _image_quality;                                    };
            RASPICAM_IMAGE_EFFECT  getImageEffect(         void) const { return _image_fx;                                         };
            RASPICAM_DRC           getDRC(                 void) const { return _drc;                                              };

            //Returns an id of the camera. We assume the camera id is the one of the raspberry
            //the id is obtained using raspberry serial number obtained in /proc/cpuinfo
            std::string            getId(                  void) const;
        };
    }
}
#endif // RASPICAM_H
