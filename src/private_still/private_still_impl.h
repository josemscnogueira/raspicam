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

#ifndef _Private_RaspiCam_STILL_IMPL_H
#define _Private_RaspiCam_STILL_IMPL_H

/**
 * Include files
 */
#include "raspicamtypes.h"
#include "interfaces/mmal/mmal.h"
#include "interfaces/mmal/util/mmal_connection.h"

#include <string>


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

        typedef void (*imageTakenCallback) (uchar* data, uint image_offset, uint length);

        class Private_Impl_Still
        {
        private:
            // Attributes
            bool                                 _is_initialized;
            std::string                          _camera_name;

            bool                                 _raw_mode;

                // MMAL Attributes
            MMAL_COMPONENT_T*                    _camera;             // Pointer to the camera component
            MMAL_COMPONENT_T*                    _encoder;            // Pointer to the encoder component

            MMAL_CONNECTION_T*                   _encoder_connection; // Connection from the camera to the encoder

            MMAL_POOL_T*                         _encoder_pool;       // Pointer to the pool of buffers used by encoder output port

            MMAL_PORT_T*                         _port_camera;
            MMAL_PORT_T*                         _port_encoder_input;
            MMAL_PORT_T*                         _port_encoder_output;
                // Camera+Sensor Settings
            uint                                 _width;
            uint                                 _height;
            uint                                 _rotation;           // 0 to 359
            uint                                 _brightness;         // 0 to 100
            uint                                 _quality;            // 0 to 100
            int                                  _iso;
            int                                  _sharpness;          // -100 to 100
            int                                  _contrast;           // -100 to 100
            int                                  _saturation;         // -100 to 100
            int                                  _video_stabilization;

            int                                  _camera_idx;         // Only compute module supports idx > 0
            int                                  _sensor_mode;

            RASPICAM_EXPOSURE                    _exposure_mode;
            int                                  _exposure_compensation;
            RASPICAM_METERING                    _exposure_metering;
            int                                  _shutter_speed;

            RASPICAM_AWB                         _awb;
            float                                _awb_gains_r;
            float                                _awb_gains_b;
            // MMAL_PARAM_IMAGEFX_T                 _image_fx; // TODO REMOVE

            bool                                 _flip_horizontal;
            bool                                 _flip_vertical;

            RASPICAM_ENCODING                    _encoding;
			RASPICAM_IMAGE_EFFECT                _image_fx;
            MMAL_PARAMETER_DRC_STRENGTH_T        _drc;

                // Other atributes
            bool                                 _settings_changed;

            // Methods
                // MMAL methods
            static MMAL_FOURCC_T                     convertEncoding(   RASPICAM_ENCODING     encoding);
            static MMAL_PARAM_EXPOSUREMETERINGMODE_T convertMetering(   RASPICAM_METERING     metering);
            static MMAL_PARAM_EXPOSUREMODE_T         convertExposure(   RASPICAM_EXPOSURE     exposure);
            static MMAL_PARAM_AWBMODE_T              convertAWB(        RASPICAM_AWB          awb     );
            static MMAL_PARAM_IMAGEFX_T              convertImageEffect(RASPICAM_IMAGE_EFFECT image_fx);
                // Camera settings
            int                               commitSaturation(  void);
            int                               commitSharpness(   void);
            int                               commitContrast(    void);
            int                               commitBrightness(  void);
            int                               commitISO(         void);
            int                               commitVideoStabilization(void);
            int                               commitExposureCompensation(void);
            int                               commitExposureMode(void);
            int                               commitExposureMetering(void);
            int                               commitAWB(         void);
            int                               commitAWBGains(    void);
            int                               commitImageFX(     void);
            int                               commitColorFX(     void);
            int                               commitRotation(    void);
            int                               commitFlips(       void);
            int                               commitROI(         void);
            int                               commitShutterSpeed(void);
            int                               commitDRC(         void);
                // Camera interface
            void                              getSensorInfo(     void);

            int                               startCapture(      void);
            int                               createCamera(      void);
            int                               createEncoder(     void);
            void                              disableComponents( void);
            void                              destroyCamera(     void);
            void                              destroyEncoder(    void);
            void                              disablePorts(      void);
            void                              setDefaults(       void);
            static MMAL_STATUS_T              connectPorts(      MMAL_PORT_T*          port_output,
                                                                 MMAL_PORT_T*          port_input ,
                                                                 MMAL_CONNECTION_T**   connection );

        public:
            static const std::string API_NAME;

            // Constructor
            Private_Impl_Still(void);

            // Destructor
            ~Private_Impl_Still(void) { this->release(); };

            int    initialize(        void);
            int    release(           void);

            int    startCapture(      imageTakenCallback          callback_user    ,
                                      uchar*                      data_preallocated,
                                      uint                        offset           ,
                                      uint                        length           );
            void   stopCapture(       void);
            bool   takePicture(       uchar*                      data_preallocated,
                                      uint                        length );

            size_t getImageBufferSize(void) const;
            void   bufferCallback(    MMAL_PORT_T*                port  ,
                                      MMAL_BUFFER_HEADER_T*       buffer);

            // Setters
            int    commitParameters(  void);
            void   setWidth(          const uint                  width             );
            void   setHeight(         const uint                  height            );
            void   setCaptureSize(    const uint                  width, uint height);
            void   setBrightness(     const uint                  brightness        );
            void   setQuality(        const uint                  quality           );
            void   setRotation(       int                         rotation          );
            void   setISO(            const int                   iso               );
            void   setSharpness(      const int                   sharpness         );
            void   setContrast(       const int                   contrast          );
            void   setSaturation(     const int                   saturation        );
            void   setEncoding(       const RASPICAM_ENCODING     encoding          );
            void   setExposure(       const RASPICAM_EXPOSURE     exposure          );
            void   setAWB(            const RASPICAM_AWB          awb               );
            void   setImageEffect(    const RASPICAM_IMAGE_EFFECT image_fx          );
            void   setMetering(       const RASPICAM_METERING     metering          );
            void   setHorizontalFlip( const bool                  flip              );
            void   setVerticalFlip(   const bool                  flip              );

            uint                  getWidth(             void) const {return _width;          };
            uint                  getHeight(            void) const {return _height;         };
            uint                  getBrightness(        void) const {return _brightness;     };
            uint                  getRotation(          void) const {return _rotation;       };
            uint                  getQuality(           void) const {return _quality;        };
            int                   getISO(               void) const {return _iso;            };
            int                   getSharpness(         void) const {return _sharpness;      };
            int                   getContrast(          void) const {return _contrast;       };
            int                   getSaturation(        void) const {return _saturation;     };
            RASPICAM_ENCODING     getEncoding(          void) const {return _encoding;       };
            RASPICAM_EXPOSURE     getExposure(          void) const {return _exposure_mode;  };
            RASPICAM_AWB          getAWB(               void) const {return _awb;            };
            RASPICAM_IMAGE_EFFECT getImageEffect(       void) const {return _image_fx;       };
            RASPICAM_METERING     getMetering(          void) const {return _exposure_metering; };
            bool                  isHorizontallyFlipped(void) const {return _flip_horizontal;};
            bool                  isVerticallyFlipped(  void) const {return _flip_vertical;  };


            //Returns an id of the camera. We assume the camera id is the one of the raspberry
            //the id is obtained using raspberry serial number obtained in /proc/cpuinfo
            std::string getId() const;
        };
    }
}
#endif // RASPICAM_H
