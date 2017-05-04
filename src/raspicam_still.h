/**
 * @file   raspicam_still.h
 * @author Jose Nogueira, josenogueira@biosurfit.com
 * @date   May 2017
 * @brief  Private implementation of camera in still mode
 *         Uses picture port
 *
 * (extensive explanation)
 */

#ifndef _RASPICAM_STILL_H_
#define _RASPICAM_STILL_H_

/**
 * Include files
 */
#include "raspicamtypes.h"

#include <cstdio>
#include <utility>


namespace raspicam
{
    // Define Camera Interface
    namespace _private { class Private_Impl_Still; };


    /**
     * Raspicam API for still camera
     */
    class RaspiCam_Still
    {
    private:
        // The implementation of the camera
        _private::Private_Impl_Still* _impl;

    public:
        //Constructor
        RaspiCam_Still(void);

        //Destructor
        ~RaspiCam_Still(void);

        // Opens camera connection
        bool open(void);

        // Grabs and set the data into the data buffer which has the indicated length. It is your responsability
        // to alloc the buffer. You can use getImageBufferSize for that matter.
        int grab_retrieve(unsigned char* data, unsigned int length);

        // Releases the camera
        bool release(void);

        // Returns the size of the images captured with the current parameters
        size_t getImageBufferSize(void) const;

        // Camera configuration
        int commitParameters(void);

        // Setters
        void   setRawMode(             const bool                  value                           );
        void   setSensorMode(          const int                   value                           );
        void   setResolution(          const unsigned int          width, const unsigned int height);
        void   setSaturation(          const int                   value                           );
        void   setSharpness(           const int                   value                           );
        void   setContrast(            const int                   value                           );
        void   setBrightness(          const unsigned int          value                           );
        void   setISO(                 const int                   value                           );
        void   setVideoStabilization(  const int                   value                           );
        void   setExposureCompensation(const int                   value                           );
        void   setExposureMode(        const RASPICAM_EXPOSURE     mode                            );
        void   setExposureMetering(    const RASPICAM_METERING     metering                        );
        void   setShutterSpeed(        const int                   value                           );
        void   setAWBMode(             const RASPICAM_AWB          awb                             );
        void   setAWBGains(            const float                 gain_r, const float gain_b      );
        void   setRotation(            const int                   value                           );
        void   setHorizontalFlip(      const bool                  flip                            );
        void   setVerticalFlip(        const bool                  flip                            );
        void   setImageEncoding(       const RASPICAM_ENCODING     encoding                        );
        void   setImageQuality(        const unsigned int          value                           );
        void   setImageEffect(         const RASPICAM_IMAGE_EFFECT image_fx                        );
        void   setDRC(                 const RASPICAM_DRC          drc                             );

        // Getters
        bool                                   getRawMode(             void) const;
        int                                    getSensorMode(          void) const;
        std::pair<unsigned int,unsigned int>   getResolution(          void) const;
        int                                    getSaturation(          void) const;
        int                                    getSharpness(           void) const;
        int                                    getContrast(            void) const;
        unsigned int                           getBrightness(          void) const;
        int                                    getISO(                 void) const;
        int                                    getVideoStabilization(  void) const;
        int                                    getExposureCompensation(void) const;
        RASPICAM_EXPOSURE                      getExposureMode(        void) const;
        RASPICAM_METERING                      getExposureMetering(    void) const;
        int                                    getShutterSpeed(        void) const;
        RASPICAM_AWB                           getAWBMode(             void) const;
        std::pair<float,float>                 getAWBGains(            void) const;
        int                                    getRotation(            void) const;
        bool                                   getHorizontalFlip(      void) const;
        bool                                   getVerticalFlip(        void) const;
        RASPICAM_ENCODING                      getImageEncoding(       void) const;
        unsigned int                           getImageQuality(        void) const;
        RASPICAM_IMAGE_EFFECT                  getImageEffect(         void) const;
        RASPICAM_DRC                           getDRC(                 void) const;
    };
}
#endif
