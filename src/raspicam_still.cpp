/**
 * @file   raspicam_still.h
 * @author Jose Nogueira, josenogueira@biosurfit.com
 * @date   May 2017
 * @brief  RaspberryPi Camera for Still capture
 *
 * (extensive explanation)
 */

/**
 * Include files
 */

#include "raspicam_still.h"
#include "private_still/private_still_impl.h"

/**
 * Namespace raspicam
 */
namespace raspicam
{

/**
 * Constructors and destructors
 */
RaspiCam_Still::RaspiCam_Still( void) { _impl = new _private::Private_Impl_Still(); }
RaspiCam_Still::~RaspiCam_Still(void) { this->release(); delete _impl; }


/**
 * Camera Workflow
 */
bool   RaspiCam_Still::open(   void) { return (_impl->initialize() == 0); }
bool   RaspiCam_Still::release(void) { return (_impl->release()    == 0); }

size_t RaspiCam_Still::getImageBufferSize(void         ) const   { return _impl-> getImageBufferSize(); }
int    RaspiCam_Still::captureFrame(      unsigned char*  data    ,
                                          const size_t    length  ,
                                          size_t&         offset  ) { return _impl->takePicture(   data, length, offset, false); }
int    RaspiCam_Still::captureFrameRaw(   unsigned char*  data    ,
                                          const size_t    length  ,
                                          unsigned char** data_raw) { return _impl->takePictureRaw(data, length, data_raw, false); }


/**
 * Camera configuration
 */
int RaspiCam_Still::commitParameters(void) { return _impl->commitParameters(); }
// Setters
void   RaspiCam_Still::setRawMode(             const bool                  value                           )
              { _impl->setRawMode(                                         value                           ); }
void   RaspiCam_Still::setSensorMode(          const int                   value                           )
              { _impl->setSensorMode(                                      value                           ); }
void   RaspiCam_Still::setResolution(          const unsigned int          width, const unsigned int height)
              { _impl->setResolution(                                      width,                    height); }
void   RaspiCam_Still::setSaturation(          const int                   value                           )
              { _impl->setSaturation(                                      value                           ); }
void   RaspiCam_Still::setSharpness(           const int                   value                           )
              { _impl->setSharpness(                                       value                           ); }
void   RaspiCam_Still::setContrast(            const int                   value                           )
              { _impl->setContrast(                                        value                           ); }
void   RaspiCam_Still::setBrightness(          const unsigned int          value                           )
              { _impl->setBrightness(                                      value                           ); }
void   RaspiCam_Still::setISO(                 const int                   value                           )
              { _impl->setISO(                                             value                           ); }
void   RaspiCam_Still::setVideoStabilization(  const int                   value                           )
              { _impl->setVideoStabilization(                              value                           ); }
void   RaspiCam_Still::setExposureCompensation(const int                   value                           )
              { _impl->setExposureCompensation(                            value                           ); }
void   RaspiCam_Still::setExposureMode(        const RASPICAM_EXPOSURE     mode                            )
              { _impl->setExposureMode(                                    mode                            ); }
void   RaspiCam_Still::setExposureMetering(    const RASPICAM_METERING     metering                        )
              { _impl->setExposureMetering(                                metering                        ); }
void   RaspiCam_Still::setShutterSpeed(        const int                   value                           )
              { _impl->setShutterSpeed(                                    value                           ); }
void   RaspiCam_Still::setAWBMode(             const RASPICAM_AWB          awb                             )
              { _impl->setAWBMode(                                         awb                             ); }
void   RaspiCam_Still::setAWBGains(            const float                 gain_r, const float       gain_b)
              { _impl->setAWBGains(                                        gain_r,                   gain_b); }
void   RaspiCam_Still::setRotation(            int                         value                           )
              { _impl->setRotation(                                        value                           ); }
void   RaspiCam_Still::setHorizontalFlip(      const bool                  flip                            )
              { _impl->setHorizontalFlip(                                  flip                            ); }
void   RaspiCam_Still::setVerticalFlip(        const bool                  flip                            )
              { _impl->setVerticalFlip(                                    flip                            ); }
void   RaspiCam_Still::setImageEncoding(       const RASPICAM_ENCODING     encoding                        )
              { _impl->setImageEncoding(                                   encoding                        ); }
void   RaspiCam_Still::setImageQuality(        const unsigned int          value                           )
              { _impl->setImageQuality(                                    value                           ); }
void   RaspiCam_Still::setImageEffect(         const RASPICAM_IMAGE_EFFECT image_fx                        )
              { _impl->setImageEffect(                                     image_fx                        ); }
void   RaspiCam_Still::setDRC(                 const RASPICAM_DRC          drc                             )
              { _impl->setDRC(                                             drc                             ); }
// Getters
bool                                   RaspiCam_Still::getRawMode(             void) const
                                       { return _impl->getRawMode(             ); }
int                                    RaspiCam_Still::getSensorMode(          void) const
                                       { return _impl->getSensorMode(          ); }
std::pair<unsigned int,unsigned int>   RaspiCam_Still::getResolution(          void) const
                                       { return _impl->getResolution(          ); }
int                                    RaspiCam_Still::getSaturation(          void) const
                                       { return _impl->getSaturation(          ); }
int                                    RaspiCam_Still::getSharpness(           void) const
                                       { return _impl->getSharpness(           ); }
int                                    RaspiCam_Still::getContrast(            void) const
                                       { return _impl->getContrast(            ); }
unsigned int                           RaspiCam_Still::getBrightness(          void) const
                                       { return _impl->getBrightness(          ); }
int                                    RaspiCam_Still::getISO(                 void) const
                                       { return _impl->getISO(                 ); }
int                                    RaspiCam_Still::getVideoStabilization(  void) const
                                       { return _impl->getVideoStabilization(  ); }
int                                    RaspiCam_Still::getExposureCompensation(void) const
                                       { return _impl->getExposureCompensation(); }
RASPICAM_EXPOSURE                      RaspiCam_Still::getExposureMode(        void) const
                                       { return _impl->getExposureMode(        ); }
RASPICAM_METERING                      RaspiCam_Still::getExposureMetering(    void) const
                                       { return _impl->getExposureMetering(    ); }
int                                    RaspiCam_Still::getShutterSpeed(        void) const
                                       { return _impl->getShutterSpeed(        ); }
RASPICAM_AWB                           RaspiCam_Still::getAWBMode(             void) const
                                       { return _impl->getAWBMode(             ); }
std::pair<float,float>                 RaspiCam_Still::getAWBGains(            void) const
                                       { return _impl->getAWBGains(            ); }
int                                    RaspiCam_Still::getRotation(            void) const
                                       { return _impl->getRotation(            ); }
bool                                   RaspiCam_Still::getHorizontalFlip(      void) const
                                       { return _impl->getHorizontalFlip(      ); }
bool                                   RaspiCam_Still::getVerticalFlip(        void) const
                                       { return _impl->getVerticalFlip(        ); }
RASPICAM_ENCODING                      RaspiCam_Still::getImageEncoding(       void) const
                                       { return _impl->getImageEncoding(       ); }
unsigned int                           RaspiCam_Still::getImageQuality(        void) const
                                       { return _impl->getImageQuality(        ); }
RASPICAM_IMAGE_EFFECT                  RaspiCam_Still::getImageEffect(         void) const
                                       { return _impl->getImageEffect(         ); }
RASPICAM_DRC                           RaspiCam_Still::getDRC(                 void) const
                                       { return _impl->getDRC(                 ); }

} // End of namespace raspicam
