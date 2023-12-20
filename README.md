## Project overview

### Build
The primary build target is a custom Linux environment built using Yocto to run on a custom compute platform.
1. Prepare an appropriate Yocto environment for you build target.
2. Clone `meta-arwain` into `sources`.
3. Add `${BSPDIR}/sources/meta-arwain` to `bblayers.conf`.
4. Run `bitbake arwain-inference-core` to build the application only.
5. Run `bitbake arwain-core` or `bitbake arwain-core-dev` to build a full image.

The project can also be built on any standard Linux machine provided dependencies are available. Key dependencies are libi2c, spidev, Eigen, gtest, and NumCpp.

# Dependencies

## Arwain/Greeve libraries

input_parser 0.1
arwain_logger 
arwain_event
arwain_math 0.1
iim42652

arwain/vector3.hpp
arwain/devices/bmp384.hpp
arwain/devices/iim42652.hpp
arwain/logger.hpp
arwain/input_parser.hpp
arwain/timers.hpp
arwain/devices/lis3mdl.hpp
arwain/devices/rfm95w.hpp
arwain/orientation/madgwick.hpp
arwain/event_manager.hpp
arwain/config_parser.hpp

## Third party libraries

libeigen3

libeigen3
NumCpp
yaml-cpp