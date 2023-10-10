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