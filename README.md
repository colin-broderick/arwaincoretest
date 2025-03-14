## Project overview

### Build
The primary build target is a custom Linux environment built using Yocto to run on a custom compute platform.
1. Prepare an appropriate Yocto environment for you build target.
2. Clone `meta-arwain` and `meta-greeve-bsp` into `sources`.
3. Add `${BSPDIR}/sources/meta-arwain` to `conf/bblayers.conf`.
3. Specify an appropriate machine name (from `meta-greeve-bsp`) in `conf/local.conf`.
4. Run `bitbake arwain-inference-core` to build the application only.
5. Run `bitbake arwain-tag-image` or `bitbake arwain-tag-image-dev` to build a full image.

The project can also be built on any standard Linux machine provided dependencies are available.

# Dependencies

Most dependencies are managed by the CMake project. These additional apt packages are required:

* nlohmann-json3-dev
* libgtest-dev
* libboost-all-dev
* libssl-dev
* libi2c-dev
* libeigen3-dev
