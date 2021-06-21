## Purpose

This repository is for development and testing of the ARWAIN C++ rewrite.

It has been written using only the C++ standard library so should be portable to various hardware with very little work.

It is almost feature-complete but no features have been formally tested. Missing feautres are

1. NPU inference. RKNN is written and functions but there are outstanding questions about shaping data to get correct inference result.

2. LoRa communication. Had some trouble getting the C++ rewrite to work. This also depends on SPI which has not yet been worked out for the Rockchip board.

## Build

### Requirements

2. Generate buildroot compilcation toolchain:
```
sudo apt install cmake git-core gitk git-gui gcc-arm-linux-gnueabihf u-boot-tools device-tree-compiler gcc-aarch64-linux-gnu mtools parted libudev-dev libusb-1.0-0-dev python-linaro-image-tools linaro-image-tools autoconf autotools-dev libsigsegv2 m4 intltool libdrm-dev curl sed make binutils build-essential gcc g++ bash patch gzip bzip2 perl tar cpio python unzip rsync file bc wget libncurses5 libqt4-dev libglib2.0-dev libgtk2.0-dev libglade2-dev cvs git mercurial openssh-client subversion asciidoc w3m dblatex graphviz python-matplotlib libc6:i386 liblz4-tool
mkdir ~/bin
mkdir ~/buildroot
curl https://storage.googleapis.com/git-repo-downloads/repo-1 > ~/bin/repo
chmod a+x ~/bin/repo
cd buildroot
~/bin/repo init --repo-url http://github.com/aosp-mirror/tools_repo.git -u https://github.com/96boards-tb-96aiot/manifest.git
~/bin/repo sync
cd ~/buildroot/u-boot
./make.sh rk1808
cd ~/buildroot/kernel
make rk1808_linux_defconfig 
make rk1808_evb_v10.img
16 | source ~/buildroot/buildroot/build/envsetup.sh
make
./mkfirmware.sh
```
I wouldn't try to run this all at once since some parts are very long running and you might get into trouble if something fails or has changed.

If any of this is out of date or depricated, Colin Broderick has a working VM with the toolchain already prepared.


### Building
To build for the RK1808:
```
cd ~/arwain_inference_core
mkdir build lib
cmake -DCMAKE_CXX_COMPILER=/path/to/buildroot/output/rockchip_rk1808/host/bin/aarch64-linux-g++ .
make -j4
```

## Run
Use `scp` or preferred method to get the `arwain` binary onto the Rockchip board. You will need a valid `arwain.conf` or it won't run. The default one from this repo will work.

To run with just console output
```
./path/to/arwain -lstd
```
For other options run
```
./path/to/arwain -h
```
Code currently assumes running from the directory where the config file is found, and where `data_` folders should be created. If you want to run from somewhere else you'll need to pass the `-conf` option to specify the location of the configuration file.
