### FreeTure
--------

*A Free software to capTure meteors*

`FreeTure` is a free open source (GPL license) meteor detection software used to monitor the sky with GigE all-sky cameras to detect and record falling stars and fireball.

It is portable and cross-platform (Linux, Windows) (not anymore!)

The project home page is http://fripon.org

Features
--------

- Support GigE cameras (Tested with Basler acA1300-30gm and DMK23G445)
- Support usb 2.0 cameras (Tested with stk1160 and Pinnacle Dazzle DVC 100 video grabber, DMx 31AU03.AS)
- Internal computation of sun ephemeris
- Night and daytime (experimental) meteor detection modes
- Fits format in output https://en.wikipedia.org/wiki/FITS
- Possibility to run regular or scheduled long exposure acquisition
- Possibility to stack frames in order to keep a kind of history

How to run CMake
----------------
EXAMPLE ON WINDOWS


DEPENDENCIES

BOOST
debian:bullseye version is 1.74
Download and Install boost (from zip extract to a folder eg. C:\Program Files\boost\boost_1_74_0)
https://sourceforge.net/projects/boost/files/boost-binaries/1.74.0/boost_1_74_0-msvc-14.2-64.exe/download
will be installed on C:\local\boost_1_74_0

OPEN CV
debian:bullseye version is 4.5.1
Download and install opencv (from zip extract to a folder eg. C:\Program Files\opencv)
https://opencv.org/releases/

https://sourceforge.net/projects/opencvlibrary/files/4.5.1/

OPEN SSL
debian:bullseye version is 1.1.1
Downloading OpenSSL from https://slproweb.com/products/Win32OpenSSL.html (Win64 OpenSSL v1.1.1w Light)

LIB ARAVIS
debian:bullseye version is 0.8

Install Conan
https://docs.conan.io/2/installation.html

Download and install meson, follow instructions for python3
https://mesonbuild.com/Getting-meson.html

Installare GTK
https://github.com/wingtk/gvsbuild

CFITSIO
download latest
follow these instructions:
https://heasarc.gsfc.nasa.gov/FTP/software/fitsio/c/README.win

Otherwise get latest from jenkins.

extract in %PROGRAMFILES%\cfits




BUILD freeture
git clone https://github.com/n3srl/freeture

cd ..
mkdir freeture-project
cd freeture-project
cmake ../freeture

free to move binaries where you want but consider this.

https://gitlab.kitware.com/cmake/community/-/wikis/FAQ#what-is-an-out-of-source-build

or use cmake-gui