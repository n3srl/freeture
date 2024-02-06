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

Download and Install boost (from zip extract to a folder eg. C:\Program Files\boost\boost_1_82_0)
https://www.boost.org/doc/libs/1_84_0/more/getting_started/windows.html#get-boost

Download and install opencv (from zip extract to a folder eg. C:\Program Files\opencv)
https://opencv.org/releases/

git clone 
from the cloned repo folder

cd ..
mkdir freeture-project
cd freeture-project
cmake ../freeture

free to move binaries where you want but consider this.

https://gitlab.kitware.com/cmake/community/-/wikis/FAQ#what-is-an-out-of-source-build