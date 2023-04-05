#!/bin/bash
BASEDIR=$(dirname "$0")
ARENASDK_ROOT="/opt/arenasdk/"

if [ ! -d $ARENASDK_ROOT ] 
then
	mkdir $ARENASDK_ROOT
fi

ARENALIB=$ARENASDK_ROOT"lib64"
GENICAMLIB=$ARENASDK_ROOT"GenICam/library/lib/Linux64_x64"
FFMPEGLIB=$ARENASDK_ROOT"ffmpeg"

cp -R $BASEDIR/include $ARENASDK_ROOT
cp -R $BASEDIR/lib64 $ARENASDK_ROOT
cp -R $BASEDIR/GenICam $ARENASDK_ROOT/GenICam
cp -R $BASEDIR/ffmpeg $ARENASDK_ROOT/ffmpeg

echo -e $ARENALIB > arenasdk.conf
echo -e $GENICAMLIB >> arenasdk.conf
echo -e $FFMPEGLIB >> arenasdk.conf

cp arenasdk.conf /etc/ld.so.conf.d/
chmod 775 $ARENALIB $GENICAMLIB $FFMPEGLIB
ldconfig
