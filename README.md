# Description

This project provides a simple peephole surveillance camera application using Orange Pi Zero.

It has following features:
1. H264 encoding using Allwinner's SOC H/W H.264 encoder
2. RTMP streaming : you can push video stream to an rtmp server
3. Motion detection : recoding for 5s when motion detected
4. [TODO] Face recognition


#  Compiling

It tested on Armbian OS. Clone this repository, and change the directory to the root of this repository.

### 1. Cedarx configuration
Cedarx 'blobs' contains shared libraries for H264 hard encoding, so we have to install it correctly first. The following commands should run as root.
- Extract shared libraries in `cedarx_blobs.tar.gz`
```shell
cd /
tar xzvf [root_of_git_repo]/blobs/cedarx_blobs.tar.gz
```
- Config the `.so` libraries:
```shell
nano "/etc/ld.so.conf.d/cedarx.conf"
```
and add the content below
```shell
/usr/local/lib/cedarx
```
- Once you save `cedarx.conf`, run:
```shell
ldconfig
```

### 2. Build libyuv
Just run following commands:
```shell
cd libyuv/;
make -f linux.mk CXX="g++ -mfpu=neon" libyuv.a;
cd ../;
```



# Acknowledgements
I borrowed tons of codes from following projects, thanks for their efforts.

1. [videoenc](https://github.com/rosimildo/videoenc)
2. [simplest_librtmp_example](https://github.com/leixiaohua1020/simplest_librtmp_example)
