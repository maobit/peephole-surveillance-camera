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
$ cd /
$ tar xzvf [root_of_git_repo]/blobs/cedarx_blobs.tar.gz
```
- Config the `.so` libraries:
```shell
$ sudo vim /etc/ld.so.conf.d/cedarx.conf
```
and add the content below
```shell
/usr/local/lib/cedarx
```
- Once you save `cedarx.conf`, run:
```shell
$ sudo ldconfig
```

### 2. Build libyuv
Just run following commands:
```shell
$ cd libyuv/;
$ make -f linux.mk CXX="g++ -mfpu=neon" libyuv.a;
$ cd ../;
```

### 3. Build this repository
After cedarx configured and libyuv compiled, just type `make` under the root directory of this repository.

# Usage
It trys to connect a local rtmp server by default once started, or you can specify another rtmp server by command arguments.

And here is a simple tutorial to build a rtmp server using nginx.
#### Using nginx to build an rtmp streaming server
- Download `PCRE` and install it

```shell
$ wget ftp://ftp.csx.cam.ac.uk/pub/software/programming/pcre/pcre-8.38.tar.gz
$ tar zxvf pcre-8.38.tar.gz
$ cd pcre-8.38
$ ./configure
$ make
$ make install (if not the root user, please use the sudo)
```
- Download `nginx` and `nginx rtmp` and then install it

```shell
# download nginx
$ wget http://nginx.org/download/nginx-1.10.0.tar.gz  
# uncompress nginx
$ tar -zxvf nginx-1.10.0.tar.gz
# download nginx-rtmp-module
$ git clone https://github.com/arut/nginx-rtmp-module.git
# configure nginx
$ ./configure --add-module=/path/to/nginx-rtmp-module
$ make
$ make install
```
- Supporting RTMP

```
$ sudo vi conf/nginx.conf
```
add following configuration
```
rtmp {
 server {
     listen 1935;
     application live {
         live on;
         record off;
     }
 }
}
```
- Start nginx

```
$ sudo /usr/local/sbin/nginx
```



# Acknowledgements
I borrowed tons of codes from following projects, thanks for their efforts.

1. [videoenc](https://github.com/rosimildo/videoenc)
2. [simplest_librtmp_example](https://github.com/leixiaohua1020/simplest_librtmp_example)
