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
After cedarx configured and libyuv compiled, just type `make` under the root directory of this repository, and you'll get a program named `peephole_camera`.

# Usage

To run `peephole_camera`, make sure you meet following requirements:

1. USB camera plugged into Orange Pi Zero, it should be `/dev/video0` by default.
2. It trys to connect a local rtmp server by default once started, or you can specify another rtmp server by command arguments.

Here is all arguments you can specify in this program:
```
--input,-i        - input source           ( default = [-] pipe source                 )
--rtmp-url,-c     - rtmp server url        ( default = rtmp://192.168.2.114:1935/live  )
--bitrate,-b      - bit rate in Kbits/Sec  ( default = 1024                            )
--fps,-r          - frames per second      ( default = 25                              )
--size,-s         - video source size      ( default = 640x480                         )
--qmin-max,-q     - encoding quality       ( default = -q 20,30                        )
--key-int,-k      - key frame interval     ( default = 5                               )
```

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

#### Build RTMPdump(libRTMP)
- Install `openssl`

We choose `openssl-1.0.1f` for compability with `libRTMP`. Following commands show how to install and configure it.

``` bash
# fisrt create soft link in /usr/local
$ cd /usr/local
$ sudo ln -s openssl ssl
$ cd
# download openssl-1.0.1f to your home
$ wget https://www.openssl.org/source/old/1.0.1/openssl-1.0.1f.tar.gz
$ tar -xvf openssl-1.0.1f.tar.gz
$ cd openssl-1.0.1f
$ ./config --prefix=/usr/local/openssl
$ ./config -t
# it may take a while to build it
$ make depend
$ sudo make install
```

Add the following content to `/etc/ld.so.conf`:
```bash
/usr/local/openssl/lib
```
and then run
```bash
sudo ldconfig
```

After that, you should configure enviroment variable for `openssl`, just add following to the end of your `~/.bashrc`:
```bash
export OPENSSL=/usr/local/openssl/bin
export PATH=$OPENSSL:$PATH:$HOME/bin
```
and then run
```bash
source ~/.bashrc
```
After all these done, we have installed openssl, and now to check if we have some mistakes.
```bash
$ ldd /usr/local/openssl/bin/openssl
    # you'll get outputs like this
    libdl.so.2 => /lib/arm-linux-gnueabihf/libdl.so.2 (0xb6eee000)
    libc.so.6 => /lib/arm-linux-gnueabihf/libc.so.6 (0xb6e02000)
    /lib/ld-linux-armhf.so.3 (0xb6f18000)
$ which openssl
/usr/local/openssl/bin/openssl
$ openssl version
OpenSSL 1.0.1f 6 Jan 2014
```
Ok, it's all right.

- Install `zlib`

It's simple to install zlib.
```bash
$ wget https://zlib.net/zlib-1.2.11.tar.gz
$ tar xvf zlib-1.2.11.tar.gz
$ cd zlib-1.2.11/
$ sudo ./configure
$ make
$ sudo make install
```

- Install `libssl-dev`

We can just use `apt` to install it.
```bash
$ sudo apt-get install libssl-dev
```

- Build `libTRMP`

We should add the search path of `openssl` head files, add the following content to the end of `~/.bashrc`
```bash
C_INCLUDE_PATH=/usr/local/openssl/include/
export C_INCLUDE_PATH
CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/usr/local/openssl/include/
export CPLUS_INCLUDE_PATH
```

And finally we can download `rtmpdump-2.3` and to compile it.
```bash
$ wget http://rtmpdump.mplayerhq.hu/download/rtmpdump-2.3.tgz
$ tar -xvf  rtmpdump-2.3.tgz
$ cd rtmpdump-2.3
$ make
$ sudo make install
```





# Acknowledgements
I borrowed tons of codes from following projects, thanks for their efforts.

1. [videoenc](https://github.com/rosimildo/videoenc)
2. [simplest_librtmp_example](https://github.com/leixiaohua1020/simplest_librtmp_example)
