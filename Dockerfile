FROM nvidia/cuda:11.2.2-devel-ubuntu20.04

# Temporarily disable interactive installs
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt update \
    && apt install -yq \
        # Generic
        sudo \
        bash-completion \
        wget \
        build-essential \
        libyaml-cpp-dev \
        cmake \
        pkg-config \
        unzip \
        yasm \
        git \
        checkinstall \
        lsb-release \
        language-pack-en-base \
        # Python
        python3-dev \
        python3-pip \
        python3-pyqtgraph \
        # Image I/O
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        # Video
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libavresample-dev \
        libxvidcore-dev \
        x264 \
        libx264-dev \
        libfaac-dev \
        libmp3lame-dev \
        libtheora-dev  \
        libfaac-dev \
        libmp3lame-dev \
        libvorbis-dev \
        # GStreamer
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-0 \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-doc \
        gstreamer1.0-tools \
        gstreamer1.0-x \
        gstreamer1.0-alsa \
        gstreamer1.0-gl \
        gstreamer1.0-gtk3 \
        gstreamer1.0-qt5 \
        gstreamer1.0-pulseaudio \
        # Speech codecs
        libopencore-amrnb-dev \
        libopencore-amrwb-dev \
        # Camera interfaces
        libdc1394-22 \
        libdc1394-22-dev \
        libxine2-dev \
        libv4l-dev \
        v4l-utils \
        # Graphical
        libgtk-3-dev \
        freeglut3-dev \
        libnvidia-gl-460 \
        qtbase5-dev \
        # Parallelism library for CPU
        libtbb-dev \
        # Optimization libraries
        libatlas-base-dev \
        libblas-dev \
        liblapack-dev \
        libsuitesparse-dev \
        libcxsparse3 \
        gfortran \
        # Optional
        libprotobuf-dev \
        protobuf-compiler \
        libgoogle-glog-dev \
        libgflags-dev \
        libgphoto2-dev \
        libhdf5-dev \
        doxygen \
        ccache \
        libcanberra-gtk-module \
        libcanberra-gtk3-module \
    && apt autoremove -yq \
    && apt clean -yq \
    && rm -rf /var/lib/apt/lists/*

# Python
RUN pip3 install -U \
    pip \
    numpy \
    matplotlib

# Install Eigen
RUN wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz -O eigen.tar.gz \
    && tar -zxf eigen.tar.gz \
    && rm -rf eigen.tar.gz \
    #
    && mkdir eigen-3.3.9/build \
    && cd eigen-3.3.9/build \
    && cmake .. \
    && make install \
    #
    && cd ../.. \
    && rm -rf eigen-3.3.9

# Install Google Test
RUN git clone https://github.com/google/googletest.git \
    && cd googletest \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && cd ../.. \
    && rm -rf googletest

# Install Ceres-Solver
RUN wget -q https://github.com/ceres-solver/ceres-solver/archive/2.0.0.tar.gz -O ceres.tar.gz \
    && tar -zxf ceres.tar.gz \
    && rm -rf ceres.tar.gz \
    #
    && mkdir ceres-solver-2.0.0/build \
    && cd ceres-solver-2.0.0/build \
    && cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D EIGENSPARSE=ON \
        -D CXX11=ON \
        -D TBB=ON .. \
    && make -j$(nproc) install \
    #
    && cd ../.. \
    && rm -rf ceres-solver-2.0.0

# Install OpenCV
RUN wget -q https://github.com/opencv/opencv/archive/4.5.2.zip -O opencv.zip \
    && unzip -q opencv.zip \
    && rm -rf opencv.zip \
    #
    && wget -q https://github.com/opencv/opencv_contrib/archive/4.5.2.zip -O opencv_contrib.zip \
    && unzip -q opencv_contrib.zip \
    && rm -rf opencv_contrib.zip

RUN mkdir opencv-4.5.2/build \
    && cd /opencv-4.5.2/build \
    && cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local/opencv4.5.2 \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D INSTALL_C_EXAMPLES=OFF \
        -D PYTHON_DEFAULT_EXECUTABLE=$(which python3) \
        -D WITH_TBB=ON \
        -D WITH_CUDA=OFF \
        -D WITH_CUDNN=OFF \
        -D WITH_CUBLAS=1 \
        -D OPENCV_DNN_CUDA=OFF \
        -D ENABLE_FAST_MATH=1 \
        -D CUDA_FAST_MATH=1 \
        -D WITH_V4L=ON \
        -D WITH_QT=OFF \
        -D WITH_OPENGL=ON \
        -D WITH_VULKAN=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_OPENMP=ON \
        -D WITH_NVCUVID=ON \
        -D WITH_GDAL=ON \
        -D WITH_CLP=ON \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_PC_FILE_NAME=opencv.pc \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib-4.5.2/modules \
        -D BUILD_USE_SYMLINKS=ON \
        -D BUILD_PROTOBUF=ON \
        -D BUILD_EXAMPLES=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_TESTS=OFF \
        -D BUILD_JAVA=OFF \
        -D BUILD_opencv_java_bindings_gen=OFF \
        -D BUILD_opencv_cnn_3dobj=OFF \
        -D ENABLE_PRECOMPILED_HEADERS=OFF \
        -D BUILD_opencv_cudacodec=OFF \
        ..

RUN cd /opencv-4.5.2/build \
    && make -j $(nproc) install \
    && cd ../.. \
    && rm -rf opencv-4.5.2 opencv_contrib-4.5.2

# Enable interactive installs
ARG DEBIAN_FRONTEND=interactive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Add non root user
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && adduser $USERNAME sudo \
    && echo "user:user" | chpasswd

# Startup the user and bash
USER $USERNAME
ENV SHELL /bin/bash

# Show git branch in terminal
RUN echo "\n# Show git branch in terminal\nexport GIT_PS1_SHOWDIRTYSTATE=1" \
         "\nsource /usr/lib/git-core/git-sh-prompt" \
         "\nexport PS1='\\\[\\\033[01;36m\\\]\\\u\\\[\\\033[01;32m\\\]" \
         "\\\W\\\[\\\033[00;31m\\\]\$(__git_ps1)\\\[\\\033[01;36m\\\] \\\$\\\[\\\033[01;0m\\\] '" \
         >> ~/.bashrc
