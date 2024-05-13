FROM ros:humble


# Install some tools and libraries.
RUN apt-get update && apt-get -y install \
    vim wget curl unzip \
    build-essential \
    cmake \
    make ninja-build \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    libceres-dev \
    && rm -rf /var/lib/apt/lists/*

COPY startup.sh /startup.sh
COPY build /workspaces/Engineer_Visual/deploy/build
COPY install /workspaces/Engineer_Visual/deploy/install

RUN chmod +x /startup.sh

CMD ["/startup.sh"]