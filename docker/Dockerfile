FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-pip \
    wget \
    unzip \
    apt-utils \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install requests

# system configuration
RUN chmod o+w /etc \
    && chmod o+w /etc/group \
    && chmod o+w /etc/passwd \
    && chmod o+rw /etc/shadow \
    && echo "Defaults !env_reset" >> /etc/sudoers \
    && echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# create and configure local user
RUN groupadd -g 1000 user \
    && useradd -u 1000 -g user -G sudo -s /bin/bash -d /tmp/user -m -l user

RUN chown -R 1000:1000 /tmp/user \
    && echo "alias ..='cd ..'" >> /tmp/user/.bashrc \
    && echo "export PATH=/tmp/user/.local/bin:$PATH" >> /tmp/user/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /tmp/user/.bashrc

WORKDIR /tmp/user/iot_sim

RUN mkdir data
RUN wget --content-disposition 'https://api.ngc.nvidia.com/v2/resources/org/nvidia/team/isaac/r2bdataset2023/2/files?redirect=true&path=r2b_hope/r2b_hope.db3' -O data/r2b_hope.db3

# configure startup
COPY ./docker/usr/local/bin /usr/local/bin
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
