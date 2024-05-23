FROM osrf/ros:humble-desktop

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
    && echo "source /opt/ros/galactic/setup.bash" >> /tmp/user/.bashrc

WORKDIR /tmp/user/iot_sim

# configure startup
COPY ./docker/usr/local/bin /usr/local/bin
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
