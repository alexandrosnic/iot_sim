#!/usr/bin/env sh
set -e

# change uid and gid of local user (if specified with --user)
UID=$(id -u)
GID=$(id -g)
if [ ${UID} -ne 0 -a ${GID} -ne 0 ]; then
    sed -i "s/1000:1000/${UID}:${GID}/" /etc/passwd
    sed -i "s/1000/${GID}/" /etc/group
    sudo find /tmp/user -uid 1000 -and -gid 1000 -exec chown ${UID}:${GID} '{}' \;
fi

# change username of local user (if specified with --env USER)
USER="${USER:-user}"
if [ "${USER}" != "root" ]; then
    sed -i "s/\buser\b/${USER}/" /etc/passwd
    sed -i "s/\buser\b/${USER}/" /etc/group
    sed -i "s/\buser\b/${USER}/" /etc/shadow
fi

# add groups provided on command-line
for gid in $(id -G); do
    sudo groupadd -f -g ${gid} group${gid}
    sudo usermod -a -G ${gid} "${USER}"
done

# call command
exec sudo -HPsu "${USER}" "$@"
