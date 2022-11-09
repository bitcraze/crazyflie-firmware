#!/bin/bash
set -ex

DOCKER=docker

# Prefer running with podman
if which podman
then
    DOCKER=podman
fi

# Test if we are currently in the builder image
if [ ! -d /opt/python ]
then
    # We are not in the image, start the image with the build script
    $DOCKER run --rm -v $(realpath $(dirname $0)/../..):/io quay.io/pypa/manylinux2014_x86_64 /io/tools/build/build-wheels.sh
else
    # We are in the image, building!
    yum install -y libusbx-devel

    cd /io

    PREV_PATH=$PATH

    for PYBIN in /opt/python/cp{36,37,38,39}*/bin; do
        PATH=${PYBIN}:$PREV_PATH
        rm -rf build/
        "${PYBIN}/pip" install -U "setuptools>=42" wheel ninja "cmake>=3.12"
        "${PYBIN}/python" bindings/setup.py bdist_wheel
    done

    WHEELS=$(echo dist/*.whl)

    for whl in $WHEELS; do
        auditwheel repair "$whl" -w dist/
    done

    rm $WHEELS
fi
