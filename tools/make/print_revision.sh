#!/usr/bin/env bash

MODIFIED=0

if test -d .git ; then
  # git
  ID=$(git describe --tags --abbrev=12 HEAD)
  REV=${ID##*-}
  REV=${REV:1}
  LOCAL=${ID#*-*-}
  TAG=${ID%-$LOCAL}
  LOCAL=${LOCAL%-*}
  git update-index -q --refresh
  if ! test -z "$(git diff-index --name-only HEAD --)" ; then
    MODIFIED=1
  fi
elif test -d .hg ; then
  # mercury
  ID=$(hg identify -nit)

  REV=$(echo -n $ID | cut -d' ' -f1)
  LOCAL=$(echo -n $ID | cut -d' ' -f2)
  TAG=$(echo -n $ID | cut -d' ' -f3)

  #LOCAL=$(hg identify -n)
  #REV=$(hg identify -i)
  #TAG=$(hg identify -t)
  if echo -n $REV | grep +\$>/dev/null; then
    MODIFIED=1
  fi
else
    LOCAL="Tarball build"
    REV=" Build from GIT tree for accurate versioning"
    TAG=`pwd | grep -o "20[0-9][0-9]\\.[0-9][0-9]\?\(\\.[0-9][0-9]\?\)\?$"`
    MODIFIED=1
fi

echo -n Build $LOCAL:$REV \($TAG\) 

if test $MODIFIED -eq 1 ; then
  echo -e " \033[1;31mMODIFIED\033[m"
else
  echo -e " \033[1;32mCLEAN\033[m"
fi
