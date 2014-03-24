#!/usr/bin/env bash

FILE=utils/src/version.c

MODIFIED=0

#Get the relevant informations
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
else
  # mercury
  ID=$(hg identify -nit)

  REV=$(echo -n $ID | cut -d' ' -f1)
  LOCAL=$(echo -n $ID | cut -d' ' -f2)
  TAG=$(echo -n $ID | cut -d' ' -f3)

  #LOCAL=$(hg identify -n)
  #REV=$(hg identify -i)
  #TAG=$(hg identify -t)
  #BRANCH=$(hg identify -b)

  if echo -n $REV | grep +\$>/dev/null; then
    MODIFIED=1
  fi
fi

#Patch version.c
sed -i "s/SLOCAL_REVISION=\".*\";\$/SLOCAL_REVISION=\"$LOCAL\";/" $FILE
sed -i "s/STAG=\".*\";\$/STAG=\"$TAG\";/" $FILE
sed -i "s/SREVISION=\".*\";\$/SREVISION=\"$REV\";/" $FILE
sed -i "s/MODIFIED=.*;\$/MODIFIED=$MODIFIED;/" $FILE

true
