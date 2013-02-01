#!/usr/bin/env bash

ID=$(hg identify -nit)

REV=$(echo -n $ID | cut -d' ' -f1)
LOCAL=$(echo -n $ID | cut -d' ' -f2)
TAG=$(echo -n $ID | cut -d' ' -f3)

#LOCAL=$(hg identify -n)
#REV=$(hg identify -i)
#TAG=$(hg identify -t)

echo -n Build $LOCAL:$REV \($TAG\) 

if echo -n $REV | grep +\$>/dev/null; then
  echo -e " \033[1;31mMODIFIED\033[m"
else
  echo -e " \033[1;32mCLEAN\033[m"
fi
