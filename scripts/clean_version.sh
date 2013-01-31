#!/usr/bin/env bash

FILE=utils/src/version.c

#Patch version.c to delete the versions informations
sed -i "s/SLOCAL_REVISION=\".*\";\$/SLOCAL_REVISION=\"\";/" $FILE
sed -i "s/STAG=\".*\";\$/STAG=\"\";/" $FILE
sed -i "s/SREVISION=\".*\";\$/SREVISION=\"\";/" $FILE
sed -i "s/MODIFIED=.*;\$/MODIFIED=1;/" $FILE

true
