#!/usr/bin/bash
# @brief    build *.ui

# echo $(dirname $(readlink -f $0))
uiFold=$(dirname $(readlink -f $0))

# list all the *.ui file
echo "uic files:"
for file in $(ls $uiFold/resource)
do
    if [ "${file##*.}" = "ui" ]; then
        # mv ${file} ${file%.*}.c
        echo ${uiFold}/resource/${file}
        uic ${uiFold}/resource/${file} -o ${uiFold}/temp/${file%.*}_uic.h
    fi
done

# moc all the *.h file
echo "moc files:"
for file in $(ls $uiFold/src)
do
    if [ "${file##*.}" = "h" ]; then
        echo ${uiFold}/src/${file}
        moc ${uiFold}/src/${file} -o ${uiFold}/temp/${file%.*}_moc.cpp
    fi
done

