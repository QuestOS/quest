#!/bin/bash

DIR="$1"
DESTDIR=drivers/acpica
INCLUDEDIR=include/acpi

[ -z "$DIR" -o ! -d "$DIR" ] && echo "Usage: $0 <dir>" && exit 1

echo "Processing directory: $DIR"

mkdir -p $DESTDIR
mkdir -p $INCLUDEDIR/platform

cp $DIR/dispatcher/*.c $DESTDIR
cp $DIR/events/*.c $DESTDIR
cp $DIR/executer/*.c $DESTDIR
cp $DIR/hardware/*.c $DESTDIR
cp $DIR/parser/*.c $DESTDIR
cp $DIR/namespace/*.c $DESTDIR
cp $DIR/utilities/*.c $DESTDIR
cp $DIR/tables/*.c $DESTDIR
cp $DIR/resources/*.c $DESTDIR
cp $DIR/include/*.h $INCLUDEDIR
cp $DIR/include/platform/acgcc.h $INCLUDEDIR/platform/
sed 's/#\(if defined(_LINUX) || defined(__linux__)\)/#if defined(_QUEST)\
#include "acquest.h"\
\
#el\1/' < $DIR/include/platform/acenv.h > $INCLUDEDIR/platform/acenv.h
