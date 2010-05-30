#!/bin/bash

DIR="$1"
DESTDIR=lwip
INCLUDEDIR=include

[ -z "$DIR" -o ! -d "$DIR" ] && echo "Usage: $0 <dir>" && exit 1

echo "Processing directory: $DIR"

mkdir -p $DESTDIR/core
mkdir -p $DESTDIR/netif
mkdir -p $INCLUDEDIR/lwip
mkdir -p $INCLUDEDIR/netif

cp -a $DIR/src/api $DESTDIR
cp -a $DIR/src/core/ipv4 $DESTDIR/core
cp $DIR/src/core/*.c $DESTDIR/core
cp $DIR/src/netif/*.c $DESTDIR/netif
cp $DIR/src/include/ipv4/lwip/*.h $INCLUDEDIR/lwip
cp $DIR/src/include/lwip/*.h $INCLUDEDIR/lwip
cp $DIR/src/include/netif/*.h $INCLUDEDIR/netif
