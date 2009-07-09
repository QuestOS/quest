#!/bin/bash

DIR="$1"
[ -z "$DIR" -o ! -d "$DIR" ] && echo "Usage: $0 <dir>" && exit 1

echo "Processing directory: $DIR"

mkdir -p acpi/include/platform

cp $DIR/dispatcher/*.c acpi/
cp $DIR/events/*.c acpi/
cp $DIR/executer/*.c acpi/
cp $DIR/hardware/*.c acpi/
cp $DIR/parser/*.c acpi/
cp $DIR/namespace/*.c acpi/
cp $DIR/utilities/*.c acpi/
cp $DIR/tables/*.c acpi/
cp $DIR/resources/*.c acpi/
cp $DIR/include/*.h acpi/include/
cp $DIR/include/platform/acgcc.h acpi/include/platform/
sed 's/#\(if defined(_LINUX) || defined(__linux__)\)/#if defined(_QUEST)\
#include "acquest.h"\
\
#el\1/' < $DIR/include/platform/acenv.h > acpi/include/platform/acenv.h




