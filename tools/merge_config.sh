#!/usr/bin/env bash
# tools/merge_config.sh
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#

set -e

TOPDIR="${PWD}"

clean_up() {
	rm -f $TMP_FILE
	rm -f $MERGE_FILE
}

usage() {
	echo "Usage: $0 [OPTIONS] [[INITFILE] [PRE_CONFIG_LIST]]"
	echo "  -h    display this help"
	echo "  -r    list redundant entries when merging fragments"
	echo "  -O    dir to put generated output files."
	echo "  -f    shell file to define INITFILE and PRE_CONFIG_LIST"
	echo "  -s    strict mode. Fail if the fragment redefines any value."
	echo "   the command merge config list files into one"
	echo "      [INITFILE]: the frist file to be merged"
	echo "      [PRE_CONFIG_LIST]: the file list to be merged"
	echo
}

OUTPUT=.
CONFIG_PREFIX=${CONFIG_-CONFIG_}

while true; do
	case $1 in
	"-h")
		usage
		exit
		;;
	"-r")
		WARNREDUN=true
		shift
		continue
		;;
	"-O")
		if [ -d $2 ];then
			OUTPUT=$(echo $2 | sed 's/\/*$//')
		else
			echo "output directory $2 does not exist" 1>&2
			exit 1
		fi
		shift 2
		continue
		;;
	"-f")
		if [ -f $2 ];then
			CONFIG_FILE=$(echo $2 | sed 's/\/*$//')
		else
			echo "config file $2 does not exist" 1>&2
			exit 1
		fi
		shift 2
		continue
		;;
	"-s")
		STRICT=true
		shift
		continue
		;;
	*)
		break
		;;
	esac
done

if [ -f "$CONFIG_FILE" ];then
	source $CONFIG_FILE
else
	INITFILE=$1
	shift;
	if [ ! -r "$INITFILE" ]; then
		echo "The base file '$INITFILE' does not exist.  Exit." >&2
		exit 1
	fi
	PRE_CONFIG_LIST=$*
fi

if [ "$OUTPUT" != . ]; then
	KCONFIG_CONFIG=$(readlink -m -- "$OUTPUT/defconfig")
else
	KCONFIG_CONFIG=.config
fi

MERGE_LIST=$PRE_CONFIG_LIST
SED_CONFIG_EXP1="s/^\(${CONFIG_PREFIX}[a-zA-Z0-9_]*\)=.*/\1/p"
SED_CONFIG_EXP2="s/^# \(${CONFIG_PREFIX}[a-zA-Z0-9_]*\) is not set$/\1/p"

TMP_FILE=$(mktemp ./.tmp.config.XXXXXXXXXX)
MERGE_FILE=$(mktemp ./.merge_tmp.config.XXXXXXXXXX)

echo "Using $INITFILE as base"

trap clean_up EXIT

cat $INITFILE > $TMP_FILE

# Merge files, printing warnings on overridden values
for ORIG_MERGE_FILE in $MERGE_LIST ; do
	echo "Merging $ORIG_MERGE_FILE"
	if [ ! -r "$ORIG_MERGE_FILE" ]; then
		echo "The merge file '$ORIG_MERGE_FILE' does not exist.  Exit." >&2
		exit 1
	fi
	cat $ORIG_MERGE_FILE > $MERGE_FILE
	CFG_LIST=$(sed -n -e "$SED_CONFIG_EXP1" -e "$SED_CONFIG_EXP2" $MERGE_FILE)

	for CFG in $CFG_LIST ; do
		grep -q -w $CFG $TMP_FILE || continue
		PREV_VAL=$(grep -w $CFG $TMP_FILE)
		NEW_VAL=$(grep -w $CFG $MERGE_FILE)
		BUILTIN_FLAG=false
		if [ "x$PREV_VAL" != "x$NEW_VAL" ] ; then
			echo Value of $CFG is redefined by fragment $ORIG_MERGE_FILE:
			echo Previous  value: $PREV_VAL
			echo New value:       $NEW_VAL
			echo
			if [ "$STRICT" = "true" ]; then
				STRICT_MODE_VIOLATED=true
			fi
		elif [ "$WARNREDUN" = "true" ]; then
			echo Value of $CFG is redundant by fragment $ORIG_MERGE_FILE:
		fi
	done
	cat $MERGE_FILE >> $TMP_FILE
done

if [ "$STRICT_MODE_VIOLATED" = "true" ]; then
	echo "The fragment redefined a value and strict mode had been passed."
	exit 1
fi

cp -T -- "$TMP_FILE" "$KCONFIG_CONFIG"
echo "#"
echo "# Merged configuration written to $KCONFIG_CONFIG"
echo "#"
exit
