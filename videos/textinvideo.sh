#!/bin/bash

if [ $# -eq 0 ]; then
	echo usage: $0 video [ text [ output ]]
elif [ $# -eq 1 ]; then

	avconv -r 25 -i /tmp/robdyn_%05d.bmp $1

elif [ $# -gt 1 ]; then
	if [ $# -eq 2 ]; then
		OUTPUT=$1
	else
		OUTPUT=$3
	fi
	if [ $# -gt 3 ]; then
		SIZE=$4
	else
		SIZE=32
	fi
	if [ $# -eq 6 ]; then
		X=$5
		Y=$6
	else
		X=45
		Y=470
	fi

	if test ! -f $1; then
		INPUT=$(mktemp tmp_XXXXX).mp4
		avconv -r 25 -i /tmp/robdyn_%05d.bmp $INPUT
		tmp_file=1
	elif [ $OUTPUT = $1 ]; then
		INPUT=$(mktemp tmp_XXXXX).mp4
		mv $1 $INPUT
		tmp_file=1
	else
		INPUT=$1
		tmp_file=0
	fi

	avconv -i $INPUT -vf drawtext="fontfile=/usr/share/fonts/truetype/freefont/FreeSansBold.ttf: \
	text=$2: fontcolor=black: fontsize=$SIZE: \
	x=$X: y=$Y" $OUTPUT

	if [ $tmp_file -eq 1 ]; then
		rm $INPUT
	fi

fi
