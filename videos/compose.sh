#!/bin/bash

# rock_step_3:
#end=650
# rock_step_3_bis:
#end=485
# groove:
end=683

for i in $(seq -f "%05g" 0 $end); do
	printf "\rProgression: %d%%" $( bc -q <<< $i*100/$end )
	composite -gravity south \( /tmp/ia_plot_$i.png -resize 1024x \) /tmp/robdyn_$i.bmp /tmp/temp_$i.png
	#composite -geometry +0+50 \( /tmp/internal_actuation_$i.png -resize 1024x \) /tmp/robdyn_$i.bmp /tmp/temp_$i.png
done

#avconv -r 25 -i /tmp/temp_%05d.png video_output.mp4
ffmpeg -r 25 -i /tmp/temp_%05d.png video_output.mp4
