#!/bin/bash
if [ "$1" -lt "$2" ]
then
	FILE="../launch/generated.launch"
	echo "<!-- This launch file was automatically generated. -->" > $FILE
	echo "<launch>" >> $FILE
	for i in `seq $1 $2`
	do
		echo '	<include file="$(find wta_demo)/launch/distributed_sim.launch">' >>$FILE
		echo "		<arg name=\"name\" value=\"robot$i\"/>" >> $FILE
		echo '		<arg name="x" value="-1.0" />' >> $FILE
		echo '		<arg name="y" value="0.0" />' >> $FILE
		echo '	</include>' >> $FILE
	done
	echo "</launch>" >> $FILE
else
	echo "Second parameter is smaller than the first"
fi

#roslaunch wta_demo generated.launch &
