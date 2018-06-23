#!/bin/bash
if [ "$1" -lt "$2" ]
then
	FILE="../launch/generatedMaster.launch"
	echo "<!-- This launch file was automatically generated. -->" > $FILE
	echo "<launch>" >> $FILE
	echo '	<include file="$(find wta_demo)/launch/master.launch">' >> $FILE
	echo '		<arg name="sim" value="true"/>' >> $FILE
	echo '	</include>' >> $FILE
	for i in `seq $1 $2`
	do
		x_m=$(($RANDOM % 2))
		x_cm=$(($RANDOM % 100))		
		if [ $RANDOM -lt 18634 ]
		then
			x=-$x_m.$x_cm
		else
			x=$x_m.$x_cm
		fi
		y_m=$(($RANDOM % 4))
		y_cm=$(($RANDOM % 100))
		y=-$y_m.$y_cm
		echo "	<group ns=\"robot$i\">" >> $FILE
		echo '		<include file="$(find wta_demo)/launch/kobuki.launch.xml">' >> $FILE
		echo "			<arg name=\"name\" value=\"robot$i\" />" >> $FILE
		echo "			<arg name=\"x\" value=\"$x\" />" >> $FILE
		echo "			<arg name=\"y\" value=\"$y\" />" >> $FILE
  		echo '		</include>' >> $FILE
		echo '	</group>' >> $FILE
	done
	echo "</launch>" >> $FILE
else
	echo "Second parameter is smaller than the first"
fi

#roslaunch wta_demo generatedMaster.launch &
