#!/bin/sh

s=1
touch result
while [ $s -lt 35 ]
do
   python onlineMaxProbAstar.py $s >> result
   #echo $s
   s=`expr $s + 1`
done
