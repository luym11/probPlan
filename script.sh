#!/bin/sh

s=1
touch result
while [ $s -lt 50 ]
do
   #python onlineMaxProbAstar.py $s >> result
   python onlineMaxProbAstar.py 5
   #echo $s
   s=`expr $s + 1`
done

