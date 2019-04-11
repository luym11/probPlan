#!/bin/sh

s=1
touch result
while [ $s -lt 100 ]
do
   #python onlineMaxProbAstar.py $s >> result
   python onlineMaxProbAstar.py 1 
   #echo $s
   s=`expr $s + 1`
done
