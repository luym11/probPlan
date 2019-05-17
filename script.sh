#!/bin/sh

s=1
touch result
while [ $s -lt 101 ]
do
   #python onlineMaxProbAstar.py $s >> result
   python onlineMaxProbAstar.py 10
   echo $s
   s=`expr $s + 1`
done