#!/bin/sh

s=1
touch result
while [ $s -lt 50 ]
do
   #python onlineMaxProbAstar.py $s >> result
   python onlineMaxProbAstar.py 1
   #echo $s
   s=`expr $s + 1`
done
echo 'NOW Step is 5'
s=1
while [ $s -lt 50 ]
do
   #python onlineMaxProbAstar.py $s >> result
   python onlineMaxProbAstar.py 5
   #echo $s
   s=`expr $s + 1`
done
