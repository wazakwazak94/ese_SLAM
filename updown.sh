#! /bin/bash


echo -n "Set the range of numbers(0~32767):" 
read range
echo "Start(0~$range)"
randnum=$(($RANDOM%$range))
input=-1
until [ $input -eq $randnum ] 
do
    echo "Choose the num:"
    read input
    if [ $input -gt $randnum ]
        then
            echo "Enter the smaller num"
    elif [ $input -lt $randnum ]
        then
            echo "Enter the bigger num"
    fi
done

echo "Answer: $randnum"

