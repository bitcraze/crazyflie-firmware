#!/bin/sh
cmd="python3 -m cfloader flash cf2.bin stm32-fw -w"
address=(
"radio://0/36/2M/E7E7E7E7E7"
"radio://0/29/2M/E7E7E7E7E7"
"radio://0/33/2M/E7E7E7E7E7"
)
if [ $# -lt 1 ]
then
  for item in ${address[@]}
  do 
  echo $item
  $cmd $item
  done
else
  for i in "$@";do
    echo "radio://0/"$i"/2M/E7E7E7E7E7"
    $cmd "radio://0/"$i"/2M/E7E7E7E7E7"
  done
fi

