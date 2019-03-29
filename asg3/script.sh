#!/bin/bash
make astar
for i in {1..100}
do
  echo $i
  time ./astar $i
done

