#!/usr/bin/env bash

until rosrun sawyer pick_place.py
do
  echo "\nError: Trying again. Make sure base_marker, dest_marker, and obj_marker are in view,\n"
done