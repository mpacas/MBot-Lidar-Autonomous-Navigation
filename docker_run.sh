#!/bin/sh
docker run \
  --name rob102-navigation \
  --rm -d \
  -p 8000:8000 \
  -p 8080:8080 \
  -v /$PWD/autonomous_navigation/:/code/autonomous_navigation \
  -v /$PWD/build/:/code/build \
  rob102-navigation
