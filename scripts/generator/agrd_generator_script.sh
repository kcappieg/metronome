#!/usr/bin/env bash

# Requires pre-configuration in filter_domains.py
# Specifically, the main function must invoke the filtering process with the correct configurations,
# all defined directly in python

# generate the domains

#################################
# UNIFORM
#################################

UNIFORM_COUNT=1000
UNIFORM_OBSTACLE_PROBABILITY=0.2
UNIFORM_INTERVENTION_PROBABILITY=0.3

for size in {7..10}
do
  for goals in {2..4}
  do
    SEED_SKIP=$((goals-2))
    ./generate_grid_world.py $size $size $UNIFORM_COUNT --filter --path ./gridworld/${goals}goal \
      --goals $goals --strategy single --observer \
      --obstacle-probability $UNIFORM_OBSTACLE_PROBABILITY \
      --intervention-probability $UNIFORM_INTERVENTION_PROBABILITY \
      --seed-skip $SEED_SKIP
  done
done

# configured for AGRD filtering
./filter_domains.py uniform


#################################
# ROOMS
#################################

MAP_FILES="64room_tiny_000.map"
for idx in {1..9}
do
  MAP_FILES="$MAP_FILES 64room_tiny_00${idx}.map"
done

ROOMS_COUNT=1000
ROOMS_INTERVENTION_PROBABILITY=0.3

for goals in {2..4}
do
  SEED_SKIP=$((goals-2))
  ./generate_gridmap_scenes.py $MAP_FILES --filter --path ./gridmap/${goals}goal \
    --resource-dir ../../resources/input/vacuum/gridmap/ \
    --goals $goals --num-scenes $ROOMS_COUNT \
    --intervention-probability $ROOMS_INTERVENTION_PROBABILITY \
    --seed-skip $SEED_SKIP
done

# configured for AGRD filtering
echo "Filtering for AGRD activity"
./filter_domains.py rooms

#################################
# LOGISTICS
#################################

LOGISTICS_COUNT=1000
LOGISTICS_CONNECTION_DIST=0.4
LOGISTICS_PACKAGES=3
LOGISTICS_TRUCKS=1
LOGISTICS_FLUENTS=1
LOGISTICS_MAX_COST=1

for locs in {7..11}
do
  for goals in {2..4}
  do
    SEED_SKIP=$((goals-2))
    ./generate_logistics.py --total $LOGISTICS_COUNT --path ./logistics/${goals}goal \
      --goals $goals --locations $locs \
      --packages $LOGISTICS_PACKAGES --trucks $LOGISTICS_TRUCKS \
      --topology geometric --connection-distance $LOGISTICS_CONNECTION_DIST \
      --ignore-connection-errors \
      --goal-fluents $LOGISTICS_FLUENTS --max-cost $LOGISTICS_MAX_COST \
      --seed-skip $SEED_SKIP
  done
done

# configured for AGRD filtering
./filter_domains.py logistics

exit 0