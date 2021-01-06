#!/usr/bin/env bash

# Requires pre-configuration in filter_domains.py
# Specifically, the main function must invoke the filtering process with the correct configurations,
# all defined directly in python

# generate the domains

UNIFORM_COUNT=500
UNIFORM_OBSTACLE_PROBABILITY=0.2
UNIFORM_INTERVENTION_PROBABILITY=0.3

for size in {7..10}
do
  ./generate_grid_world.py $size $size $UNIFORM_COUNT --filter --path ./gridworld/2goal \
      --goals 2 --strategy single --observer \
      --obstacle-probability $UNIFORM_OBSTACLE_PROBABILITY \
      --intervention-probability $UNIFORM_INTERVENTION_PROBABILITY

  ./generate_grid_world.py $size $size $UNIFORM_COUNT --filter --path ./gridworld/3goal \
      --goals 3 --strategy single --observer \
      --obstacle-probability $UNIFORM_OBSTACLE_PROBABILITY \
      --intervention-probability $UNIFORM_INTERVENTION_PROBABILITY  \
      --seed-skip 1

  ./generate_grid_world.py $size $size $UNIFORM_COUNT --filter --path ./gridworld/4goal \
      --goals 4 --strategy single --observer \
      --obstacle-probability $UNIFORM_OBSTACLE_PROBABILITY \
      --intervention-probability $UNIFORM_INTERVENTION_PROBABILITY  \
      --seed-skip 2
done

# configured for AGRD filtering when invoked from command line
# see note at top of file about pre-configuring python code
./filter_domains.py

exit 0