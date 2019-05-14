#!/bin/bash
set -v on
diff Attack.cpp ../NG-NB2/Attack.cpp -s -b
diff Map.cpp ../NG-NB2/Map.cpp -s -b
diff TaskScheduler.cpp ../NG-NB2/TaskScheduler.cpp -s -b
