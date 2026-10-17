# Builds the C++ test harness that runs HPR's original Event_Logic.ino on a laptop.
#   make            build ./harness
#   make run        convert the two-stage OpenRocket export and replay it

HARNESS_CXX := clang++
# -w: declarations.h and Event_Logic.ino are compiled unchanged and produce warnings we don't want to fix here
HARNESS_FLAGS := -std=c++17 -O2 -w

SRC := harness.cpp
DEPS := arduino_stubs.h ../../Main\ Code/declarations.h ../../Main\ Code/Event_Logic.ino

harness: $(SRC) $(DEPS)
	$(HARNESS_CXX) $(HARNESS_FLAGS) -o $@ $(SRC)

run: harness
	python3 ../../Parser/openrocket_to_test.py ../2_Stage_High_Powered_Rocket_Test.csv
	./harness ../2_Stage_High_Powered_Rocket_Test_sensors.csv --profile 2

clean:
	rm -f harness

.PHONY: run clean
