CC ?= cc
CFLAGS ?= -std=gnu11 -Wall -Wextra -O2 -Iinclude -Isrc
LDLIBS := -lm

BUILD := build
LIB_SRCS := src/simmy.c src/simmy_induction.c src/simmy_bldc.c src/simmy_json.c src/simmy_config.c
LIB_OBJS := $(patsubst src/%.c,$(BUILD)/%.o,$(LIB_SRCS))
LIB := $(BUILD)/libsimmy.a

ANALOG_SERVO_DIR := ../analog-servo
ANALOG_SERVO_RLS_DIR ?= ../analog-servo-rls

.PHONY: all lib test test-tools examples integration rls-integration rls-axis-command-v2 clean

all: lib test examples

lib: $(LIB)

$(BUILD)/%.o: src/%.c include/simmy.h include/simmy_bldc.h src/simmy_json.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(LIB): $(LIB_OBJS)
	ar rcs $@ $(LIB_OBJS)

test: $(LIB)
	$(CC) $(CFLAGS) tests/test_simmy.c $(LIB) $(LDLIBS) -o $(BUILD)/test_simmy
	$(BUILD)/test_simmy
	$(CC) $(CFLAGS) tests/test_stepper.c $(LIB) $(LDLIBS) -o $(BUILD)/test_stepper
	$(BUILD)/test_stepper
	$(CC) $(CFLAGS) tests/test_stepper_sweep.c $(LIB) $(LDLIBS) -o $(BUILD)/test_stepper_sweep
	$(BUILD)/test_stepper_sweep
	$(CC) $(CFLAGS) tests/test_stepper_compensation.c $(LIB) $(LDLIBS) -o $(BUILD)/test_stepper_compensation
	$(BUILD)/test_stepper_compensation
	$(CC) $(CFLAGS) tests/test_stepper_moves.c $(LIB) $(LDLIBS) -o $(BUILD)/test_stepper_moves
	$(BUILD)/test_stepper_moves
	$(CC) $(CFLAGS) tests/test_induction.c $(LIB) $(LDLIBS) -o $(BUILD)/test_induction
	$(BUILD)/test_induction
	$(CC) $(CFLAGS) tests/test_induction_foc.c $(LIB) $(LDLIBS) -o $(BUILD)/test_induction_foc
	$(BUILD)/test_induction_foc
	$(CC) $(CFLAGS) tests/test_induction_ifoc.c $(LIB) $(LDLIBS) -o $(BUILD)/test_induction_ifoc
	$(BUILD)/test_induction_ifoc
	$(CC) $(CFLAGS) tests/test_bldc.c $(LIB) $(LDLIBS) -o $(BUILD)/test_bldc
	$(BUILD)/test_bldc
	$(CC) $(CFLAGS) tests/test_bldc_moves.c $(LIB) $(LDLIBS) -o $(BUILD)/test_bldc_moves
	$(BUILD)/test_bldc_moves

# Optional Python stdlib tooling tests; not a dependency of the C-only suite.
test-tools:
	python3 -m unittest discover -s tests -p 'test_*runner.py'
	python3 -m unittest discover -s tests -p 'test_bldc_metrics.py'

examples: $(LIB)
	$(CC) $(CFLAGS) examples/mechanical_signatures.c $(LIB) $(LDLIBS) -o $(BUILD)/mechanical_signatures
	$(CC) $(CFLAGS) examples/dc_motor.c $(LIB) $(LDLIBS) -o $(BUILD)/dc_motor
	$(CC) $(CFLAGS) examples/basic_two_mass.c $(LIB) $(LDLIBS) -o $(BUILD)/basic_two_mass
	$(CC) $(CFLAGS) examples/three_mass_leadscrew.c $(LIB) $(LDLIBS) -o $(BUILD)/three_mass_leadscrew
	$(CC) $(CFLAGS) examples/bidirectional_move.c $(LIB) $(LDLIBS) -o $(BUILD)/bidirectional_move
	$(CC) $(CFLAGS) examples/stepper_velocity.c $(LIB) $(LDLIBS) -o $(BUILD)/stepper_velocity
	$(CC) $(CFLAGS) examples/stepper_compensation.c $(LIB) $(LDLIBS) -o $(BUILD)/stepper_compensation
	$(CC) $(CFLAGS) examples/induction_vfd.c $(LIB) $(LDLIBS) -o $(BUILD)/induction_vfd
	$(CC) $(CFLAGS) examples/induction_foc.c $(LIB) $(LDLIBS) -o $(BUILD)/induction_foc
	$(CC) $(CFLAGS) examples/induction_ifoc.c $(LIB) $(LDLIBS) -o $(BUILD)/induction_ifoc
	$(CC) $(CFLAGS) examples/bldc_faults.c $(LIB) $(LDLIBS) -o $(BUILD)/bldc_faults
	$(CC) $(CFLAGS) examples/bldc_equivalence.c $(LIB) $(LDLIBS) -o $(BUILD)/bldc_equivalence
	$(CC) $(CFLAGS) examples/bldc_circulation.c $(LIB) $(LDLIBS) -o $(BUILD)/bldc_circulation
	@echo "Built examples in $(BUILD)/ -- run each from the repo root"

# Optional: only builds when the analog-servo sibling repo is present.
# Links the REAL firmware/src/Control.c, read-only, by relative path --
# proves actual control-loop linkage, not just the plain examples' own
# demo controller. See examples/analog_servo_integration/.
integration: $(LIB)
	@if [ ! -f $(ANALOG_SERVO_DIR)/firmware/src/Control.c ]; then \
	  echo "skipping integration example: $(ANALOG_SERVO_DIR)/firmware/src/Control.c not found"; \
	  exit 0; \
	fi
	$(CC) $(CFLAGS) -I $(ANALOG_SERVO_DIR)/firmware/include \
	  examples/analog_servo_integration/main.c \
	  $(ANALOG_SERVO_DIR)/firmware/src/Control.c \
	  $(LIB) $(LDLIBS) -o $(BUILD)/analog_servo_integration
	$(BUILD)/analog_servo_integration

# Optional: only builds when the analog-servo-rls sibling (the
# autotune-rls branch's own worktree, see its AGENTS.md's Branch
# convention) is present. Links the REAL firmware/src/{Control,RLS,
# GainGlide}.c read-only -- RLS.c/GainGlide.c don't depend on Autotune.c
# (only its result-enum header, see RLS.h), so that file is deliberately
# not linked here. See examples/rls_integration/.
rls-integration: $(LIB)
	@if [ ! -f $(ANALOG_SERVO_RLS_DIR)/firmware/src/RLS.c ]; then \
	  echo "skipping rls-integration example: $(ANALOG_SERVO_RLS_DIR)/firmware/src/RLS.c not found"; \
	  exit 0; \
	fi
	$(CC) $(CFLAGS) -I $(ANALOG_SERVO_RLS_DIR)/firmware/include \
	  examples/rls_integration/main.c \
	  $(ANALOG_SERVO_RLS_DIR)/firmware/src/Control.c \
	  $(ANALOG_SERVO_RLS_DIR)/firmware/src/RLS.c \
	  $(ANALOG_SERVO_RLS_DIR)/firmware/src/GainGlide.c \
	  $(LIB) $(LDLIBS) -o $(BUILD)/rls_integration
	$(BUILD)/rls_integration

# Optional, same sibling/linking convention as rls-integration above, but
# against the real frozen empirical SM232AE/TQ10 axis model instead of a
# simmy chain -- see examples/rls_axis_command_v2/.
rls-axis-command-v2: $(LIB)
	@if [ ! -f $(ANALOG_SERVO_RLS_DIR)/firmware/src/RLS.c ]; then \
	  echo "skipping rls-axis-command-v2 example: $(ANALOG_SERVO_RLS_DIR)/firmware/src/RLS.c not found"; \
	else \
	  $(CC) $(CFLAGS) -I $(ANALOG_SERVO_RLS_DIR)/firmware/include \
	    examples/rls_axis_command_v2/main.c \
	    $(ANALOG_SERVO_RLS_DIR)/firmware/src/Control.c \
	    $(ANALOG_SERVO_RLS_DIR)/firmware/src/RLS.c \
	    $(ANALOG_SERVO_RLS_DIR)/firmware/src/GainGlide.c \
	    $(LIB) $(LDLIBS) -o $(BUILD)/rls_axis_command_v2 && \
	  $(BUILD)/rls_axis_command_v2; \
	fi

clean:
	rm -rf $(BUILD)
