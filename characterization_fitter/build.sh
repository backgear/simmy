#!/bin/sh
# Builds this tool's own private shared object, straight from the real
# src/simmy.c -- the same chain stepper the C library and every example
# use, not a reimplementation. Deliberately does not touch the repo root
# Makefile or build/libsimmy.a: this is additive, local to
# characterization_fitter/, and the core library's normal build is
# untouched by it.
set -eu
cd "$(dirname "$0")"
mkdir -p build
cc -std=gnu11 -Wall -Wextra -O2 -fPIC -shared \
  -I../include \
  ../src/simmy.c chain_fit_bridge.c \
  -lm -o build/chain_fit.so
