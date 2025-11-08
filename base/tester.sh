#!/bin/bash
# Uso: ./tester A.cpp  -> buscará in/A.in, in/A1.in, etc.

SRC=$1
BIN="${SRC%.*}"

BASENAME=$(basename "$BIN")
mkdir -p out
g++ -std=c++17 -O2 -Wall -DLOCAL -fsanitize=address,undefined "$SRC" -o "$BIN" || exit 1
shopt -s nullglob
for IN in in/"${BASENAME}"*.in; do
    TEST_FILE=$(basename "$IN")
    TEST_NAME="${TEST_FILE%.in}"
    echo -e "\nTesting $IN..."
    /usr/bin/time -v "./$BIN" < "$IN" > "out/$TEST_NAME.out" 2>&1 || {
        time "./$BIN" < "$IN" > "out/$TEST_NAME.out"
    }
    echo "--- INPUT ($IN) ---"
    head -n 5 "$IN"
    echo -e "\n--- OUTPUT (out/$TEST_NAME.out) ---"
    head -n 10 "out/$TEST_NAME.out"
    echo "========================================"
done