#!/usr/bin/env bash
set -euo pipefail

# Source-based coverage using clang's profile/coverage mapping.
# Produces a per-file summary and an HTML report in build/coverage/.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_DIR}/build/coverage"

cd "$PROJECT_DIR"

# Clean build with coverage instrumentation
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

cmake -S "$PROJECT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DBUILD_TESTING=ON \
    -DCMAKE_CXX_FLAGS="-fprofile-instr-generate -fcoverage-mapping" \
    -DCMAKE_EXE_LINKER_FLAGS="-fprofile-instr-generate"

cmake --build "$BUILD_DIR" --parallel "$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)"

# Run tests to generate raw profiles (one per test via PID pattern)
LLVM_PROFILE_FILE="$BUILD_DIR/%p.profraw" ctest --test-dir "$BUILD_DIR" --output-on-failure

# Locate LLVM coverage tools
PROFDATA=$(xcrun --find llvm-profdata)
COV=$(xcrun --find llvm-cov)

# Merge all raw profiles into indexed data
"$PROFDATA" merge -sparse "$BUILD_DIR"/*.profraw -o "$BUILD_DIR/default.profdata"

# Collect test executables
TEST_BINS=("$BUILD_DIR"/core/test/test_*)

echo ""
echo "=== Coverage summary ==="
for bin in "${TEST_BINS[@]}"; do
    "$COV" report "$bin" -instr-profile="$BUILD_DIR/default.profdata"
done

# Generate HTML report (one show per binary, merged into one dir)
mkdir -p "$BUILD_DIR/html"
for bin in "${TEST_BINS[@]}"; do
    "$COV" show "$bin" \
        -instr-profile="$BUILD_DIR/default.profdata" \
        -format=html \
        -output-dir="$BUILD_DIR/html/$(basename "$bin")"
done

echo ""
echo "=== HTML reports ==="
for bin in "${TEST_BINS[@]}"; do
    echo "  $BUILD_DIR/html/$(basename "$bin")/index.html"
done
