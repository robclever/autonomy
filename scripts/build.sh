#!/usr/bin/env bash
set -euo pipefail

# Build script for the autonomy project
# Usage: ./scripts/build.sh [build_type]
#   build_type: Debug (default) or Release

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_TYPE="${1:-Debug}"
BUILD_DIR="${PROJECT_DIR}/build/${BUILD_TYPE}"

echo "=== Building autonomy (${BUILD_TYPE}) ==="
echo "Build directory: ${BUILD_DIR}"

# Create build directory
mkdir -p "${BUILD_DIR}"

# Configure
cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DBUILD_TESTING=ON

# Build
cmake --build "${BUILD_DIR}" --parallel "$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)"

echo ""
echo "=== Build complete ==="
echo "Binaries are in: ${BUILD_DIR}"

# Run tests if ctest is available
if command -v ctest &> /dev/null; then
    echo ""
    echo "=== Running tests ==="
    ctest --test-dir "${BUILD_DIR}" --output-on-failure
fi
