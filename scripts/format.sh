#!/usr/bin/env bash
set -euo pipefail

# clang-format runner for the autonomy project
# Usage: ./scripts/format.sh [--check]
#   --check  Only report files that would be reformatted (no changes written)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

FILES=$(find core sim -type f \( -name '*.cpp' -o -name '*.h' \))

if [ -z "$FILES" ]; then
    echo "No source files found."
    exit 0
fi

if [[ "${1:-}" == "--check" ]]; then
    # Dry-run: list files that differ from the desired format.
    OUT_OF_FORMAT=0
    for f in $FILES; do
        if ! clang-format --dry-run --Werror "$f" 2>/dev/null; then
            echo "$f"
            OUT_OF_FORMAT=1
        fi
    done
    if [ $OUT_OF_FORMAT -eq 0 ]; then
        echo "All files are properly formatted."
    else
        echo ""
        echo "Run './scripts/format.sh' to fix."
    fi
    exit $OUT_OF_FORMAT
else
    echo "=== Formatting source files ==="
    clang-format -i --style=file $FILES
    echo "Done."
fi
