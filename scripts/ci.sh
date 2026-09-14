# manual run of CI steps for local testing

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
clang-format --dry-run --Werror $(git ls-files '*.cpp' '*.h')
python3 scripts/check_deps.py