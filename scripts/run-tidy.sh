#!/usr/bin/env bash
# clang-tidy over every hand-written TU (vendored qcustomplot excluded), against
# the compile database in build/ (or the dir passed as $1). CI runs this same
# script, so a clean local run means a clean CI run. Headers are covered via
# HeaderFilterRegex in .clang-tidy; findings are errors (WarningsAsErrors: '*').
set -euo pipefail
cd "$(dirname "$0")/.."

build_dir="${1:-build}"
tidy="${CLANG_TIDY:-clang-tidy}"
runner="${RUN_CLANG_TIDY:-run-clang-tidy}"

if [[ ! -f "${build_dir}/compile_commands.json" ]]; then
    echo "error: ${build_dir}/compile_commands.json not found -- configure and build first" >&2
    exit 2
fi

git ls-files '*.cpp' \
    | grep -E '^(core|cli|gui|visualizer|tests)/' \
    | grep -v qcustomplot \
    | xargs "${runner}" -clang-tidy-binary "${tidy}" -quiet -p "${build_dir}"
