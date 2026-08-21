#!/usr/bin/env bash
# Runs every acceptance test. Each opens a real GUI window; they take ~1 min total.
set -u
cd "$(dirname "$0")"
fail=0
for t in tests/test_*.py; do
  printf '%-28s ' "$(basename "$t")"
  out=$(timeout 300 python3 "$t" 2>&1)
  status=$?
  line=$(printf '%s' "$out" | grep -oE 'OK:.*' | head -1)
  if [ -n "$line" ] && [ $status -eq 0 ]; then
    echo "$line"
  else
    fail=1
    echo "FAILED"
    printf '%s\n' "$out" | grep -E 'Error|assert' | head -5 | sed 's/^/    /'
  fi
done
exit $fail
