#!/bin/sh
# Breaks the codec on purpose and reports which gates notice.
#
# A green suite cannot tell you whether it would have gone red. This can.
#
# Runs in a worktree of HEAD, so it measures what is committed and never
# touches the tree you are sitting in.
set -e
cd "$(dirname "$0")"
python3 mutate.py
