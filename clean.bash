#!/usr/bin/env bash
set -euo pipefail
case "${1:-all}" in
  build) rm -rf build/ ;;
  install) rm -rf install/ ;;
  log) rm -rf log/ ;;
  all)
    read -r -p "Remove build/, install/, and log/? [y/N] " answer
    [[ "$answer" =~ ^[Yy]$ ]] || exit 0
    rm -rf build/ install/ log/
    ;;
  *) echo "Usage: $0 [build|install|log|all]" >&2; exit 2 ;;
esac
