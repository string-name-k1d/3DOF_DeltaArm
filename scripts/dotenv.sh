#!/usr/bin/env bash
# Shared .env loader for run_arm.sh and scripts/build.sh.
#
# Supported syntax is deliberately small so the same file also works with
# Docker Compose:
#   KEY=value
#   export KEY=value
#   # comment
#   blank lines
# Values may be single- or double-quoted. Inline comments are not supported.
#
# Shell environment variables always win: a key is only exported when it is
# not already set. This mirrors Docker Compose's precedence.

load_dotenv() {
  local file="$1"
  local line key value

  [ -f "$file" ] || return 0

  while IFS= read -r line || [ -n "$line" ]; do
    # Trim leading and trailing whitespace.
    line="${line#"${line%%[![:space:]]*}"}"
    line="${line%"${line##*[![:space:]]}"}"

    case "$line" in
      ''|'#'*) continue ;;
    esac

    line="${line#export }"
    key="${line%%=*}"
    value="${line#*=}"

    # Ignore lines without '='.
    [ "$key" = "$line" ] && continue
    [ -n "$key" ] || continue

    # Strip one layer of matching quotes.
    case "$value" in
      \"*\") value="${value#\"}"; value="${value%\"}" ;;
      \'*\') value="${value#\'}"; value="${value%\'}" ;;
    esac

    if [ -z "${!key+x}" ]; then
      export "$key=$value"
    fi
  done < "$file"
}

# Load the first existing candidate file and export its variables into the
# current shell. Sets DOTENV_FILE to the path that was loaded (empty when none)
# and returns 1 when no candidate exists. Must be called directly, not inside
# $(...) - a subshell would discard the exports.
load_dotenv_first() {
  local file
  DOTENV_FILE=""
  for file in "$@"; do
    if [ -f "$file" ]; then
      load_dotenv "$file"
      DOTENV_FILE="$file"
      return 0
    fi
  done
  return 1
}
