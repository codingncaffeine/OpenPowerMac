#!/bin/bash
# Query a g4run event stream (--events FILE).
#
# Most analysis in the ATA dig was ad-hoc awk over a hundred thousand lines
# of prose. "Every manager completion between the driver call and its answer"
# took three attempts to express and one of them was silently mis-scoped,
# which is the worst possible failure for an analysis tool. The event stream
# is one JSON object per line, so the same question is a filter.
#
#   tools/events.sh <file> kinds                    what happened, and how often
#   tools/events.sh <file> between LO HI [kind]     events in an instruction window
#   tools/events.sh <file> kind KIND [n]            the last n events of one kind
#   tools/events.sh <file> field KIND NAME          tally one field across a kind
#   tools/events.sh <file> around AT SPAN           everything near an instant
#   tools/events.sh <file> fails                    non-zero ATA results, with fn
#
# Fields are flat and numeric, so `grep`/`awk` still work on the raw file if a
# question does not fit these verbs.
set -e
F=${1:?usage: events.sh <events.jsonl> <verb> [args]}
V=${2:?usage: events.sh <events.jsonl> <verb> [args]}
shift 2

num() { sed -n "s/.*\"$1\":\([-0-9]*\).*/\1/p"; }

case "$V" in
kinds)
  sed -n 's/.*"kind":"\([a-z_]*\)".*/\1/p' "$F" | sort | uniq -c | sort -rn
  ;;
between)
  LO=${1:?between LO HI [kind]}; HI=${2:?between LO HI [kind]}; K=${3:-}
  awk -v lo="$LO" -v hi="$HI" -v k="$K" '
    { at = $0; sub(/^\{"at":/, "", at); sub(/,.*/, "", at) }
    at+0 >= lo+0 && at+0 <= hi+0 && (k == "" || index($0, "\"kind\":\"" k "\"")) ' "$F"
  ;;
kind)
  K=${1:?kind KIND [n]}; N=${2:-40}
  grep "\"kind\":\"$K\"" "$F" | tail -n "$N"
  ;;
field)
  K=${1:?field KIND NAME}; N=${2:?field KIND NAME}
  grep "\"kind\":\"$K\"" "$F" | num "$N" | sort -n | uniq -c | sort -rn
  ;;
around)
  AT=${1:?around AT SPAN}; SP=${2:-100000}
  awk -v lo=$((AT - SP)) -v hi=$((AT + SP)) '
    { at = $0; sub(/^\{"at":/, "", at); sub(/,.*/, "", at) }
    at+0 >= lo && at+0 <= hi ' "$F"
  ;;
fails)
  # The ATA Manager reports success as zero, so a non-zero completion is the
  # whole signal. Only a handful occur in a three-billion instruction boot.
  grep '"kind":"ata_result"' "$F" | grep -v '"result":0[,}]'
  ;;
*)
  echo "unknown verb: $V" >&2
  exit 2
  ;;
esac
