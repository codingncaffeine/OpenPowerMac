#!/bin/bash
# Which code regions executed in one run but not the other.
#
# diffrun.sh compares summary axes; this answers the sharper question a fix
# actually poses — did new territory get reached? g4run's coverage timeline
# records the first entry into each 1 KB region, so the set difference of
# those region addresses is exactly "code B ran that A never did", and vice
# versa. A fix that unlocks progress shows up here as new regions even when
# every summary number looks unchanged.
#
#   tools/covdiff.sh <before.log> <after.log>
set -e
A=${1:?usage: covdiff.sh <before.log> <after.log>}
B=${2:?usage: covdiff.sh <before.log> <after.log>}

regions() { # the addresses in the coverage timeline, one per line
    sed -n '/^-- coverage timeline/,/^-- uni-north/p' "$1" |
        grep -oE '^\s+@[0-9]+\s+[0-9a-f]{8}' |
        awk '{print $2}' | sort -u
}

TA=$(mktemp); TB=$(mktemp)
trap 'rm -f "$TA" "$TB"' EXIT
regions "$A" > "$TA"
regions "$B" > "$TB"

printf 'A = %s  (%s regions listed)\n' "$A" "$(wc -l < "$TA")"
printf 'B = %s  (%s regions listed)\n\n' "$B" "$(wc -l < "$TB")"

printf -- '-- only in B (new territory):\n'
comm -13 "$TA" "$TB" | sed 's/^/   /' | head -40
printf -- '-- only in A (lost territory):\n'
comm -23 "$TA" "$TB" | sed 's/^/   /' | head -40

printf -- '\n-- totals: A-only=%s  B-only=%s  shared=%s\n' \
    "$(comm -23 "$TA" "$TB" | wc -l)" \
    "$(comm -13 "$TA" "$TB" | wc -l)" \
    "$(comm -12 "$TA" "$TB" | wc -l)"
printf -- '(note: the timeline prints only the last 32 first-entries, so\n'
printf -- ' this compares the tail of each run, which is the part that\n'
printf -- ' matters when a boot is parked.)\n'
