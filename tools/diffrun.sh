#!/bin/bash
# Compare two g4run logs on the axes that actually indicate boot progress.
#
# "Did that change anything?" was asked a dozen times in one session and
# answered by eyeballing LBA sets, coverage counts and histograms. Doing it
# by hand is slow and it hides small wins: one fix moved the whole boot 830M
# instructions earlier and that was nearly missed. This extracts the summary
# axes from both logs and prints them side by side, so a change is either
# visible immediately or provably absent.
#
#   tools/diffrun.sh <before.log> <after.log>
set -e
A=${1:?usage: diffrun.sh <before.log> <after.log>}
B=${2:?usage: diffrun.sh <before.log> <after.log>}

field() { # field <log> <label> <command...>
    local log=$1 label=$2; shift 2
    printf '%s' "$("$@" < "$log" 2>/dev/null | tr '\n' ' ' | cut -c1-72)"
}

cd_lbas()  { sed -n '/^-- cd command log/,/^-- ata traffic/p' |
             grep -oE 'a8:[0-9a-f]+' | sort -u | tr '\n' ' '; }
cov_count(){ grep -oE '^-- coverage timeline \([0-9]+' | grep -oE '[0-9]+'; }
cov_last() { sed -n '/^-- coverage timeline/,/^-- uni-north/p' | tail -1; }
hist_top() { sed -n '/^-- 68k pc histogram/,+1p' | tail -1; }
crtc()     { grep -m1 '^-- ati crtc'; }
executed() { grep -m1 '^-- executed'; }
dbdma()    { grep -m1 '^-- ata dbdma events'; }
picraise() { sed -n '/^-- openpic raises per source/,+3p' | tail -2 |
             tr '\n' ' '; }
drvq()     { grep '^DRVQ' | tail -1; }
ataerr()   { grep '^ATARES' | grep -oE 'result=-?[0-9]+' | sort | uniq -c |
             tr '\n' ' '; }

row() { # row <label> <fn>
    printf '%-14s\n  A: %s\n  B: %s\n' "$1" \
        "$(field "$A" "$1" $2)" "$(field "$B" "$1" $2)"
}

printf 'A = %s\nB = %s\n\n' "$A" "$B"
row 'executed'   executed
row 'coverage n' cov_count
row 'coverage ->' cov_last
row 'CD LBAs'    cd_lbas
row '68K hottest' hist_top
row 'CRTC'       crtc
row 'DBDMA'      dbdma
row 'PIC raises' picraise
row 'DrvQHdr'    drvq
row 'ATA results' ataerr

printf '\n-- verdict: '
if [ "$(sed -n '/^-- cd command log/,/^-- ata traffic/p' "$A" |
        grep -oE 'a8:[0-9a-f]+' | sort -u)" = \
     "$(sed -n '/^-- cd command log/,/^-- ata traffic/p' "$B" |
        grep -oE 'a8:[0-9a-f]+' | sort -u)" ]; then
    printf 'CD access pattern IDENTICAL'
else
    printf 'CD access pattern CHANGED'
fi
printf '\n'
