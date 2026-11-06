#!/bin/bash
# gen.sh [jtcores dir]: regenerate ../6801.vh, ../6801_param.vh, ../6801.uc (MC6802 microcode) from 6802.yaml.
#
# jotego's jt680x .v files `include "6801_param.vh" / "6801.vh"` and are used unmodified: the 6802 control store is
# generated under those names by jotego's own tool (jtframe ucode, cfg jt6802.yaml: src 6802.yaml, output 6801).
# jtcores is never written: the module is copied to a temp folder with our two yaml files added, jtframe is built
# there (needs Go >= 1.22, online the first time), and the generated $readmemb path is made relative ("6801.uc",
# found next to the .vh by Quartus; the Verilator harness links it into its run folder).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$(dirname "$HERE")"
JT="$(cd "${1:-$HERE/../../../../../jtcores}" && pwd)"
TMP="$(mktemp -d)"
trap 'chmod -R u+w "$TMP"; rm -rf "$TMP"' EXIT     # Go leaves its module cache read-only

mkdir -p "$TMP/modules" "$TMP/bin" "$TMP/out"
cp -r "$JT/modules/jt680x" "$TMP/modules/"
ln -s "$JT/modules/jtframe" "$TMP/modules/jtframe"
cp "$HERE/6802.yaml" "$TMP/modules/jt680x/ucode/6802.yaml"
cp "$HERE/jt6802.yaml" "$TMP/modules/jt680x/cfg/jt6802.yaml"

( cd "$JT/modules/jtframe/src/jtframe" &&
  GOPATH="$TMP/go" GOCACHE="$TMP/gocache" GOMODCACHE="$TMP/go/pkg/mod" go build -o "$TMP/bin/jtframe" . )
JTBIN="$TMP/bin" CORES="$JT/cores" JTROOT="$JT" MODULES="$TMP/modules" JTFRAME="$JT/modules/jtframe" \
    "$TMP/bin/jtframe" ucode jt680x 6802 -o "$TMP/out/6801"

n=$(grep -c "\$readmemb(\"$TMP/out/6801.uc\"" "$TMP/out/6801.vh")
[ "$n" = 1 ] || { echo "gen.sh: expected one \$readmemb of the generated .uc in 6801.vh, found $n" >&2; exit 1; }
sed -i "s|\$readmemb(\"$TMP/out/6801.uc\"|\$readmemb(\"6801.uc\"|" "$TMP/out/6801.vh"
cp "$TMP/out/6801.vh" "$TMP/out/6801_param.vh" "$TMP/out/6801.uc" "$OUT/"
echo "gen.sh: wrote $OUT/6801.vh 6801_param.vh 6801.uc (jtcores: $JT)"
