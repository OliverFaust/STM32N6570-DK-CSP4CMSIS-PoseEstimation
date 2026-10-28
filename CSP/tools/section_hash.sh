#!/bin/bash
# Print arm-none-eabi-size and a SHA-256 of every loadable section of an ELF.
# Usage: CSP/tools/section_hash.sh <file.elf>
set -euo pipefail

PREFIX=${PREFIX:-arm-none-eabi-}
elf=${1:?usage: $0 <file.elf>}
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

${PREFIX}size "$elf"
echo

printf '%-20s %-10s %10s  %s\n' section vma size sha256
${PREFIX}objdump -h "$elf" | awk '
  /^ *[0-9]+ / { name = $2; size = $3; vma = $4; next }
  name != "" { if ($0 ~ /LOAD/ && $0 ~ /CONTENTS/) print name, vma, size; name = "" }
' | while read -r name vma size; do
  ${PREFIX}objcopy -O binary --only-section="$name" "$elf" "$tmp/s.bin"
  printf '%-20s 0x%-8s %10d  %s\n' "$name" "$vma" "$((16#$size))" \
    "$(sha256sum "$tmp/s.bin" | cut -d' ' -f1)"
done
