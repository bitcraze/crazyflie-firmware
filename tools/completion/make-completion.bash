#!/usr/bin/env bash
#
# Bash tab-completion for `make` in crazyflie-firmware.
#
# Completes:
#   - every target listed by `make help` (config targets, clean, all, etc.)
#   - every configs/*_defconfig file, as `make <name>_defconfig`
#
# Only takes effect inside a checkout of this repo (detected via
# tools/kbuild/Makefile.kbuild); everywhere else, `make` completion falls
# back to bash's normal behaviour.
#
# Usage: add this line to your ~/.bashrc:
#   source /path/to/crazyflie-firmware/tools/completion/make-completion.bash

_cf_kbuild_root() {
    local dir="$PWD"
    while [[ "$dir" != "/" ]]; do
        if [[ -f "$dir/tools/kbuild/Makefile.kbuild" ]]; then
            printf '%s\n' "$dir"
            return 0
        fi
        dir=$(dirname "$dir")
    done
    return 1
}

_cf_make() {
    local root
    root=$(_cf_kbuild_root)

    if [[ -z "$root" ]]; then
        if declare -F _make >/dev/null; then
            _make
        elif declare -F _filedir >/dev/null; then
            _filedir
        fi
        return
    fi

    local cur targets
    cur="${COMP_WORDS[COMP_CWORD]}"

    targets=$(
        {
            command make -s -C "$root" help 2>/dev/null \
                | sed -n -E 's/^\*?[[:space:]]+([A-Za-z0-9_.-]+)[[:space:]]+-[[:space:]]+.*$/\1/p'
            for f in "$root"/configs/*_defconfig; do
                [[ -e "$f" ]] && basename "$f"
            done
        } | sort -u
    )

    COMPREPLY=($(compgen -W "$targets" -- "$cur"))
}

complete -F _cf_make -o default make
