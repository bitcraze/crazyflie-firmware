#!/usr/bin/env zsh
#
# Zsh tab-completion for `make` in crazyflie-firmware.
#
# Completes:
#   - every target listed by `make help` (config targets, clean, all, etc.)
#   - every configs/*_defconfig file, as `make <name>_defconfig`
#
# Only takes effect inside a checkout of this repo (detected via
# tools/kbuild/Makefile.kbuild); everywhere else, `make` completion falls
# back to zsh's normal behaviour.
#
# Usage: add this line to your ~/.zshrc:
#   source /path/to/crazyflie-firmware/tools/completion/make-completion.zsh

_cf_kbuild_root() {
	local dir=$PWD
	while [[ $dir != / ]]; do
		if [[ -f $dir/tools/kbuild/Makefile.kbuild ]]; then
			print -r -- "$dir"
			return 0
		fi
		dir=${dir:h}
	done
	return 1
}

_cf_make() {
	local root
	root=$(_cf_kbuild_root)

	if [[ -z $root ]]; then
		if (( $+functions[_make] )); then
			_make
		else
			_default
		fi
		return
	fi

	local -a help_targets defconfig_targets

	help_targets=(${(f)"$(
		command make -s -C "$root" help 2>/dev/null \
			| sed -n -E 's/^\*?[[:space:]]+([A-Za-z0-9_.-]+)[[:space:]]+-[[:space:]]+(.*)$/\1:\2/p'
	)"})

	defconfig_targets=(${(f)"$(
		for f in "$root"/configs/*_defconfig(N); do
			print -r -- "${f:t}:defconfig (${${f:t}%_defconfig})"
		done
	)"})

	_describe -t make-help-targets 'make target' help_targets
	_describe -t make-defconfig-targets 'defconfig target' defconfig_targets
}

compdef _cf_make make
