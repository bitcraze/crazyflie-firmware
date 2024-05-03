#!/usr/bin/env bash
OPTIONS='--recursive --indent=spaces=2  --style=kr --indent-switches --indent-preproc-define --pad-oper  --pad-header --unpad-paren --align-pointer=name --break-one-line-headers  --add-braces --max-code-length=100 --break-after-logical --suffix=none'
astyle src/modules/*.c $OPTIONS >>tmp.txt
astyle src/deck/*.c $OPTIONS >>tmp.txt
astyle src/platform/*.c $OPTIONS >>tmp.txt
astyle test/*.c  $OPTIONS >>tmp.txt
if grep -q "Formatted" "tmp.txt"; then 
    echo "Formatted files" 
else
    echo No files where changed
fi

rm tmp.txt
