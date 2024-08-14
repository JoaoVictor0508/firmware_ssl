#!/usr/bin/sh
echo "Formating code with file .clang-format:\n=============================================="
cat ../.clang-format
echo "=============================================="
find . -regex '.*\.\(cpp\|h\|cc\|c\)' -exec clang-format -style=file -i {} \; 
echo "Formatting finished!"
