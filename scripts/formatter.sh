#!/bin/bash
FILES=$(find include/ src/ nodes/ auto_code/ -name "*.h" -o -name "*.c" -o -name "*.cpp")
function print_usage()
{
    echo "Usage Instructions"
    echo -e "<mode>:
        dry-run: Dry-Run"
    exit 1
}

function format_dryrun {
    # Run clang-format on the files and check for errors
    format_error_file="format_cpp_error.log"
    cmd="clang-format --dry-run -Werror "$FILES" &> "$format_error_file
    eval $cmd
    statusCode=$?
    if [ $statusCode -eq 0 ]; then
        echo "No formatting errors found."
        rm $format_error_file
        return 0
    else
        echo "Formatting errors found. Please run clang-format to fix them."
        return 1
    fi
}

if [ $# -eq 0 ]; then
    print_usage
else
    case $1 in
        "dry-run") format_dryrun;;
    esac
fi