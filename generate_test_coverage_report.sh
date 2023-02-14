#!/bin/bash
mkdir ./test_report

# Build the lcov command by looping over all folders which contain at least one .gcno file.
cmd="lcov --capture"
for result in $(find . -type f -name '*f*' | sed -r 's|/[^/]+$||' |sort |uniq); do
     cmd="${cmd} --directory ${result}"
done
cmd="${cmd} --output-file=coverage.info"

# Execute the lcov command.
eval $cmd

# Coverage reports include system headers. This removes them.
lcov --remove coverage.info '/usr/*' -o filtered_coverage.info

# Gerneate html report pages.
genhtml filtered_coverage.info --output-directory=test_report 

echo "Test reports placed in PROJECT_DIR/build/test_report/. Run an http server from that location to view in browser."
echo "This can be done by, for example:"
echo "    python3 -m http.server"
