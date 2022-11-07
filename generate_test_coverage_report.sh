#!/bin/bash
mkdir ./test_report
lcov --capture \
     --directory ./CMakeFiles/arwain_test.dir/quaternion/ \
     --directory ./CMakeFiles/arwain_test.dir/logger/ \
     --directory ./CMakeFiles/arwain_test.dir/input_parser/ \
     --directory ./CMakeFiles/arwain_test.dir/orientation/ \
     --directory ./CMakeFiles/arwain_test.dir/vector3/ \
     --directory ./CMakeFiles/arwain_test.dir/timers/ \
     --output-file=coverage.info
genhtml coverage.info --output-directory=test_report 

echo "Test reports placed in PROJECT_DIR/build/test_report/. Run an http server from that location to view in browser."
echo "This can be done by, for example:"
echo "    python3 -m http.server"
