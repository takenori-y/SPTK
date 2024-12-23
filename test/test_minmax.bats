#!/usr/bin/env bats
# ------------------------------------------------------------------------ #
# Copyright 2021 SPTK Working Group                                        #
#                                                                          #
# Licensed under the Apache License, Version 2.0 (the "License");          #
# you may not use this file except in compliance with the License.         #
# You may obtain a copy of the License at                                  #
#                                                                          #
#     http://www.apache.org/licenses/LICENSE-2.0                           #
#                                                                          #
# Unless required by applicable law or agreed to in writing, software      #
# distributed under the License is distributed on an "AS IS" BASIS,        #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. #
# See the License for the specific language governing permissions and      #
# limitations under the License.                                           #
# ------------------------------------------------------------------------ #

sptk3=tools/sptk/bin
sptk4=bin
tmp=test_minmax

setup() {
    mkdir -p $tmp
}

teardown() {
    rm -rf $tmp
}

@test "minmax: compatibility" {
    $sptk3/nrand -l 20 | $sptk3/minmax -l 1 -o 0 > $tmp/1
    $sptk3/nrand -l 20 | $sptk4/minmax -l 1 -o 0 -w 1 > $tmp/2
    run $sptk4/aeq $tmp/1 $tmp/2
    [ "$status" -eq 0 ]

    $sptk3/nrand -l 20 | $sptk3/minmax -l 10 -b 2 -o 0 > $tmp/1
    $sptk3/nrand -l 20 | $sptk4/minmax -l 10 -b 2 -o 0 -w 0 > $tmp/2
    run $sptk4/aeq $tmp/1 $tmp/2
    [ "$status" -eq 0 ]

    $sptk3/nrand -l 20 | $sptk3/sopr -FIX | $sptk3/minmax -b 2 > $tmp/1
    $sptk3/nrand -l 20 | $sptk3/sopr -FIX | $sptk4/minmax -b 2 > $tmp/2
    run $sptk4/aeq $tmp/1 $tmp/2
    [ "$status" -eq 0 ]

    $sptk3/nrand -l 20 | $sptk3/minmax -b 2 -d | cut -d: -f2 | $sptk3/x2x +ai > $tmp/1
    $sptk3/nrand -l 20 | $sptk4/minmax -b 2 -p $tmp/2 > /dev/null
    run $sptk4/aeq $tmp/1 $tmp/2
    [ "$status" -eq 0 ]
}

@test "minmax: valgrind" {
    $sptk3/nrand -l 20 > $tmp/1
    run valgrind $sptk4/minmax -l 1 -o 0 -w 0 $tmp/1
    [ "$(echo "${lines[-1]}" | sed -r 's/.*SUMMARY: ([0-9]*) .*/\1/')" -eq 0 ]
    run valgrind $sptk4/minmax -l 1 -o 0 -w 1 $tmp/1
    [ "$(echo "${lines[-1]}" | sed -r 's/.*SUMMARY: ([0-9]*) .*/\1/')" -eq 0 ]
}
