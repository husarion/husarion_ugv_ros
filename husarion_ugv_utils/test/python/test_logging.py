# Copyright 2026 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import pytest
from husarion_ugv_utils.logging import limit_log_level_to_info
from launch import LaunchContext


@pytest.mark.parametrize(
    "log_level, expected",
    [
        ("DEBUG", "rcl:=INFO"),
        ("INFO", "rcl:=INFO"),
        ("WARN", "rcl:=WARN"),
        ("ERROR", "rcl:=ERROR"),
        ("FATAL", "rcl:=FATAL"),
    ],
)
def test_limit_log_level_to_info(log_level, expected):
    assert limit_log_level_to_info("rcl", log_level).perform(LaunchContext()) == expected
