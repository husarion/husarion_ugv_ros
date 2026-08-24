#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
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

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import PythonExpression


def normalize_log_level(log_level: SomeSubstitutionsType):
    # The launch arguments accept WARNING but rcl only parses WARN - a node
    # handed "--log-level WARNING" dies on argument parsing before it spins.
    return PythonExpression(
        ["'WARN' if '", log_level, "'.upper() == 'WARNING' else '", log_level, "'"]
    )


def limit_log_level_to_info(unit: SomeSubstitutionsType, log_level: SomeSubstitutionsType):
    # Keeps a chatty logger unit at INFO when the operator asks for DEBUG.
    # Evaluation has to happen inside one substitution: a bare `if
    # PythonExpression(...)` in Python is always truthy, which is how the old
    # version pinned every unit to INFO no matter the requested level.
    return PythonExpression(
        [
            "'",
            unit,
            ":=' + ('INFO' if '",
            log_level,
            "'.upper() == 'DEBUG' else ('WARN' if '",
            log_level,
            "'.upper() == 'WARNING' else '",
            log_level,
            "'))",
        ]
    )
