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


def limit_log_level_to_info(unit: SomeSubstitutionsType, log_level: SomeSubstitutionsType):
    log_level = PythonExpression(["'", log_level, "'.upper()'"])

    if PythonExpression(["'", log_level, "' == 'DEBUG'"]):
        return PythonExpression(["'", unit, "' + ':=' + 'INFO'"])
    else:
        return PythonExpression(["'", unit, "' + ':=' + ", log_level])
