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

import re


def extract_version_tuple(version_string: str) -> tuple[int, int, int]:
    match = re.search(r"v(\d+)\.(\d+)\.(\d+)", version_string)
    return tuple(map(int, match.groups())) if match else (0, 0, 0)


def check_version_compatibility(version: str, min_required_version: str) -> bool:
    """
    Check if the version is compatible with the minimum required version.
    Assumes versions in format "v0.0.0".
    """
    return extract_version_tuple(version) >= extract_version_tuple(min_required_version)
