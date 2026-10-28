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

import importlib.util
import pathlib

import pytest

SCRIPT = pathlib.Path(__file__).parent.parent / "scripts" / "nmea_socket_driver.py"
GGA = "$GPGGA,122849.00,5002.588836,N,02000.562674,E,1,09,0.6,192.3,M,42.0,M,,*61"
RMC = "$GPRMC,122849.00,A,5002.588836,N,02000.562674,E,0.0,0.0,221026,,,A*56"


@pytest.fixture(scope="module")
def driver():
    spec = importlib.util.spec_from_file_location("nmea_socket_driver", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def checksum_ok(sentence):
    # The same check nmea_navsat_driver applies, which drops the first character.
    body, checksum = sentence.split("*")
    value = 0
    for c in body[1:]:
        value ^= ord(c)
    return "%02X" % value == checksum.strip().upper()


def test_router_null_prefix_is_stripped(driver):
    data = ("(null)" + GGA + "\r\n").encode("ascii")
    assert driver.split_sentences(data) == [("(null)", GGA)]
    assert checksum_ok(GGA)
    assert not checksum_ok("(null)" + GGA)


def test_clean_sentence_passes_unchanged(driver):
    assert driver.split_sentences((GGA + "\r\n").encode("ascii")) == [("", GGA)]


def test_each_line_of_a_datagram_is_stripped(driver):
    data = ("(null)" + GGA + "\r\n" + "junk " + RMC + "\n").encode("ascii")
    assert driver.split_sentences(data) == [("(null)", GGA), ("junk ", RMC)]
    assert checksum_ok(RMC)


def test_lines_without_dollar_are_dropped(driver):
    data = ("(null)\r\n\r\n" + GGA + "\r\n").encode("ascii")
    assert driver.split_sentences(data) == [("", GGA)]


def test_non_ascii_junk_does_not_raise(driver):
    data = b"\xff\xfe" + GGA.encode("ascii") + b"\r\n"
    assert driver.split_sentences(data) == [("", GGA)]


def test_prefix_is_reported_once(driver):
    warnings = []
    reporter = driver.PrefixReporter(warnings.append)
    for _ in range(3):
        reporter.report("(null)")
    reporter.report("")
    reporter.report("other")
    assert len(warnings) == 2
    assert "'(null)'" in warnings[0]
    assert "'other'" in warnings[1]


def test_changing_prefixes_stop_being_logged(driver):
    warnings = []
    reporter = driver.PrefixReporter(warnings.append)
    for i in range(100):
        reporter.report(f"{i} ")
    assert len(warnings) == driver.PrefixReporter.MAX_PREFIXES + 1
    assert "no longer logging" in warnings[-1]
