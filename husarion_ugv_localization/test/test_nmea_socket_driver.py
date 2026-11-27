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
import socket

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


try:
    from libnmea_navsat_driver.checksum_utils import check_nmea_checksum as checksum_ok
except ImportError:
    checksum_ok = None

needs_upstream = pytest.mark.skipif(
    checksum_ok is None, reason="nmea_navsat_driver is not installed"
)


@needs_upstream
def test_upstream_driver_accepts_the_stripped_sentence(driver):
    import rclpy
    from libnmea_navsat_driver.driver import Ros2NMEADriver

    [(prefix, sentence)] = driver.split_sentences(("(null)" + GGA + "\r\n").encode("ascii"))
    rclpy.init()
    try:
        node = Ros2NMEADriver()
        try:
            assert not node.add_sentence(prefix + sentence, "gps")
            assert node.add_sentence(sentence, "gps")
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_router_null_prefix_is_stripped(driver):
    data = ("(null)" + GGA + "\r\n").encode("ascii")
    assert driver.split_sentences(data) == [("(null)", GGA)]


@needs_upstream
def test_upstream_checksum_rejects_prefix_and_accepts_stripped_sentence():
    assert checksum_ok(GGA)
    assert checksum_ok(RMC)
    assert not checksum_ok("(null)" + GGA)


def test_clean_sentence_passes_unchanged(driver):
    assert driver.split_sentences((GGA + "\r\n").encode("ascii")) == [("", GGA)]


def test_each_line_of_a_datagram_is_stripped(driver):
    data = ("(null)" + GGA + "\r\n" + "junk " + RMC + "\n").encode("ascii")
    assert driver.split_sentences(data) == [("(null)", GGA), ("junk ", RMC)]


def test_lines_without_dollar_are_dropped(driver):
    data = ("(null)\r\n\r\n" + GGA + "\r\n").encode("ascii")
    assert driver.split_sentences(data) == [("", GGA)]


def test_non_ascii_junk_does_not_raise(driver):
    data = b"\xff\xfe" + GGA.encode("ascii") + b"\r\n"
    assert driver.split_sentences(data) == [("", GGA)]


def test_non_ascii_byte_inside_a_sentence_drops_it(driver):
    # Two equal characters cancel out in the XOR checksum, so losing both
    # would leave a wrong latitude with a valid checksum.
    bad = GGA.replace("5002.588836", "5002.5\xb8\xb836").encode("latin-1")
    data = bad + b"\r\n" + RMC.encode("ascii") + b"\r\n"
    assert driver.split_sentences(data) == [("", RMC)]


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


class FakeLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def error(self, msg):
        self.errors.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)


@pytest.fixture
def udp_pair(driver):
    receiver = driver.open_socket("127.0.0.1", 0, 0.05)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    yield receiver, sender
    sender.close()
    receiver.close()


def stop_after(n):
    calls = []

    def ok():
        calls.append(None)
        return len(calls) <= n

    return ok


def test_receive_hands_stripped_sentence_to_driver(driver, udp_pair):
    receiver, sender = udp_pair
    sender.sendto(("(null)" + GGA + "\r\n").encode("ascii"), receiver.getsockname())
    sentences = []
    logger = FakeLogger()
    received = driver.receive(
        receiver,
        4096,
        sentences.append,
        driver.PrefixReporter(logger.warning),
        logger,
        stop_after(3),
    )
    assert received
    assert sentences == [GGA]
    assert len(logger.warnings) == 1


def test_value_error_does_not_stop_receive(driver, udp_pair):
    receiver, sender = udp_pair
    sender.sendto((GGA + "\r\n" + RMC + "\r\n").encode("ascii"), receiver.getsockname())
    sentences = []

    def add_sentence(sentence):
        sentences.append(sentence)
        if sentence == GGA:
            raise ValueError("missing fields")

    logger = FakeLogger()
    received = driver.receive(
        receiver, 4096, add_sentence, driver.PrefixReporter(logger.warning), logger, stop_after(3)
    )
    assert received
    assert sentences == [GGA, RMC]
    assert any("missing fields" in w for w in logger.warnings)


def test_receive_error_stops_receive(driver, udp_pair):
    # main() then opens the socket again after a pause, so a lasting error does not busy-loop.
    receiver, _ = udp_pair
    receiver.close()
    logger = FakeLogger()
    received = driver.receive(
        receiver,
        4096,
        lambda s: None,
        driver.PrefixReporter(logger.warning),
        logger,
        stop_after(100),
    )
    assert not received
    assert len(logger.errors) == 1


def test_bind_failure_raises(driver, udp_pair):
    receiver, _ = udp_pair
    with pytest.raises(OSError):
        driver.open_socket("127.0.0.1", receiver.getsockname()[1], 0.05)
