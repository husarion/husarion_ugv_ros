#!/usr/bin/env python3

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

# UDP NMEA receiver in place of nmea_navsat_driver's nmea_socket_driver. The
# upstream node passes each line to the checksum check as it arrived, and that
# check drops the first character assuming it is '$'. Some router firmware
# puts text in front of every sentence (a Teltonika RUTX50 sends "(null)$GPGGA,..."),
# so every sentence failed the checksum and gps/fix never appeared. Parsing and
# publishing stay with the upstream Ros2NMEADriver.

import socket
import sys


def split_sentences(data):
    """Split one datagram into NMEA sentences, each cut down to start at '$'.

    Returns a list of (prefix, sentence) pairs. prefix is whatever came before
    the first '$' on that line. Lines without a '$' are dropped.
    """
    # A junk byte that is not ASCII must not kill the receive loop. It is
    # dropped here, and if it sat inside a sentence the checksum rejects it.
    text = data.decode("ascii", errors="ignore")
    sentences = []
    for line in text.splitlines():
        start = line.find("$")
        if start < 0:
            continue
        sentences.append((line[:start], line[start:].strip()))
    return sentences


class PrefixReporter:
    """Warns once for each distinct prefix, so the cause stays in the log without a line per sentence."""

    # A prefix that changes with every sentence (a counter, a timestamp) would
    # otherwise grow the set and the log forever.
    MAX_PREFIXES = 8

    def __init__(self, warn):
        self._warn = warn
        self._seen = set()
        self._full = False

    def report(self, prefix):
        if not prefix.strip() or prefix in self._seen or self._full:
            return
        if len(self._seen) >= self.MAX_PREFIXES:
            self._full = True
            self._warn("NMEA sentences keep arriving with new prefixes, no longer logging them.")
            return
        self._seen.add(prefix)
        self._warn(
            f"NMEA sentences arrive with {prefix!r} in front of the '$', ignoring it. "
            "Further sentences with the same prefix are not logged."
        )


def main(args=None):
    import rclpy
    from libnmea_navsat_driver.driver import Ros2NMEADriver

    rclpy.init(args=args)
    driver = Ros2NMEADriver()

    local_ip = driver.declare_parameter("ip", "0.0.0.0").value
    local_port = driver.declare_parameter("port", 10110).value
    buffer_size = driver.declare_parameter("buffer_size", 4096).value
    timeout = driver.declare_parameter("timeout_sec", 2).value
    frame_id = driver.get_frame_id()

    driver.get_logger().info(
        f"Using parameters ip {local_ip} port {local_port} "
        f"buffer_size {buffer_size} timeout_sec {timeout}"
    )
    reporter = PrefixReporter(driver.get_logger().warn)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((local_ip, local_port))
        # The timeout only lets the loop notice a shutdown while no data comes in.
        sock.settimeout(timeout)
    except OSError as exc:
        driver.get_logger().error(f"Cannot set up the UDP socket: {exc}")
        sys.exit(1)

    try:
        while rclpy.ok():
            try:
                data, _ = sock.recvfrom(buffer_size)
            except socket.timeout:
                continue
            except OSError as exc:
                driver.get_logger().error(f"Receiving NMEA data failed: {exc}")
                continue

            for prefix, sentence in split_sentences(data):
                reporter.report(prefix)
                try:
                    driver.add_sentence(sentence, frame_id)
                except ValueError as exc:
                    driver.get_logger().warn(
                        f"Value error, likely due to missing fields in the NMEA message: {exc}"
                    )
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
