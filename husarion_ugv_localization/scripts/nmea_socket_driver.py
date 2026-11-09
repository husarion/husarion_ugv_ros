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

# UDP NMEA receiver that strips text before the '$'; parsing is done by nmea_navsat_driver's
# Ros2NMEADriver.

import socket
import sys
import threading
import time


def split_sentences(data):
    """Split one datagram into NMEA sentences, each cut down to start at '$'.

    Returns a list of (prefix, sentence) pairs. prefix is whatever came before
    the first '$' on that line. Lines without a '$' are dropped, and so are
    sentences that are not plain ASCII.
    """
    sentences = []
    for line in data.splitlines():
        start = line.find(b"$")
        if start < 0:
            continue
        # Junk in front of the '$' may be anything. A sentence must decode
        # strictly: dropping a bad byte from it is not safe, because the XOR
        # checksum still matches when two equal characters disappear.
        try:
            sentence = line[start:].decode("ascii").strip()
        except UnicodeDecodeError:
            continue
        sentences.append((line[:start].decode("ascii", errors="ignore"), sentence))
    return sentences


class PrefixReporter:
    """Warns once for each distinct prefix.

    The cause stays in the log without a line per sentence.
    """

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
            f"NMEA sentences arrive with {prefix[:32]!r} in front of the '$', ignoring it. "
            "Further sentences with the same prefix are not logged."
        )


def receive(sock, buffer_size, add_sentence, reporter, logger, ok):
    """Receive datagrams until ok() turns false. Returns False if receiving failed."""
    while ok():
        try:
            data, _ = sock.recvfrom(buffer_size)
        except socket.timeout:
            continue
        except OSError as exc:
            logger.error(f"Receiving NMEA data failed: {exc}")
            return False

        for prefix, sentence in split_sentences(data):
            reporter.report(prefix)
            try:
                add_sentence(sentence)
            except ValueError as exc:
                logger.warning(f"Dropped an NMEA sentence the driver could not parse: {exc}")
    return True


def open_socket(ip, port, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((ip, port))
        # The timeout only lets the loop notice a shutdown while no data comes in.
        sock.settimeout(timeout)
    except OSError:
        sock.close()
        raise
    return sock


def main(args=None):
    # Imported here so the tests can load this file without a ROS environment.
    import rclpy
    from libnmea_navsat_driver.driver import Ros2NMEADriver
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

    rclpy.init(args=args)
    driver = Ros2NMEADriver()

    local_ip = driver.declare_parameter("ip", "0.0.0.0").value
    local_port = driver.declare_parameter("port", 10110).value
    buffer_size = driver.declare_parameter("buffer_size", 4096).value
    timeout = driver.declare_parameter("timeout_sec", 2).value
    frame_id = driver.get_frame_id()

    # The receive loop blocks this thread, so parameter services and the rest of the node's
    # interfaces are served by an executor on a second one.
    executor = SingleThreadedExecutor()
    executor.add_node(driver)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    logger = driver.get_logger()
    logger.info(
        f"Prefix-tolerant NMEA receiver on {local_ip}:{local_port} "
        f"(buffer_size {buffer_size}, timeout_sec {timeout})"
    )

    reporter = PrefixReporter(logger.warning)
    sock = None
    rc = 0
    try:
        while rclpy.ok():
            try:
                sock = open_socket(local_ip, local_port, timeout)
            except OSError as exc:
                logger.error(f"Cannot set up the UDP socket: {exc}")
                rc = 1
                break
            received = receive(
                sock,
                buffer_size,
                lambda sentence: driver.add_sentence(sentence, frame_id),
                reporter,
                logger,
                rclpy.ok,
            )
            sock.close()
            sock = None
            if received:
                break
            # Open the socket again like the upstream node does, but not in a busy loop.
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if sock is not None:
            sock.close()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        driver.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
