import network
from time import ticks_ms, ticks_diff


class WiFiManager:
    def __init__(
        self,
        ssid,
        password,
        retry_interval_ms=5000,
        connect_timeout_ms=15000
    ):
        self.ssid = ssid
        self.password = password
        self.retry_interval_ms = retry_interval_ms
        self.connect_timeout_ms = connect_timeout_ms

        try:
            station_interface = network.WLAN.IF_STA
        except AttributeError:
            station_interface = network.STA_IF

        self.wlan = network.WLAN(station_interface)
        self.wlan.active(True)

        self.last_attempt_time = None
        self.attempt_start_time = None
        self.was_connected = False

    def is_connected(self):
        return self.wlan.isconnected()

    def ip_address(self):
        if not self.is_connected():
            return None

        return self.wlan.ifconfig()[0]

    def _start_connection(self):
        print("Connecting to Wi-Fi:", self.ssid)

        try:
            self.wlan.disconnect()
        except Exception:
            pass

        self.wlan.active(True)
        self.wlan.connect(self.ssid, self.password)

        now = ticks_ms()
        self.last_attempt_time = now
        self.attempt_start_time = now

    def update(self):
        now = ticks_ms()
        connected = self.is_connected()

        if connected:
            if not self.was_connected:
                self.was_connected = True
                self.attempt_start_time = None

                print(
                    "Wi-Fi connected | IP:",
                    self.ip_address()
                )

                return "connected"

            return None

        if self.was_connected:
            self.was_connected = False
            print("Wi-Fi disconnected.")
            disconnect_event = "disconnected"
        else:
            disconnect_event = None

        attempt_timed_out = (
            self.attempt_start_time is not None
            and ticks_diff(
                now,
                self.attempt_start_time
            ) >= self.connect_timeout_ms
        )

        retry_due = (
            self.last_attempt_time is None
            or ticks_diff(
                now,
                self.last_attempt_time
            ) >= self.retry_interval_ms
        )

        if self.attempt_start_time is None or attempt_timed_out:
            if retry_due:
                self._start_connection()

        return disconnect_event
