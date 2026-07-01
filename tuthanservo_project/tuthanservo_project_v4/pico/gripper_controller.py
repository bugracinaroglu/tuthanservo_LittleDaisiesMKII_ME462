from time import ticks_ms, ticks_diff


class GripperController:
    VALID_TARGETS = ("open", "close", "neutral", "stop")

    def __init__(
        self,
        left_servo,
        right_servo,
        left_fsr,
        right_fsr,
        left_mean_angle,
        right_mean_angle,
        open_offset,
        close_offset,
        initial_position="open",
        step_deg=1,
        move_interval_ms=20,
        close_settle_ms=250
    ):
        self.left_servo = left_servo
        self.right_servo = right_servo
        self.left_fsr = left_fsr
        self.right_fsr = right_fsr

        self.left_mean_angle = float(left_mean_angle)
        self.right_mean_angle = float(right_mean_angle)
        self.open_offset = float(open_offset)
        self.close_offset = float(close_offset)

        self.left_open_angle = self.left_mean_angle - self.open_offset
        self.right_open_angle = self.right_mean_angle + self.open_offset

        self.left_close_angle = self.left_mean_angle + self.close_offset
        self.right_close_angle = self.right_mean_angle - self.close_offset

        self.step_deg = float(step_deg)
        self.move_interval_ms = move_interval_ms
        self.close_settle_ms = close_settle_ms

        self.last_move_time = ticks_ms()
        self.close_settle_start = None

        self.state = "stopped"
        self.target = None
        self.command_id = None
        self.last_reason = "startup"

        self.left_contact = False
        self.right_contact = False

        self._events = []

        self._check_angles()
        self.go_to_initial_position(initial_position)

    def _check_angles(self):
        angles = {
            "left_open_angle": self.left_open_angle,
            "right_open_angle": self.right_open_angle,
            "left_close_angle": self.left_close_angle,
            "right_close_angle": self.right_close_angle,
            "left_mean_angle": self.left_mean_angle,
            "right_mean_angle": self.right_mean_angle
        }

        for name, angle in angles.items():
            if angle < 0 or angle > 180:
                raise ValueError(
                    name + " is outside 0-180 degrees: " + str(angle)
                )

    def _queue_event(self, event_type, details=None):
        event = {
            "event": event_type,
            "state": self.state,
            "target": self.target,
            "command_id": self.command_id
        }

        if details is not None:
            event["details"] = details

        self._events.append(event)

    def pop_events(self):
        events = self._events
        self._events = []
        return events

    def _set_state(self, state, reason=None):
        changed = state != self.state
        self.state = state

        if reason is not None:
            self.last_reason = reason

        if changed:
            print("Gripper state:", self.state)

            self._queue_event(
                "state_changed",
                {
                    "reason": self.last_reason,
                    "left_angle": round(self.left_servo.current_angle, 1),
                    "right_angle": round(self.right_servo.current_angle, 1)
                }
            )

    def go_to_initial_position(self, initial_position):
        position = initial_position.lower()

        if position == "open":
            self.left_servo.go_to_angle(self.left_open_angle)
            self.right_servo.go_to_angle(self.right_open_angle)
            self.target = "open"
            self._set_state("open", "initial_position")

        elif position == "neutral":
            self.left_servo.go_to_angle(self.left_mean_angle)
            self.right_servo.go_to_angle(self.right_mean_angle)
            self.target = "neutral"
            self._set_state("neutral", "initial_position")

        elif position == "close":
            self.left_servo.go_to_angle(self.left_close_angle)
            self.right_servo.go_to_angle(self.right_close_angle)
            self.target = "close"
            self._set_state("closed_unverified", "initial_position")

        else:
            raise ValueError(
                "initial_position must be open, neutral, or close"
            )

    def request_state(self, target, command_id=None):
        target = str(target).strip().lower()

        if target not in self.VALID_TARGETS:
            self._queue_event(
                "command_rejected",
                {"target": target, "reason": "invalid_target"}
            )
            return False

        self.command_id = command_id
        self.target = target
        self.close_settle_start = None

        if target == "open":
            self.left_contact = False
            self.right_contact = False
            self.left_fsr.reset_filter()
            self.right_fsr.reset_filter()
            self._set_state("opening", "command_received")

        elif target == "close":
            self.left_contact = False
            self.right_contact = False

            # Start each close operation with fresh FSR samples.
            # Otherwise the pressed result from the previous close can
            # stop the servos immediately.
            self.left_fsr.reset_filter()
            self.right_fsr.reset_filter()

            self._set_state("closing", "command_received")

        elif target == "neutral":
            self.left_contact = False
            self.right_contact = False
            self.left_fsr.reset_filter()
            self.right_fsr.reset_filter()
            self._set_state("moving_to_neutral", "command_received")

        elif target == "stop":
            self._set_state("stopped", "command_received")

        self._queue_event(
            "command_accepted",
            {"target": target}
        )

        return True

    def _move_servo_towards(self, servo, target_angle):
        difference = target_angle - servo.current_angle

        if abs(difference) <= self.step_deg:
            servo.go_to_angle(target_angle)
            return True

        if difference > 0:
            servo.move_by(self.step_deg)
        else:
            servo.move_by(-self.step_deg)

        return False

    def _print_contact(self, side, servo, fsr):
        fsr_status = fsr.get_status()

        print()
        print(side, "FSR contact detected")
        print("Stopped angle:", round(servo.current_angle, 1))
        print(
            "Raw:", fsr_status["raw"],
            "| Average raw:", fsr_status["average_raw"]
        )
        print(
            "Voltage:", fsr_status["voltage"],
            "V | Average voltage:",
            fsr_status["average_voltage"],
            "V | Threshold:",
            fsr_status["threshold_voltage"],
            "V"
        )

        self._queue_event(
            "fsr_contact",
            {
                "side": side.lower(),
                "stopped_angle": round(servo.current_angle, 1),
                "fsr": fsr_status
            }
        )

    def _update_contact_latches(self):
        if self.state != "closing":
            return

        if not self.left_contact and self.left_fsr.is_pressed():
            self.left_contact = True
            self._print_contact(
                "Left",
                self.left_servo,
                self.left_fsr
            )

        if not self.right_contact and self.right_fsr.is_pressed():
            self.right_contact = True
            self._print_contact(
                "Right",
                self.right_servo,
                self.right_fsr
            )

    def _finish_close(self):
        if self.left_contact and self.right_contact:
            self._set_state(
                "closed",
                "both_fsr_thresholds_reached"
            )

        elif self.left_contact or self.right_contact:
            self._set_state(
                "partial_contact",
                "only_one_fsr_threshold_reached"
            )

        else:
            self._set_state(
                "close_failed",
                "close_angle_limits_reached_without_fsr_contact"
            )

    def update(self):
        # FSR sampling is independent of servo timing.
        self.left_fsr.update()
        self.right_fsr.update()
        self._update_contact_latches()

        now = ticks_ms()

        if ticks_diff(now, self.last_move_time) < self.move_interval_ms:
            return

        self.last_move_time = now

        if self.state == "opening":
            left_done = self._move_servo_towards(
                self.left_servo,
                self.left_open_angle
            )
            right_done = self._move_servo_towards(
                self.right_servo,
                self.right_open_angle
            )

            if left_done and right_done:
                self._set_state("open", "open_angles_reached")

        elif self.state == "moving_to_neutral":
            left_done = self._move_servo_towards(
                self.left_servo,
                self.left_mean_angle
            )
            right_done = self._move_servo_towards(
                self.right_servo,
                self.right_mean_angle
            )

            if left_done and right_done:
                self._set_state(
                    "neutral",
                    "neutral_angles_reached"
                )

        elif self.state == "closing":
            if self.left_contact:
                left_done = True
            else:
                left_done = self._move_servo_towards(
                    self.left_servo,
                    self.left_close_angle
                )

            if self.right_contact:
                right_done = True
            else:
                right_done = self._move_servo_towards(
                    self.right_servo,
                    self.right_close_angle
                )

            # Contacts may become true while the close limits are reached.
            if self.left_contact and self.right_contact:
                self._finish_close()
                return

            if left_done and right_done:
                if self.close_settle_start is None:
                    self.close_settle_start = now

                elif ticks_diff(
                    now,
                    self.close_settle_start
                ) >= self.close_settle_ms:
                    self._finish_close()
            else:
                self.close_settle_start = None

    def get_status(self):
        return {
            "state": self.state,
            "target": self.target,
            "command_id": self.command_id,
            "reason": self.last_reason,

            "left_angle": round(
                self.left_servo.current_angle,
                1
            ),
            "right_angle": round(
                self.right_servo.current_angle,
                1
            ),

            "left_contact": self.left_contact,
            "right_contact": self.right_contact,

            "left_fsr": self.left_fsr.get_status(),
            "right_fsr": self.right_fsr.get_status(),

            "targets": {
                "left_open": self.left_open_angle,
                "right_open": self.right_open_angle,
                "left_close_limit": self.left_close_angle,
                "right_close_limit": self.right_close_angle,
                "left_neutral": self.left_mean_angle,
                "right_neutral": self.right_mean_angle
            }
        }

    def deinit(self):
        self.left_servo.deinit()
        self.right_servo.deinit()
