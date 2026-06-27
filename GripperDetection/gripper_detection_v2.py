import cv2
import numpy as np
import time


class GripperVision:
    def __init__(self, camera_index=2):
        self.camera_index = camera_index

        self.frame_width = 1280
        self.frame_height = 720

        self.table_hsv_center = None
        self.empty_gripper_ignore_mask = None

        self.ready_counter = 0
        self.ready_frames_required = 6

        self.min_tip_area = 40

        self.roi_top_ratio = 0.30
        self.roi_bottom_ratio = 0.68
        self.close_line_ratio = 0.30

        self.min_object_area_ratio = 0.004
        self.close_object_area_ratio = 0.018

        # Object detection sensitivity
        self.object_lab_delta_threshold = 34
        self.object_min_fill_ratio = 0.10
        self.object_max_bbox_area_ratio = 0.55
        self.object_min_bbox_size = 12

        self.last_command_text = "IDLE"
        self.last_command_until = 0.0

    # =====================================================
    # Camera
    # =====================================================

    def open_camera(self):
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        cap.set(cv2.CAP_PROP_FPS, 30)

        if not cap.isOpened():
            raise RuntimeError("Camera could not be opened. Try camera_index=0, 1, or 2.")

        return cap

    # =====================================================
    # Basic Helpers
    # =====================================================

    def clean_mask(self, mask, kernel_size=5, iterations=1):
        kernel = np.ones((kernel_size, kernel_size), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)

        return mask

    def hsv_mask(self, hsv, lower, upper):
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)

        return cv2.inRange(hsv, lower, upper)

    def find_contours(self, mask):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        return contours

    def bbox_from_mask(self, mask, min_area=100):
        contours = self.find_contours(mask)

        valid_contours = []

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < min_area:
                continue

            valid_contours.append(cnt)

        if len(valid_contours) == 0:
            return None

        all_points = np.vstack(valid_contours)
        x, y, w, h = cv2.boundingRect(all_points)

        return x, y, w, h

    def clamp_bbox_xyxy(self, x1, y1, x2, y2, frame):
        h, w = frame.shape[:2]

        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w - 1))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h - 1))

        if x2 <= x1:
            x2 = min(w - 1, x1 + 1)

        if y2 <= y1:
            y2 = min(h - 1, y1 + 1)

        return x1, y1, x2, y2

    def draw_text(self, frame, text, pos, color=(255, 255, 255),
                  bg=(40, 40, 40), scale=0.75, thickness=2):

        x, y = pos
        font = cv2.FONT_HERSHEY_SIMPLEX

        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)

        cv2.rectangle(
            frame,
            (x - 5, y - th - 8),
            (x + tw + 5, y + baseline + 5),
            bg,
            -1
        )

        cv2.putText(
            frame,
            text,
            (x, y),
            font,
            scale,
            color,
            thickness,
            cv2.LINE_AA
        )

    # =====================================================
    # Color Masks
    # =====================================================

    def get_color_masks(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue gripper arms
        blue_mask = self.hsv_mask(
            hsv,
            [85, 45, 35],
            [135, 255, 255]
        )

        # Orange tape on tips
        orange_mask = self.hsv_mask(
            hsv,
            [5, 70, 60],
            [30, 255, 255]
        )

        # White gripper links
        white_mask = self.hsv_mask(
            hsv,
            [0, 0, 145],
            [179, 95, 255]
        )

        blue_mask = self.clean_mask(blue_mask, kernel_size=5)
        orange_mask = self.clean_mask(orange_mask, kernel_size=5)
        white_mask = self.clean_mask(white_mask, kernel_size=5)

        gripper_mask = cv2.bitwise_or(blue_mask, orange_mask)
        gripper_mask = cv2.bitwise_or(gripper_mask, white_mask)

        gripper_mask = self.clean_mask(gripper_mask, kernel_size=7)

        return {
            "blue": blue_mask,
            "orange": orange_mask,
            "white": white_mask,
            "gripper": gripper_mask
        }

    # =====================================================
    # Orange Tip Detection
    # =====================================================

    def detect_orange_tips(self, frame, orange_mask):
        contours = self.find_contours(orange_mask)

        tip_candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < self.min_tip_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2

            aspect = w / max(1, h)

            if aspect > 4.0 or aspect < 0.15:
                continue

            tip_candidates.append({
                "area": area,
                "bbox": (x, y, w, h),
                "center": (cx, cy)
            })

        if len(tip_candidates) < 2:
            return None

        tip_candidates = sorted(
            tip_candidates,
            key=lambda p: p["area"],
            reverse=True
        )

        tips = tip_candidates[:2]
        tips = sorted(tips, key=lambda p: p["center"][0])

        return {
            "left": tips[0],
            "right": tips[1]
        }

    # =====================================================
    # Gripper ROI and Arm Rectangles
    # =====================================================

    def make_grip_roi(self, frame, tips):
        h, w = frame.shape[:2]

        lx, ly = tips["left"]["center"]
        rx, ry = tips["right"]["center"]

        if lx > rx:
            lx, rx = rx, lx
            ly, ry = ry, ly

        gap_width = rx - lx

        if gap_width < 50:
            return None

        center_x = (lx + rx) // 2
        center_y = (ly + ry) // 2

        roi_x1 = int(lx + 0.08 * gap_width)
        roi_x2 = int(rx - 0.08 * gap_width)

        roi_y1 = int(center_y - self.roi_top_ratio * gap_width)
        roi_y2 = int(center_y + self.roi_bottom_ratio * gap_width)

        roi_x1, roi_y1, roi_x2, roi_y2 = self.clamp_bbox_xyxy(
            roi_x1,
            roi_y1,
            roi_x2,
            roi_y2,
            frame
        )

        close_line_y = int(center_y + self.close_line_ratio * gap_width)
        close_line_y = max(0, min(close_line_y, h - 1))

        return {
            "bbox": (roi_x1, roi_y1, roi_x2, roi_y2),
            "center": (center_x, center_y),
            "gap_width": gap_width,
            "close_line_y": close_line_y
        }

    def detect_gripper_arm_rectangles(self, frame, tips, gripper_mask):
        if tips is None:
            return None

        h, w = frame.shape[:2]

        lx, ly = tips["left"]["center"]
        rx, ry = tips["right"]["center"]

        if lx > rx:
            lx, rx = rx, lx
            ly, ry = ry, ly

        gap_width = abs(rx - lx)

        if gap_width < 50:
            return None

        center_x = (lx + rx) // 2
        center_y = (ly + ry) // 2

        y1 = int(center_y - 0.45 * gap_width)
        y2 = int(center_y + 0.55 * gap_width)

        x1_left = int(lx - 0.35 * gap_width)
        x2_left = int(center_x)

        x1_right = int(center_x)
        x2_right = int(rx + 0.35 * gap_width)

        x1_left, y1_left, x2_left, y2_left = self.clamp_bbox_xyxy(
            x1_left, y1, x2_left, y2, frame
        )

        x1_right, y1_right, x2_right, y2_right = self.clamp_bbox_xyxy(
            x1_right, y1, x2_right, y2, frame
        )

        left_roi_mask = np.zeros((h, w), dtype=np.uint8)
        right_roi_mask = np.zeros((h, w), dtype=np.uint8)

        left_roi_mask[y1_left:y2_left, x1_left:x2_left] = 255
        right_roi_mask[y1_right:y2_right, x1_right:x2_right] = 255

        left_arm_mask = cv2.bitwise_and(gripper_mask, left_roi_mask)
        right_arm_mask = cv2.bitwise_and(gripper_mask, right_roi_mask)

        left_bbox = self.bbox_from_mask(left_arm_mask, min_area=150)
        right_bbox = self.bbox_from_mask(right_arm_mask, min_area=150)

        return {
            "left_bbox": left_bbox,
            "right_bbox": right_bbox,
            "left_mask": left_arm_mask,
            "right_mask": right_arm_mask
        }

    # =====================================================
    # Background / Table Model
    # =====================================================

    def calibrate_table(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]

        patch_h = int(0.16 * h)
        patch_w = int(0.22 * w)

        patches = [
            hsv[0:patch_h, 0:patch_w],
            hsv[0:patch_h, w - patch_w:w],
            hsv[h - patch_h:h, 0:patch_w],
            hsv[h - patch_h:h, w - patch_w:w]
        ]

        samples = np.vstack([p.reshape(-1, 3) for p in patches])

        self.table_hsv_center = np.median(samples, axis=0).astype(np.uint8)

        print("Table HSV calibrated:", self.table_hsv_center)

    def get_table_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if self.table_hsv_center is None:
            lower = np.array([5, 15, 35], dtype=np.uint8)
            upper = np.array([45, 190, 245], dtype=np.uint8)
        else:
            h0, s0, v0 = self.table_hsv_center

            h_tol = 14
            s_tol = 65
            v_tol = 75

            lower = np.array([
                max(0, int(h0) - h_tol),
                max(0, int(s0) - s_tol),
                max(0, int(v0) - v_tol)
            ], dtype=np.uint8)

            upper = np.array([
                min(179, int(h0) + h_tol),
                min(255, int(s0) + s_tol),
                min(255, int(v0) + v_tol)
            ], dtype=np.uint8)

        table_mask = cv2.inRange(hsv, lower, upper)
        table_mask = self.clean_mask(table_mask, kernel_size=7)

        return table_mask

    def capture_empty_gripper_ignore_mask(self, frame, masks):
        table_mask = self.get_table_mask(frame)

        not_table = cv2.bitwise_not(table_mask)

        ignore_mask = cv2.bitwise_or(not_table, masks["gripper"])

        kernel = np.ones((13, 13), np.uint8)
        ignore_mask = cv2.dilate(ignore_mask, kernel, iterations=1)

        self.empty_gripper_ignore_mask = ignore_mask

        print("Empty gripper ignore mask captured.")

    # =====================================================
    # Object Detection - Improved
    # =====================================================

    def get_local_background_lab(self, frame, grip_roi, masks):
        """
        Estimate local background color around the grip ROI.
        This is better than using one global table color because lighting is not uniform.
        """
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = grip_roi["bbox"]

        roi_w = x2 - x1
        roi_h = y2 - y1

        margin_x = int(0.20 * roi_w)
        margin_y = int(0.20 * roi_h)

        bx1 = max(0, x1 - margin_x)
        bx2 = min(w - 1, x2 + margin_x)
        by1 = max(0, y1 - margin_y)
        by2 = min(h - 1, y2 + margin_y)

        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

        sample_mask = np.zeros((h, w), dtype=np.uint8)
        sample_mask[by1:by2, bx1:bx2] = 255

        # Do not sample the grip ROI itself.
        sample_mask[y1:y2, x1:x2] = 0

        # Remove gripper colors from background samples.
        gripper_dilated = cv2.dilate(
            masks["gripper"],
            np.ones((15, 15), np.uint8),
            iterations=1
        )
        sample_mask[gripper_dilated > 0] = 0

        ys, xs = np.where(sample_mask > 0)

        if len(xs) < 100:
            sample_mask = np.zeros((h, w), dtype=np.uint8)
            sample_mask[y1:y2, x1:x2] = 255
            sample_mask[gripper_dilated > 0] = 0

            ys, xs = np.where(sample_mask > 0)

        if len(xs) < 100:
            return None

        samples = lab[ys, xs]
        bg_lab = np.median(samples, axis=0).astype(np.float32)

        return bg_lab

    def find_object_components(self, object_mask, grip_roi):
        """
        Use connected components instead of contours.
        This avoids ugly contour shapes and helps reject sparse false positives.
        """
        x1, y1, x2, y2 = grip_roi["bbox"]
        roi_area = max(1, (x2 - x1) * (y2 - y1))
        gap_width = grip_roi["gap_width"]

        min_area = max(
            250,
            self.min_object_area_ratio * gap_width * gap_width
        )

        max_bbox_area = self.object_max_bbox_area_ratio * roi_area

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            object_mask,
            connectivity=8
        )

        candidates = []

        for label in range(1, num_labels):
            x = stats[label, cv2.CC_STAT_LEFT]
            y = stats[label, cv2.CC_STAT_TOP]
            bw = stats[label, cv2.CC_STAT_WIDTH]
            bh = stats[label, cv2.CC_STAT_HEIGHT]
            area = stats[label, cv2.CC_STAT_AREA]

            if area < min_area:
                continue

            if bw < self.object_min_bbox_size or bh < self.object_min_bbox_size:
                continue

            bbox_area = bw * bh

            if bbox_area <= 0:
                continue

            if bbox_area > max_bbox_area:
                continue

            fill_ratio = area / bbox_area

            if fill_ratio < self.object_min_fill_ratio:
                continue

            aspect = bw / max(1, bh)

            if aspect > 5.5 or aspect < 0.18:
                continue

            cx, cy = centroids[label]
            cx = int(cx)
            cy = int(cy)

            candidates.append({
                "bbox": (x, y, bw, bh),
                "center": (cx, cy),
                "area": area,
                "fill_ratio": fill_ratio,
                "bbox_area": bbox_area,
                "label": label
            })

        if len(candidates) == 0:
            return None

        roi_cx, roi_cy = grip_roi["center"]

        def score_candidate(c):
            cx, cy = c["center"]
            dist = abs(cx - roi_cx) + 0.6 * abs(cy - roi_cy)

            return c["area"] - 0.35 * dist

        best = max(candidates, key=score_candidate)

        return best

    def detect_object(self, frame, grip_roi, masks):
        if grip_roi is None:
            return None, np.zeros(frame.shape[:2], dtype=np.uint8)

        h, w = frame.shape[:2]

        roi_x1, roi_y1, roi_x2, roi_y2 = grip_roi["bbox"]
        gap_width = grip_roi["gap_width"]

        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)

        bg_lab = self.get_local_background_lab(frame, grip_roi, masks)

        if bg_lab is None:
            return None, np.zeros(frame.shape[:2], dtype=np.uint8)

        # Difference from local background color.
        diff = lab - bg_lab
        delta = np.sqrt(
            diff[:, :, 0] ** 2 +
            diff[:, :, 1] ** 2 +
            diff[:, :, 2] ** 2
        )

        object_mask = np.zeros((h, w), dtype=np.uint8)

        # Object assumption:
        # Object should be different from local table/background.
        object_mask[delta > self.object_lab_delta_threshold] = 255

        # Keep only grip ROI.
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        roi_mask[roi_y1:roi_y2, roi_x1:roi_x2] = 255
        object_mask = cv2.bitwise_and(object_mask, roi_mask)

        # Remove gripper colors.
        gripper_mask_dilated = cv2.dilate(
            masks["gripper"],
            np.ones((17, 17), np.uint8),
            iterations=1
        )
        object_mask[gripper_mask_dilated > 0] = 0

        # Remove static empty gripper structures if captured.
        if self.empty_gripper_ignore_mask is not None:
            object_mask[self.empty_gripper_ignore_mask > 0] = 0

        # Clean object mask.
        object_mask = cv2.medianBlur(object_mask, 5)

        object_mask = cv2.morphologyEx(
            object_mask,
            cv2.MORPH_OPEN,
            np.ones((5, 5), np.uint8),
            iterations=1
        )

        object_mask = cv2.morphologyEx(
            object_mask,
            cv2.MORPH_CLOSE,
            np.ones((9, 9), np.uint8),
            iterations=1
        )

        component = self.find_object_components(object_mask, grip_roi)

        if component is None:
            return None, object_mask

        x, y, bw, bh = component["bbox"]
        cx, cy = component["center"]
        area = component["area"]

        bottom_y = y + bh
        close_line_y = grip_roi["close_line_y"]

        close_area = max(
            900,
            self.close_object_area_ratio * gap_width * gap_width
        )

        roi_center_x, roi_center_y = grip_roi["center"]

        x_tol = max(20, int(0.13 * gap_width))
        y_tol = max(20, int(0.12 * gap_width))

        centered_x = abs(cx - roi_center_x) <= x_tol
        centered_y = abs(cy - roi_center_y) <= y_tol

        low_enough = bottom_y >= close_line_y
        large_enough = area >= close_area

        close_ready = centered_x and low_enough and large_enough

        object_data = {
            "bbox": (x, y, bw, bh),
            "center": (cx, cy),
            "area": area,
            "bottom_y": bottom_y,
            "close_ready": close_ready,
            "centered_x": centered_x,
            "centered_y": centered_y,
            "low_enough": low_enough,
            "large_enough": large_enough,
            "fill_ratio": component["fill_ratio"]
        }

        return object_data, object_mask

    # =====================================================
    # Decision
    # =====================================================

    def get_decision(self, grip_roi, object_data):
        if grip_roi is None:
            self.ready_counter = 0
            return "GRIPPER NOT FOUND", False

        if object_data is None:
            self.ready_counter = 0
            return "SEARCH OBJECT", False

        roi_cx, roi_cy = grip_roi["center"]
        gap_width = grip_roi["gap_width"]

        obj_cx, obj_cy = object_data["center"]

        x_tol = max(20, int(0.13 * gap_width))
        y_tol = max(20, int(0.12 * gap_width))

        messages = []

        if obj_cx < roi_cx - x_tol:
            messages.append("MOVE LEFT")
        elif obj_cx > roi_cx + x_tol:
            messages.append("MOVE RIGHT")

        if obj_cy < roi_cy - y_tol:
            messages.append("MOVE UP")
        elif obj_cy > roi_cy + y_tol:
            messages.append("MOVE DOWN")

        if len(messages) > 0:
            self.ready_counter = 0
            return " + ".join(messages), False

        if not object_data["low_enough"] or not object_data["large_enough"]:
            self.ready_counter = 0
            return "COME CLOSER", False

        if object_data["close_ready"]:
            self.ready_counter += 1
        else:
            self.ready_counter = 0

        if self.ready_counter >= self.ready_frames_required:
            return "READY TO HOLD - PRESS g", True

        return "STABILIZING...", False

    # =====================================================
    # Drawing
    # =====================================================

    def draw_results(self, frame, tips, grip_roi, arm_rects,
                     object_data, decision, close_allowed):

        output = frame.copy()

        # Draw gripper tips.
        if tips is not None:
            for name, tip in tips.items():
                x, y, w, h = tip["bbox"]
                cx, cy = tip["center"]

                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 140, 255), 2)
                cv2.circle(output, (cx, cy), 5, (0, 140, 255), -1)

                cv2.putText(
                    output,
                    name.upper() + " TIP",
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 140, 255),
                    2
                )

            lx, ly = tips["left"]["center"]
            rx, ry = tips["right"]["center"]
            cv2.line(output, (lx, ly), (rx, ry), (0, 140, 255), 2)

        # Draw gripper arm rectangles.
        if arm_rects is not None:
            left_bbox = arm_rects["left_bbox"]
            right_bbox = arm_rects["right_bbox"]

            if left_bbox is not None:
                x, y, w, h = left_bbox
                cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(
                    output,
                    "LEFT GRIPPER ARM",
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 0, 0),
                    2
                )

            if right_bbox is not None:
                x, y, w, h = right_bbox
                cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(
                    output,
                    "RIGHT GRIPPER ARM",
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 0, 0),
                    2
                )

        # Draw grip ROI.
        if grip_roi is not None:
            x1, y1, x2, y2 = grip_roi["bbox"]
            close_line_y = grip_roi["close_line_y"]

            cv2.rectangle(output, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.line(output, (x1, close_line_y), (x2, close_line_y), (0, 0, 255), 2)

            cv2.putText(
                output,
                "GRIP AREA",
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 0),
                2
            )

            cv2.putText(
                output,
                "CLOSE LINE",
                (x1, close_line_y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2
            )

            cx, cy = grip_roi["center"]
            cv2.circle(output, (cx, cy), 5, (255, 255, 0), -1)

        # Draw object rectangle only, no contour.
        if object_data is not None:
            x, y, w, h = object_data["bbox"]
            cx, cy = object_data["center"]

            color = (0, 255, 0) if close_allowed else (0, 255, 255)

            cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
            cv2.circle(output, (cx, cy), 5, color, -1)

            cv2.putText(
                output,
                "OBJECT",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2
            )

            cv2.putText(
                output,
                "Area: " + str(int(object_data["area"])) +
                " Fill: " + str(round(object_data["fill_ratio"], 2)),
                (20, 118),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2
            )

        # Decision text.
        if close_allowed:
            bg = (0, 130, 0)
        elif "MOVE" in decision:
            bg = (140, 90, 0)
        elif "CLOSER" in decision:
            bg = (140, 70, 0)
        else:
            bg = (0, 0, 150)

        self.draw_text(
            output,
            "Decision: " + decision,
            (20, 40),
            bg=bg,
            scale=0.85
        )

        self.draw_text(
            output,
            "Close allowed: " + str(close_allowed),
            (20, 82),
            bg=bg,
            scale=0.70
        )

        command_text = self.get_command_text()

        self.draw_text(
            output,
            "Command: " + command_text,
            (20, 160),
            bg=(50, 50, 50),
            scale=0.65
        )

        bg_status = "TABLE: SET" if self.table_hsv_center is not None else "TABLE: DEFAULT"
        ignore_status = "EMPTY MASK: SET" if self.empty_gripper_ignore_mask is not None else "EMPTY MASK: NOT SET"

        self.draw_text(
            output,
            bg_status + " | " + ignore_status,
            (20, output.shape[0] - 58),
            bg=(40, 40, 40),
            scale=0.55
        )

        self.draw_text(
            output,
            "Keys: t=calibrate table | i=empty gripper mask | g=grab | r=release | q=quit",
            (20, output.shape[0] - 22),
            bg=(40, 40, 40),
            scale=0.55
        )

        return output

    def set_command_text(self, text, duration=1.5):
        self.last_command_text = text
        self.last_command_until = time.monotonic() + duration

    def get_command_text(self):
        if time.monotonic() < self.last_command_until:
            return self.last_command_text

        return "IDLE"

    # =====================================================
    # Main Loop
    # =====================================================

    def run(self):
        cap = self.open_camera()

        cv2.namedWindow("Gripper Vision", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Orange Mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Object Mask", cv2.WINDOW_NORMAL)

        print("Gripper vision started.")
        print("Important:")
        print("- Use an object with a color different from the table/background.")
        print("- Avoid blue/orange/white objects for now.")
        print("")
        print("Keys:")
        print("t : calibrate table/background color")
        print("i : capture empty gripper ignore mask")
        print("g : grab command placeholder")
        print("r : release command placeholder")
        print("q : quit")

        while True:
            ret, frame = cap.read()

            if not ret:
                print("Frame could not be read.")
                break

            masks = self.get_color_masks(frame)

            tips = self.detect_orange_tips(frame, masks["orange"])

            grip_roi = None
            arm_rects = None
            object_data = None
            object_mask = np.zeros(frame.shape[:2], dtype=np.uint8)

            if tips is not None:
                grip_roi = self.make_grip_roi(frame, tips)

                arm_rects = self.detect_gripper_arm_rectangles(
                    frame,
                    tips,
                    masks["gripper"]
                )

                object_data, detected_object_mask = self.detect_object(
                    frame,
                    grip_roi,
                    masks
                )

                if detected_object_mask is not None and detected_object_mask.size > 0:
                    object_mask = detected_object_mask

            decision, close_allowed = self.get_decision(grip_roi, object_data)

            output = self.draw_results(
                frame,
                tips,
                grip_roi,
                arm_rects,
                object_data,
                decision,
                close_allowed
            )

            cv2.imshow("Gripper Vision", output)
            cv2.imshow("Orange Mask", masks["orange"])

            if object_mask is not None and object_mask.size > 0:
                cv2.imshow("Object Mask", object_mask)
            else:
                empty_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                cv2.imshow("Object Mask", empty_mask)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord("t"):
                self.calibrate_table(frame)
                self.set_command_text("TABLE CALIBRATED", duration=1.5)

            elif key == ord("i"):
                self.capture_empty_gripper_ignore_mask(frame, masks)
                self.set_command_text("EMPTY MASK CAPTURED", duration=1.5)

            elif key == ord("g"):
                if close_allowed:
                    self.set_command_text("GRAB / CLOSING", duration=2.0)
                    print("GRAB command placeholder. Later send close-with-FSR to Pico.")
                else:
                    self.set_command_text("NOT READY", duration=1.5)
                    print("Not ready to grab.")

            elif key == ord("r"):
                self.set_command_text("RELEASE", duration=2.0)
                print("RELEASE command placeholder. Later send release/open to Pico.")

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    vision = GripperVision(camera_index=2)
    vision.run()