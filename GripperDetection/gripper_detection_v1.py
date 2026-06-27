import cv2
import numpy as np
import math


class GripperVision:
    def __init__(self, camera_index=2):
        self.camera_index = camera_index

        self.frame_width = 1280
        self.frame_height = 720

        self.table_hsv_center = None
        self.gripper_ignore_mask = None

        self.clicked_point = None

        self.tracker = None
        self.tracker_name = None
        self.selected_center = None
        self.selected_bbox = None
        self.selected_source = None
        self.lost_counter = 0

        self.ready_counter = 0
        self.ready_frames_required = 6

        self.min_tip_area = 35
        self.min_object_area = 350

        # Obje yakınlık ayarları
        self.close_object_area_ratio = 0.018
        self.close_line_ratio = 0.28

        # Grip ROI ayarları
        self.roi_top_ratio = 0.30
        self.roi_bottom_ratio = 0.65

    def open_camera(self):
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        cap.set(cv2.CAP_PROP_FPS, 30)

        if not cap.isOpened():
            raise RuntimeError("Camera could not be opened.")

        return cap

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_point = (x, y)

    def clean_mask(self, mask, kernel_size=5):
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def hsv_mask(self, hsv, lower, upper):
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        return cv2.inRange(hsv, lower, upper)

    def get_color_masks(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Turuncu bantlar
        orange_mask = self.hsv_mask(hsv, [5, 70, 60], [30, 255, 255])

        # Mavi gripper parçaları
        blue_mask = self.hsv_mask(hsv, [85, 50, 40], [135, 255, 255])

        # Beyaz gripper parçaları
        white_mask = self.hsv_mask(hsv, [0, 0, 145], [180, 90, 255])

        orange_mask = self.clean_mask(orange_mask, 5)
        blue_mask = self.clean_mask(blue_mask, 5)
        white_mask = self.clean_mask(white_mask, 5)

        return orange_mask, blue_mask, white_mask

    def get_gripper_color_mask(self, frame):
        orange_mask, blue_mask, white_mask = self.get_color_masks(frame)

        # Önce mavi + turuncu ile gripper'ın genel yerini buluyoruz.
        seed_mask = cv2.bitwise_or(orange_mask, blue_mask)

        contours, _ = cv2.findContours(
            seed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        h, w = frame.shape[:2]

        if len(contours) > 0:
            valid = [c for c in contours if cv2.contourArea(c) > 40]

            if len(valid) > 0:
                all_points = np.vstack(valid)
                x, y, bw, bh = cv2.boundingRect(all_points)

                margin = 90

                x1 = max(0, x - margin)
                y1 = max(0, y - margin)
                x2 = min(w - 1, x + bw + margin)
                y2 = min(h - 1, y + bh + margin)

                white_near_gripper = np.zeros_like(white_mask)
                white_near_gripper[y1:y2, x1:x2] = white_mask[y1:y2, x1:x2]

                gripper_mask = cv2.bitwise_or(seed_mask, white_near_gripper)
            else:
                gripper_mask = cv2.bitwise_or(seed_mask, white_mask)
        else:
            gripper_mask = cv2.bitwise_or(seed_mask, white_mask)

        gripper_mask = self.clean_mask(gripper_mask, 5)
        gripper_mask = cv2.dilate(gripper_mask, np.ones((7, 7), np.uint8), iterations=1)

        return gripper_mask, orange_mask

    def detect_gripper_bbox(self, gripper_mask):
        contours, _ = cv2.findContours(
            gripper_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        valid = [c for c in contours if cv2.contourArea(c) > 300]

        if len(valid) == 0:
            return None

        all_points = np.vstack(valid)
        x, y, w, h = cv2.boundingRect(all_points)

        return {
            "bbox": (x, y, w, h),
            "center": (x + w // 2, y + h // 2),
            "contours": valid
        }

    def detect_orange_tips(self, orange_mask):
        contours, _ = cv2.findContours(
            orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        tip_candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < self.min_tip_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2

            tip_candidates.append({
                "contour": cnt,
                "area": area,
                "bbox": (x, y, w, h),
                "center": (cx, cy)
            })

        if len(tip_candidates) < 2:
            return None

        tip_candidates = sorted(tip_candidates, key=lambda p: p["area"], reverse=True)
        tips = tip_candidates[:2]
        tips = sorted(tips, key=lambda p: p["center"][0])

        return {
            "left": tips[0],
            "right": tips[1]
        }

    def make_grip_roi(self, frame, tips, gripper_data=None):
        h, w = frame.shape[:2]

        if tips is not None:
            lx, ly = tips["left"]["center"]
            rx, ry = tips["right"]["center"]

            gap_width = abs(rx - lx)

            if gap_width < 50:
                return None

            center_y = int((ly + ry) / 2)

            roi_x1 = int(min(lx, rx) + 0.06 * gap_width)
            roi_x2 = int(max(lx, rx) - 0.06 * gap_width)

            roi_y1 = int(center_y - self.roi_top_ratio * gap_width)
            roi_y2 = int(center_y + self.roi_bottom_ratio * gap_width)

            close_line_y = int(center_y + self.close_line_ratio * gap_width)

        elif gripper_data is not None:
            # Turuncu bant bulunamazsa fallback olarak genel gripper kutusundan ROI üretir.
            x, y, gw, gh = gripper_data["bbox"]

            gap_width = gw

            roi_x1 = int(x + 0.22 * gw)
            roi_x2 = int(x + 0.78 * gw)
            roi_y1 = int(y)
            roi_y2 = int(y + 0.80 * gh)

            close_line_y = int(y + 0.48 * gh)

        else:
            return None

        roi_x1 = max(0, roi_x1)
        roi_x2 = min(w - 1, roi_x2)
        roi_y1 = max(0, roi_y1)
        roi_y2 = min(h - 1, roi_y2)
        close_line_y = max(0, min(h - 1, close_line_y))

        return {
            "bbox": (roi_x1, roi_y1, roi_x2, roi_y2),
            "gap_width": gap_width,
            "center": ((roi_x1 + roi_x2) // 2, (roi_y1 + roi_y2) // 2),
            "close_line_y": close_line_y
        }

    def calibrate_table(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]

        patch_h = int(0.18 * h)
        patch_w = int(0.25 * w)

        top_left = hsv[0:patch_h, 0:patch_w]
        top_right = hsv[0:patch_h, w - patch_w:w]

        samples = np.vstack([
            top_left.reshape(-1, 3),
            top_right.reshape(-1, 3)
        ])

        self.table_hsv_center = np.median(samples, axis=0).astype(np.uint8)
        print("Table calibrated:", self.table_hsv_center)

    def get_table_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if self.table_hsv_center is None:
            lower_table = np.array([5, 20, 40], dtype=np.uint8)
            upper_table = np.array([45, 180, 245], dtype=np.uint8)
            table_mask = cv2.inRange(hsv, lower_table, upper_table)
            return self.clean_mask(table_mask, 7)

        h0, s0, v0 = self.table_hsv_center

        h_tol = 16
        s_tol = 75
        v_tol = 85

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
        table_mask = self.clean_mask(table_mask, 7)

        return table_mask

    def capture_gripper_ignore_mask(self, frame):
        table_mask = self.get_table_mask(frame)
        gripper_mask, _ = self.get_gripper_color_mask(frame)

        not_table = cv2.bitwise_not(table_mask)

        ignore_mask = cv2.bitwise_or(not_table, gripper_mask)
        ignore_mask = cv2.dilate(ignore_mask, np.ones((11, 11), np.uint8), iterations=1)

        self.gripper_ignore_mask = ignore_mask

        print("Empty gripper ignore mask captured.")

    def detect_object_candidates(self, frame, gripper_mask):
        h, w = frame.shape[:2]

        table_mask = self.get_table_mask(frame)
        object_mask = cv2.bitwise_not(table_mask)

        if self.gripper_ignore_mask is not None:
            object_mask[self.gripper_ignore_mask > 0] = 0
        else:
            object_mask[gripper_mask > 0] = 0

        object_mask = self.clean_mask(object_mask, 5)

        contours, _ = cv2.findContours(
            object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < self.min_object_area:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)

            if bw < 10 or bh < 10:
                continue

            # Çok büyük alanları elemek için
            if area > 0.60 * h * w:
                continue

            candidates.append({
                "bbox": (x, y, bw, bh),
                "center": (x + bw // 2, y + bh // 2),
                "area": area,
                "contour": cnt,
                "selected": False,
                "source": "segmentation"
            })

        candidates = sorted(candidates, key=lambda obj: obj["area"], reverse=True)

        return candidates, object_mask

    def point_inside_bbox(self, point, bbox, margin=8):
        px, py = point
        x, y, w, h = bbox

        return (
            x - margin <= px <= x + w + margin and
            y - margin <= py <= y + h + margin
        )

    def create_tracker(self):
        tracker_creators = [
            ("CSRT", "TrackerCSRT_create"),
            ("KCF", "TrackerKCF_create"),
            ("MIL", "TrackerMIL_create")
        ]

        for name, creator in tracker_creators:
            if hasattr(cv2, creator):
                return getattr(cv2, creator)(), name

            if hasattr(cv2, "legacy") and hasattr(cv2.legacy, creator):
                return getattr(cv2.legacy, creator)(), name

        return None, None

    def init_tracker(self, frame, bbox, source_name="manual"):
        x, y, w, h = [int(v) for v in bbox]

        if w <= 5 or h <= 5:
            return False

        tracker, tracker_name = self.create_tracker()

        if tracker is None:
            print("Tracker bulunamadı. Daha iyi takip için şunu kurabilirsin:")
            print("pip install opencv-contrib-python")
            self.tracker = None
            self.tracker_name = None
            self.selected_bbox = (x, y, w, h)
            self.selected_center = (x + w // 2, y + h // 2)
            self.selected_source = "bbox_fallback"
            return True

        ok = tracker.init(frame, (x, y, w, h))

        if not ok:
            print("Tracker init failed.")
            return False

        self.tracker = tracker
        self.tracker_name = tracker_name
        self.selected_bbox = (x, y, w, h)
        self.selected_center = (x + w // 2, y + h // 2)
        self.selected_source = source_name
        self.lost_counter = 0

        print(f"Selected object with {tracker_name} tracker.")

        return True

    def clear_selection(self):
        self.tracker = None
        self.tracker_name = None
        self.selected_center = None
        self.selected_bbox = None
        self.selected_source = None
        self.lost_counter = 0
        self.ready_counter = 0
        print("Selection cleared.")

    def handle_mouse_selection(self, frame, candidates):
        if self.clicked_point is None:
            return

        click = self.clicked_point
        self.clicked_point = None

        selected = None

        for obj in candidates:
            if self.point_inside_bbox(click, obj["bbox"]):
                selected = obj
                break

        if selected is not None:
            self.init_tracker(frame, selected["bbox"], source_name="click")
        else:
            print("Clicked point is not inside any detected object.")

    def get_selected_object(self, frame, candidates, grip_roi):
        if self.tracker is not None:
            ok, bbox = self.tracker.update(frame)

            if ok:
                x, y, w, h = [int(v) for v in bbox]

                if w > 5 and h > 5:
                    self.selected_bbox = (x, y, w, h)
                    self.selected_center = (x + w // 2, y + h // 2)
                    self.lost_counter = 0

                    return {
                        "bbox": self.selected_bbox,
                        "center": self.selected_center,
                        "area": float(w * h),
                        "selected": True,
                        "source": f"tracker_{self.tracker_name}"
                    }

            self.lost_counter += 1

            if self.lost_counter > 20:
                print("Tracker lost object.")
                self.clear_selection()

            return None

        # Tracker yoksa, seçili merkeze en yakın segmentasyon objesiyle takip etmeyi dene.
        if self.selected_center is None:
            return None

        if len(candidates) == 0:
            self.lost_counter += 1
            return None

        sx, sy = self.selected_center

        best_obj = None
        best_dist = 1e9

        for obj in candidates:
            cx, cy = obj["center"]
            dist = math.sqrt((cx - sx) ** 2 + (cy - sy) ** 2)

            if dist < best_dist:
                best_dist = dist
                best_obj = obj

        if grip_roi is not None:
            max_dist = max(90, 0.45 * grip_roi["gap_width"])
        else:
            max_dist = 130

        if best_obj is not None and best_dist < max_dist:
            best_obj["selected"] = True
            self.selected_center = best_obj["center"]
            self.selected_bbox = best_obj["bbox"]
            self.lost_counter = 0
            return best_obj

        self.lost_counter += 1

        if self.lost_counter > 20:
            self.clear_selection()

        return None

    def is_close_ready(self, selected_object, grip_roi):
        if selected_object is None or grip_roi is None:
            return False

        x, y, w, h = selected_object["bbox"]
        cx, cy = selected_object["center"]
        area = selected_object["area"]

        roi_x1, roi_y1, roi_x2, roi_y2 = grip_roi["bbox"]
        close_line_y = grip_roi["close_line_y"]
        gap_width = grip_roi["gap_width"]

        obj_x1 = x
        obj_y1 = y
        obj_x2 = x + w
        obj_y2 = y + h

        inter_x1 = max(obj_x1, roi_x1)
        inter_y1 = max(obj_y1, roi_y1)
        inter_x2 = min(obj_x2, roi_x2)
        inter_y2 = min(obj_y2, roi_y2)

        inter_w = max(0, inter_x2 - inter_x1)
        inter_h = max(0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        bbox_area = max(1, w * h)
        inside_ratio = inter_area / bbox_area

        center_inside_roi = (
            roi_x1 <= cx <= roi_x2 and
            roi_y1 <= cy <= roi_y2
        )

        bottom_reached_close_line = obj_y2 >= close_line_y

        close_area_threshold = max(
            1000,
            self.close_object_area_ratio * gap_width * gap_width
        )

        large_enough = area >= close_area_threshold

        close_ready = (
            (center_inside_roi or inside_ratio > 0.30) and
            bottom_reached_close_line and
            large_enough
        )

        return close_ready

    def get_decision(self, selected_object, grip_roi):
        if selected_object is None:
            self.ready_counter = 0

            if self.selected_center is None:
                return "WAIT_OBJECT_SELECTION", False

            return "SEARCH_SELECTED_OBJECT", False

        close_ready = self.is_close_ready(selected_object, grip_roi)

        if close_ready:
            self.ready_counter += 1
        else:
            self.ready_counter = 0

        stable_ready = self.ready_counter >= self.ready_frames_required

        if stable_ready:
            return "READY_TO_CLOSE", True

        return "APPROACH_SELECTED_OBJECT", False

    def draw_results(
        self,
        frame,
        gripper_data,
        tips,
        grip_roi,
        candidates,
        selected_object,
        object_mask,
        decision,
        close_allowed
    ):
        output = frame.copy()

        if gripper_data is not None:
            x, y, w, h = gripper_data["bbox"]
            cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(
                output,
                "Gripper blue+white+orange",
                (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 0, 0),
                2
            )

        if tips is not None:
            for name, tip in tips.items():
                x, y, w, h = tip["bbox"]
                cx, cy = tip["center"]

                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 140, 255), 2)
                cv2.circle(output, (cx, cy), 5, (0, 140, 255), -1)
                cv2.putText(
                    output,
                    name,
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 140, 255),
                    2
                )

            lx, ly = tips["left"]["center"]
            rx, ry = tips["right"]["center"]
            cv2.line(output, (lx, ly), (rx, ry), (0, 140, 255), 2)

        if grip_roi is not None:
            x1, y1, x2, y2 = grip_roi["bbox"]
            close_line_y = grip_roi["close_line_y"]

            cv2.rectangle(output, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.line(output, (x1, close_line_y), (x2, close_line_y), (0, 0, 255), 2)

            cv2.putText(
                output,
                "Grip ROI",
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 0),
                2
            )

        for i, obj in enumerate(candidates):
            x, y, w, h = obj["bbox"]

            cv2.rectangle(output, (x, y), (x + w, y + h), (0, 220, 220), 1)
            cv2.putText(
                output,
                f"obj {i}",
                (x, max(20, y - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 220, 220),
                1
            )

        if selected_object is not None:
            x, y, w, h = selected_object["bbox"]
            cx, cy = selected_object["center"]

            color = (255, 0, 255)

            cv2.rectangle(output, (x, y), (x + w, y + h), color, 3)
            cv2.circle(output, (cx, cy), 6, color, -1)

            cv2.putText(
                output,
                "SELECTED",
                (x, max(25, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                color,
                2
            )

            cv2.putText(
                output,
                f"Selected area: {int(selected_object['area'])}",
                (20, 115),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2
            )

        status_color = (0, 255, 0) if close_allowed else (0, 0, 255)

        cv2.putText(
            output,
            f"Decision: {decision}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            status_color,
            2
        )

        cv2.putText(
            output,
            f"Close allowed: {close_allowed}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            status_color,
            2
        )

        tracker_text = "None" if self.tracker_name is None else self.tracker_name

        cv2.putText(
            output,
            f"Tracker: {tracker_text}",
            (20, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

        if self.table_hsv_center is None:
            table_text = "Table: not calibrated, press t"
            table_color = (0, 0, 255)
        else:
            table_text = "Table: calibrated"
            table_color = (0, 255, 0)

        cv2.putText(
            output,
            table_text,
            (20, 185),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            table_color,
            2
        )

        if self.gripper_ignore_mask is None:
            ignore_text = "Empty gripper mask: not captured, press i"
            ignore_color = (0, 0, 255)
        else:
            ignore_text = "Empty gripper mask: captured"
            ignore_color = (0, 255, 0)

        cv2.putText(
            output,
            ignore_text,
            (20, 220),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            ignore_color,
            2
        )

        cv2.putText(
            output,
            "Left click=select detected object | s=manual ROI select | c=clear | t=table | i=empty gripper | q=quit",
            (20, output.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2
        )

        return output

    def manual_roi_select(self, frame):
        bbox = cv2.selectROI(
            "Manual ROI Selection",
            frame,
            showCrosshair=True,
            fromCenter=False
        )

        cv2.destroyWindow("Manual ROI Selection")

        x, y, w, h = [int(v) for v in bbox]

        if w > 5 and h > 5:
            self.init_tracker(frame, (x, y, w, h), source_name="manual_roi")
        else:
            print("Manual ROI cancelled.")

    def run(self):
        cap = self.open_camera()

        cv2.namedWindow("Gripper Vision", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Object Mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Gripper Mask", cv2.WINDOW_NORMAL)

        cv2.setMouseCallback("Gripper Vision", self.on_mouse)

        while True:
            ret, frame = cap.read()

            if not ret:
                print("Frame could not be read.")
                break

            gripper_mask, orange_mask = self.get_gripper_color_mask(frame)
            gripper_data = self.detect_gripper_bbox(gripper_mask)
            tips = self.detect_orange_tips(orange_mask)

            grip_roi = self.make_grip_roi(frame, tips, gripper_data)

            candidates, object_mask = self.detect_object_candidates(frame, gripper_mask)

            self.handle_mouse_selection(frame, candidates)

            selected_object = self.get_selected_object(frame, candidates, grip_roi)

            decision, close_allowed = self.get_decision(selected_object, grip_roi)

            output = self.draw_results(
                frame,
                gripper_data,
                tips,
                grip_roi,
                candidates,
                selected_object,
                object_mask,
                decision,
                close_allowed
            )

            cv2.imshow("Gripper Vision", output)
            cv2.imshow("Object Mask", object_mask)
            cv2.imshow("Gripper Mask", gripper_mask)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord("t"):
                self.calibrate_table(frame)

            elif key == ord("i"):
                self.capture_gripper_ignore_mask(frame)

            elif key == ord("s"):
                self.manual_roi_select(frame)

            elif key == ord("c"):
                self.clear_selection()

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    vision = GripperVision(camera_index=2)
    vision.run()