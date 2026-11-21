import argparse
import math
import os
import sys
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

try:  # optional: real monospace font for the on-screen text
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    Image = None

HEAD_FRAC = 0.915  # used for frame boundary clipping check and selftest
FULL_BODY_IDS = (0, 11, 12, 23, 24, 25, 26, 27, 28, 29, 30)  # nose, shoulders, hips, knees, ankles, heels
PALM_IDS = [0, 5, 9, 13, 17]  # wrist + the four knuckles at the base of the fingers
FONT_SIZE = 14
TEXT_COLOR = (255, 255, 255)


# --------------------------------------------------------------------- text
def _load_mono_font(size: int):
    if Image is None:
        return None
    for path in (r"C:\Windows\Fonts\consola.ttf", r"C:\Windows\Fonts\lucon.ttf",
                 r"C:\Windows\Fonts\cour.ttf", "/System/Library/Fonts/Menlo.ttc",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"):
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
    return None


_MONO_FONT = _load_mono_font(FONT_SIZE)


def draw_texts(frame, items) -> None:
    """Draw small white monospace text. items = [(text, (x, baseline_y)), ...]."""
    if not items:
        return
    if _MONO_FONT is not None:
        img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(img)
        for text, org in items:
            d.text((int(org[0]), int(org[1])), text, font=_MONO_FONT,
                   fill=TEXT_COLOR, anchor="ls")
        frame[:] = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    else:
        for text, org in items:
            cv2.putText(frame, text, (int(org[0]), int(org[1])),
                        cv2.FONT_HERSHEY_PLAIN, 1.0, TEXT_COLOR, 1, cv2.LINE_AA)


# ---------------------------------------------------------------- stereo rig
class StereoRig:
    """Calibrated pair of cameras. Triangulates matching pixels into 3D metres."""

    def __init__(self, K1, D1, K2, D2, R, T, size):
        self.K1 = np.asarray(K1, np.float64)
        self.D1 = np.asarray(D1, np.float64).reshape(-1)
        self.K2 = np.asarray(K2, np.float64)
        self.D2 = np.asarray(D2, np.float64).reshape(-1)
        self.R = np.asarray(R, np.float64)
        self.T = np.asarray(T, np.float64).reshape(3, 1)
        self.size = (int(size[0]), int(size[1]))
        # Work in normalised (undistorted, focal-length-free) image coordinates.
        self.P1 = np.hstack([np.eye(3), np.zeros((3, 1))])
        self.P2 = np.hstack([self.R, self.T])
        self.baseline_m = float(np.linalg.norm(self.T))

    @classmethod
    def from_file(cls, path: str) -> "StereoRig":
        d = np.load(path)
        return cls(d["K1"], d["D1"], d["K2"], d["D2"], d["R"], d["T"], d["size"])

    @staticmethod
    def _normalise(px, K, D):
        p = np.asarray(px, np.float64).reshape(-1, 1, 2)
        return cv2.undistortPoints(p, K, D).reshape(-1, 2)

    def triangulate(self, px_left, px_right) -> Tuple[np.ndarray, np.ndarray]:
        """
        px_left / px_right: Nx2 pixel coordinates of the SAME N points.
        Returns (X, err): X is Nx3 in metres (left-camera frame); err is the
        reprojection error in pixels (large = the two views disagree = a bad match).
        """
        nl = self._normalise(px_left, self.K1, self.D1)
        nr = self._normalise(px_right, self.K2, self.D2)
        Xh = cv2.triangulatePoints(self.P1, self.P2, nl.T, nr.T)
        with np.errstate(all="ignore"):
            X = (Xh[:3] / Xh[3]).T
            XR = (self.R @ X.T + self.T).T
            eL = np.linalg.norm(X[:, :2] / X[:, 2:3] - nl, axis=1) * self.K1[0, 0]
            eR = np.linalg.norm(XR[:, :2] / XR[:, 2:3] - nr, axis=1) * self.K2[0, 0]
            err = np.maximum(eL, eR)
        bad = ~np.isfinite(err) | (X[:, 2] <= 0) | (XR[:, 2] <= 0)
        err[bad] = np.inf  # behind a camera or degenerate
        return X, err


# --------------------------------------------------------------- calibration
def compute_calibration(obj_pts, pts_l, pts_r, size):
    """Intrinsics for each camera, then the rotation/translation between them."""
    rms_l, K1, D1, _, _ = cv2.calibrateCamera(obj_pts, pts_l, size, None, None)
    rms_r, K2, D2, _, _ = cv2.calibrateCamera(obj_pts, pts_r, size, None, None)
    crit = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 200, 1e-7)
    rms, K1, D1, K2, D2, R, T, _, _ = cv2.stereoCalibrate(
        obj_pts, pts_l, pts_r, K1, D1, K2, D2, size,
        flags=cv2.CALIB_FIX_INTRINSIC, criteria=crit)
    return {"K1": K1, "D1": D1, "K2": K2, "D2": D2, "R": R, "T": T,
            "rms_left": rms_l, "rms_right": rms_r, "rms_stereo": rms}


def _find_corners(gray, pattern):
    if hasattr(cv2, "findChessboardCornersSB"):
        return cv2.findChessboardCornersSB(
            gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
    ok, c = cv2.findChessboardCorners(
        gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    return ok, c


def open_camera(index: int, width: int, height: int):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {index}")
    # MJPG leaves room for two cameras on one USB controller.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # refocusing changes the focal length!
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def grab_pair(capL, capR):
    """grab() both first, THEN decode, so the two exposures are as close as possible."""
    if not (capL.grab() and capR.grab()):
        return None
    t = time.time()
    okL, fL = capL.retrieve()
    okR, fR = capR.retrieve()
    return (fL, fR, t) if okL and okR else None


def run_calibrate(args):
    cols, rows = args.board
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= args.square_mm / 1000.0  # metres

    capL = open_camera(args.cam_left, args.width, args.height)
    capR = open_camera(args.cam_right, args.width, args.height)
    obj_pts, pts_l, pts_r, size = [], [], [], None
    print("SPACE = capture a pair (needs the board visible in BOTH), ENTER = finish, q = quit")

    try:
        while True:
            pair = grab_pair(capL, capR)
            if pair is None:
                print("Camera frame not received.")
                break
            fL, fR, _ = pair
            if fL.shape != fR.shape:
                raise RuntimeError("The two cameras are returning different frame sizes.")
            size = (fL.shape[1], fL.shape[0])
            okL, cL = _find_corners(cv2.cvtColor(fL, cv2.COLOR_BGR2GRAY), (cols, rows))
            okR, cR = _find_corners(cv2.cvtColor(fR, cv2.COLOR_BGR2GRAY), (cols, rows))
            visL, visR = fL.copy(), fR.copy()
            if okL:
                cv2.drawChessboardCorners(visL, (cols, rows), cL, okL)
            if okR:
                cv2.drawChessboardCorners(visR, (cols, rows), cR, okR)
            ready = okL and okR
            draw_texts(visL, [(f"pairs: {len(obj_pts)}/{args.min_pairs}+   "
                               f"{'READY - press SPACE' if ready else 'board not seen by both'}",
                               (10, 25))])
            both = np.hstack([visL, visR])
            scale = min(1.0, 1500 / both.shape[1])
            cv2.imshow("Stereo calibration", cv2.resize(both, None, fx=scale, fy=scale))
            key = cv2.waitKey(1) & 0xFF
            if key == 32 and ready:
                obj_pts.append(objp)
                pts_l.append(cL.reshape(-1, 1, 2).astype(np.float32))
                pts_r.append(cR.reshape(-1, 1, 2).astype(np.float32))
                print(f"captured pair {len(obj_pts)}")
            elif key in (13, ord("q"), 27):
                break
    finally:
        capL.release()
        capR.release()
        cv2.destroyAllWindows()

    if len(obj_pts) < args.min_pairs:
        sys.exit(f"Only {len(obj_pts)} pairs captured; need at least {args.min_pairs}.")

    cal = compute_calibration(obj_pts, pts_l, pts_r, size)
    np.savez(args.out, K1=cal["K1"], D1=cal["D1"], K2=cal["K2"], D2=cal["D2"],
             R=cal["R"], T=cal["T"], size=np.array(size))
    baseline_cm = float(np.linalg.norm(cal["T"])) * 100
    print(f"\nSaved {args.out}")
    print(f"  left camera RMS error : {cal['rms_left']:.3f} px")
    print(f"  right camera RMS error: {cal['rms_right']:.3f} px")
    print(f"  stereo RMS error      : {cal['rms_stereo']:.3f} px   (aim for < 0.5)")
    print(f"  focal length          : {cal['K1'][0, 0]:.1f} px (left), {cal['K2'][0, 0]:.1f} px (right)")
    print(f"  baseline              : {baseline_cm:.2f} cm   <- compare with a tape measure")
    if cal["rms_stereo"] > 1.0:
        print("  WARNING: error is high. Retake the pairs: flat rigid board, sharp images,"
              " more varied tilts and positions, no motion blur.")


# ------------------------------------------------------------------ helpers
class _EMA:
    def __init__(self, alpha=0.3):
        self.alpha, self.v = alpha, {}

    def __call__(self, key, value):
        prev = self.v.get(key)
        self.v[key] = value if prev is None else self.alpha * value + (1 - self.alpha) * prev
        return self.v[key]

    def keep_only(self, keys):
        for k in list(self.v):
            if k not in keys:
                del self.v[k]


class VelocityTracker:
    """3D velocity of a point (m/s) from a short sliding window, with a dead-band."""

    def __init__(self, window_s=0.6, alpha=0.4, deadband=0.08, max_gap_s=0.6):
        self.window_s, self.alpha = window_s, alpha
        self.deadband, self.max_gap_s = deadband, max_gap_s
        self.hist: deque = deque()
        self.v = np.zeros(3)

    def reset(self):
        self.hist.clear()
        self.v = np.zeros(3)

    def update(self, pos: Optional[np.ndarray], t: float) -> Optional[Dict]:
        if pos is None:
            self.reset()
            return None
        if self.hist and t - self.hist[-1][0] > self.max_gap_s:
            self.reset()
        self.hist.append((t, pos))
        while len(self.hist) > 1 and t - self.hist[0][0] > self.window_s:
            self.hist.popleft()
        t0, p0 = self.hist[0]
        if t - t0 >= 0.2:
            self.v = self.v + self.alpha * ((pos - p0) / (t - t0) - self.v)
        speed = float(np.linalg.norm(self.v))
        if speed < self.deadband:
            return {"speed": 0.0, "v": np.zeros(3)}
        return {"speed": speed, "v": self.v.copy()}


# ------------------------------------------------------------ stereo tracker
class StereoTracker:
    def __init__(self, rig: StereoRig, pose_model=1, min_vis=0.6, max_reproj_px=3.0,
                 height_offset_m=0.0, floor_offset_m=0.0):
        try:
            import mediapipe as mp
            self._mp_pose = mp.solutions.pose
            self._mp_hands = mp.solutions.hands
        except (ImportError, AttributeError) as exc:
            raise SystemExit("MediaPipe with the 'solutions' API is required.\n"
                             'Install it with: pip install "mediapipe==0.10.21"') from exc
        self.rig = rig
        self.min_vis = min_vis
        self.max_reproj_px = max_reproj_px
        self.height_offset_m = height_offset_m
        self.floor_offset_m = floor_offset_m

        def pose():
            return self._mp_pose.Pose(model_complexity=pose_model, smooth_landmarks=True,
                                      min_detection_confidence=0.5, min_tracking_confidence=0.5)

        def hands():
            return self._mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5,
                                        min_tracking_confidence=0.5)

        self._pose_l, self._pose_r = pose(), pose()
        self._hands_l, self._hands_r = hands(), hands()
        self._res: Dict = {}
        self.reset()

    def reset(self):
        self._ema = _EMA(0.3)
        self._vel = VelocityTracker()
        self._heights: deque = deque()    # (t, metres)
        self._cam_hs: deque = deque()     # (t, metres)
        self._up: Optional[np.ndarray] = None
        self._up_n = 0

    # ---------------------------------------------------------------- body
    def _body(self, fL, fR):
        hL, wL = fL.shape[:2]
        hR, wR = fR.shape[:2]
        resL = self._pose_l.process(cv2.cvtColor(fL, cv2.COLOR_BGR2RGB))
        resR = self._pose_r.process(cv2.cvtColor(fR, cv2.COLOR_BGR2RGB))
        self._res["pose"] = (resL, resR)
        if not (resL.pose_landmarks and resR.pose_landmarks):
            return None
        pL = np.array([[l.x * wL, l.y * hL] for l in resL.pose_landmarks.landmark])
        pR = np.array([[l.x * wR, l.y * hR] for l in resR.pose_landmarks.landmark])
        vis = np.minimum([l.visibility for l in resL.pose_landmarks.landmark],
                         [l.visibility for l in resR.pose_landmarks.landmark])
        inside = ((pL[:, 0] >= 0) & (pL[:, 0] < wL) & (pL[:, 1] >= 0) & (pL[:, 1] < hL) &
                  (pR[:, 0] >= 0) & (pR[:, 0] < wR) & (pR[:, 1] >= 0) & (pR[:, 1] < hR))
        idx = np.flatnonzero((vis >= self.min_vis) & inside)
        if len(idx) < 4:
            return None
        X, err = self.rig.triangulate(pL[idx], pR[idx])
        good = err <= self.max_reproj_px  # drop landmarks the two views disagree on
        if good.sum() < 4:
            return None
        pts = {int(i): X[k] for k, i in enumerate(idx) if good[k]}
        return pts, pL, pR, float(np.median(err[good])), (hL, hR)

    @staticmethod
    def _full_body(pts, pL, pR, hs) -> bool:
        if not all(i in pts for i in FULL_IDS_SET):
            return False
        for p, h in ((pL, hs[0]), (pR, hs[1])):
            nose_y = p[0][1]
            heel_y = (p[29][1] + p[30][1]) / 2
            head_top = nose_y - 0.085 * (heel_y - nose_y) / HEAD_FRAC
            if head_top < 0 or max(p[27][1], p[28][1], p[29][1], p[30][1]) > 0.99 * h:
                return False  # head or feet cut off
        return True

    # --------------------------------------------------------------- hands
    def _hands(self, fL, fR):
        hL, wL = fL.shape[:2]
        hR, wR = fR.shape[:2]
        resL = self._hands_l.process(cv2.cvtColor(fL, cv2.COLOR_BGR2RGB))
        resR = self._hands_r.process(cv2.cvtColor(fR, cv2.COLOR_BGR2RGB))
        self._res["hands"] = (resL, resR)

        def collect(res, w, h):
            out = {}
            for lms, hd in zip(res.multi_hand_landmarks or [], res.multi_handedness or []):
                label = hd.classification[0].label
                if label not in out:
                    out[label] = np.array([[l.x * w, l.y * h] for l in lms.landmark])
            return out

        dL, dR = collect(resL, wL, hL), collect(resR, wR, hR)
        hands = []
        for label in sorted(dL.keys() & dR.keys()):
            X, err = self.rig.triangulate(dL[label][PALM_IDS], dR[label][PALM_IDS])
            good = err <= self.max_reproj_px
            if good.sum() < 3:
                continue
            p = X[good].mean(axis=0)
            # MediaPipe assumes a mirrored (selfie) image; ours is not, so swap.
            shown = "Right" if label == "Left" else "Left"
            hands.append({"label": shown, "pos": p, "wrist_px": dL[label][0],
                          "distance_m": self._ema(f"hand_{shown}", float(np.linalg.norm(p)))})
        self._ema.keep_only({"body", *(f"hand_{h['label']}" for h in hands)})
        return hands

    # -------------------------------------------------------------- update
    def update(self, fL, fR, t: float) -> Dict:
        out = {"distance_m": None, "depth_m": None, "height_m": None, "cam_height_m": None,
               "tilt_deg": None, "velocity": None, "hands": [], "reproj_px": None,
               "full_body": False}
        body = self._body(fL, fR)
        ref = None
        if body is not None:
            pts, pL, pR, med_err, hs = body
            out["reproj_px"] = med_err
            if 23 in pts and 24 in pts:
                ref = (pts[23] + pts[24]) / 2          # hip centre
            elif 11 in pts and 12 in pts:
                ref = (pts[11] + pts[12]) / 2          # shoulder centre
            if ref is not None:
                out["distance_m"] = self._ema("body", float(np.linalg.norm(ref)))
                out["depth_m"] = float(ref[2])
            if self._full_body(pts, pL, pR, hs):
                out["full_body"] = True
                heel = (pts[29] + pts[30]) / 2.0
                ankle = (pts[27] + pts[28]) / 2.0
                knee = (pts[25] + pts[26]) / 2.0
                hip = (pts[23] + pts[24]) / 2.0
                shoulder = (pts[11] + pts[12]) / 2.0
                eye_mid = (pts[2] + pts[5]) / 2.0 if (2 in pts and 5 in pts) else pts[0]

                # Multi-segment 3D skeletal summation (resilient to posture/proportions)
                d_foot = float(np.linalg.norm(ankle - heel))
                d_shank = float(np.linalg.norm(knee - ankle))
                d_thigh = float(np.linalg.norm(hip - knee))
                d_torso = float(np.linalg.norm(shoulder - hip))
                d_neck_head = float(np.linalg.norm(eye_mid - shoulder))
                d_cranial_top = 0.12 * (d_shank + d_thigh + d_torso)

                total_h = d_foot + d_shank + d_thigh + d_torso + d_neck_head + d_cranial_top
                self._heights.append((t, total_h + self.height_offset_m))

                axis = pts[0] - heel
                length = float(np.linalg.norm(axis))
                u = axis / length                       # body axis = "up" (person assumed upright)
                self._up_n += 1
                a = max(1.0 / self._up_n, 0.01)
                self._up = u if self._up is None else self._normalise((1 - a) * self._up + a * u)
                self._cam_hs.append((t, float(-self._up @ heel) + self.floor_offset_m))
        out["velocity"] = self._vel.update(ref, t)

        for dq, horizon, maxlen in ((self._heights, 3.0, 90), (self._cam_hs, 10.0, 300)):
            while dq and t - dq[0][0] > horizon:
                dq.popleft()
            while len(dq) > maxlen:
                dq.popleft()
        if out["full_body"] and self._heights:
            out["height_m"] = float(np.median([v for _, v in self._heights]))  # median beats outliers
        if self._cam_hs:
            out["cam_height_m"] = float(np.median([v for _, v in self._cam_hs]))
        if self._up is not None:
            out["tilt_deg"] = math.degrees(-math.asin(float(np.clip(self._up[2], -1, 1))))  # + = tilted down
        out["hands"] = self._hands(fL, fR)
        return out

    @staticmethod
    def _normalise(v):
        return v / np.linalg.norm(v)

    # ---------------------------------------------------------------- draw
    def draw(self, fL, fR):
        pc = self._mp_pose.POSE_CONNECTIONS
        hc = self._mp_hands.HAND_CONNECTIONS
        for frame, which in ((fL, 0), (fR, 1)):
            h, w = frame.shape[:2]
            res = self._res.get("pose", (None, None))[which]
            if res is not None and res.pose_landmarks:
                pts = [(int(l.x * w), int(l.y * h), l.visibility) for l in res.pose_landmarks.landmark]
                for a, b in pc:
                    if pts[a][2] > 0.5 and pts[b][2] > 0.5:
                        cv2.line(frame, pts[a][:2], pts[b][:2], (255, 160, 0), 2, cv2.LINE_AA)
                for x, y, v in pts:
                    if v > 0.5:
                        cv2.circle(frame, (x, y), 3, (255, 255, 255), -1, cv2.LINE_AA)
            hres = self._res.get("hands", (None, None))[which]
            for lms in (hres.multi_hand_landmarks or []) if hres is not None else []:
                pts = [(int(l.x * w), int(l.y * h)) for l in lms.landmark]
                for a, b in hc:
                    cv2.line(frame, pts[a], pts[b], (0, 200, 255), 2, cv2.LINE_AA)

    def close(self):
        for m in (self._pose_l, self._pose_r, self._hands_l, self._hands_r):
            m.close()


FULL_IDS_SET = set(FULL_BODY_IDS)


def _fmt(m: float) -> str:
    return f"{m * 100:.0f} cm" if m < 1.0 else f"{m:.2f} m"


def annotate(frame, info: Dict) -> None:
    """Write the measurements from StereoTracker.update() onto the LEFT image."""
    lines = []
    if info["distance_m"] is not None:
        lines.append(f"Body distance: {_fmt(info['distance_m'])}  (depth {_fmt(info['depth_m'])})")
    else:
        lines.append("Body distance: -- (no body seen by both cameras)")
    lines.append(f"Height: {_fmt(info['height_m'])}" if info["height_m"] is not None
                 else "Height: -- (full body not in frame)")
    if info["cam_height_m"] is not None:
        lines.append(f"Camera height: {_fmt(info['cam_height_m'])}   tilt: {info['tilt_deg']:+.1f} deg (+ = down)")
    else:
        lines.append("Camera height: -- (needs full body in frame)")
    v = info["velocity"]
    if v is None:
        lines.append("Person speed: -- (no body)")
    elif v["speed"] == 0.0:
        lines.append("Person speed: 0.00 m/s (still)")
    else:
        vx, vy, vz = v["v"]
        lines.append(f"Person speed: {v['speed']:.2f} m/s  ({'away' if vz > 0 else 'toward'} {abs(vz):.2f}, "
                     f"cam-{'right' if vx > 0 else 'left'} {abs(vx):.2f}, {'down' if vy > 0 else 'up'} {abs(vy):.2f})")
    if info["reproj_px"] is not None:
        lines.append(f"Match quality: {info['reproj_px']:.2f} px reprojection error")
    h = frame.shape[0]
    items = [(text, (10, h - 10 - 20 * (len(lines) - 1 - i))) for i, text in enumerate(lines)]
    for hand in info["hands"]:
        wx, wy = hand["wrist_px"]
        items.append((f"{hand['label']} hand: {_fmt(hand['distance_m'])}", (max(wx - 60, 5), min(wy + 30, h - 5))))
    draw_texts(frame, items)


def run_track(args):
    rig = StereoRig.from_file(args.calib)
    capL = open_camera(args.cam_left, rig.size[0], rig.size[1])
    capR = open_camera(args.cam_right, rig.size[0], rig.size[1])
    tracker = StereoTracker(rig, args.pose_model, args.min_vis, args.max_reproj_px,
                            args.height_offset_cm / 100.0, args.floor_offset_cm / 100.0)
    log = None
    if args.log:
        log = open(args.log, "w", buffering=1)
        log.write("t,distance_m,depth_m,height_m,cam_height_m,tilt_deg,speed_mps,vx,vy,vz,reproj_px\n")
    print(f"Baseline from calibration: {rig.baseline_m * 100:.2f} cm. q / ESC quits, r resets.")

    try:
        while True:
            pair = grab_pair(capL, capR)
            if pair is None:
                print("Camera frame not received; exiting.")
                break
            fL, fR, t = pair
            if (fL.shape[1], fL.shape[0]) != rig.size:
                sys.exit(f"Camera returned {fL.shape[1]}x{fL.shape[0]} but calibration was "
                         f"done at {rig.size[0]}x{rig.size[1]}. Recalibrate or match the size.")
            info = tracker.update(fL, fR, t)
            tracker.draw(fL, fR)

            annotate(fL, info)
            v = info["velocity"]

            if log:
                vv = v["v"] if v else (float("nan"),) * 3
                log.write(",".join("" if x is None else f"{x:.4f}" for x in (
                    t, info["distance_m"], info["depth_m"], info["height_m"], info["cam_height_m"],
                    info["tilt_deg"], v["speed"] if v else None, vv[0], vv[1], vv[2], info["reproj_px"])) + "\n")

            both = np.hstack([fL, fR])
            cv2.imshow("Stereo Tracker  (left | right)", cv2.resize(both, None, fx=args.display_scale, fy=args.display_scale))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                tracker.reset()
    finally:
        tracker.close()
        capL.release()
        capR.release()
        cv2.destroyAllWindows()
        if log:
            log.close()


# ------------------------------------------------------------------ run stereo camera
def run_zed(args):
    rig = StereoRig.from_file(args.calib)
    w, h = rig.size  # e.g. 1280, 720

    # Cross-platform backend selection (DirectShow on Windows, V4L2 on Linux)
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_V4L2
    cap = cv2.VideoCapture(args.cam, backend)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.cam)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w * 2)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    tracker = StereoTracker(rig, args.pose_model, args.min_vis, args.max_reproj_px,
                            args.height_offset_cm / 100.0, args.floor_offset_cm / 100.0)
    print("Tracking with ZED. Press 'q' or ESC to quit.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            # Ensure frame is 3-channel BGR (handles raw YUYV if delivered by camera driver)
            if len(frame.shape) == 2 or (len(frame.shape) == 3 and frame.shape[2] == 2):
                frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)

            # Slice left and right halves
            fL = frame[:, :w]
            fR = frame[:, w:]
            t = time.time()

            info = tracker.update(fL, fR, t)
            tracker.draw(fL, fR)
            annotate(fL, info)

            both = np.hstack([fL, fR])
            cv2.imshow("ZED Stereo Tracker", cv2.resize(both, None, fx=args.display_scale, fy=args.display_scale))
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        tracker.close()
        cap.release()
        cv2.destroyAllWindows()


# ------------------------------------------------------------------ self-test
def run_selftest():
    """Camera-free check: synthetic cameras, known 3D points, noisy pixels."""
    f, B, W, H = 900.0, 0.20, 1280, 720
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])
    rig = StereoRig(K, np.zeros(5), K, np.zeros(5), np.eye(3), [-B, 0, 0], (W, H))
    rng = np.random.default_rng(0)
    cam_h, true_h, sigma = 1.0, 1.75, 0.5

    def project(P):
        q = K @ P
        return q[:2] / q[2]

    print(f"1) Triangulation: f={f:.0f}px, baseline={B * 100:.0f}cm, {sigma}px noise on every pixel,"
          f" person {true_h}m tall, camera {cam_h}m above the floor")
    print(f"{'range':>7} {'depth err (measured)':>22} {'depth err (theory)':>20} {'height err':>12}")
    for Z in (2.0, 3.0, 5.0, 8.0):
        heel = np.array([0, cam_h, Z])
        nose = np.array([0, cam_h - HEAD_FRAC * true_h, Z])
        dz, dh = [], []
        for _ in range(400):
            L = np.array([project(nose), project(heel)]) + rng.normal(0, sigma, (2, 2))
            R = np.array([project(nose + [-B, 0, 0]), project(heel + [-B, 0, 0])]) + rng.normal(0, sigma, (2, 2))
            X, _ = rig.triangulate(L, R)
            dz.append(X[1, 2] - Z)
            dh.append(np.linalg.norm(X[0] - X[1]) / HEAD_FRAC - true_h)
        theory = Z * Z * (sigma * math.sqrt(2)) / (f * B)
        print(f"{Z:6.1f}m {np.sqrt(np.mean(np.square(dz))) * 100:19.1f} cm {theory * 100:17.1f} cm "
              f"{np.sqrt(np.mean(np.square(dh))) * 100:9.1f} cm")
    print("   (real MediaPipe landmarks jitter by a few pixels, so expect larger errors than this.)\n")

    print("2) Calibration round-trip: 25 synthetic checkerboard views, 0.3px corner noise")
    cols, rows, sq = 9, 6, 0.025
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * sq
    th = math.radians(6)  # right camera is slightly toed-in relative to the left
    R_true = cv2.Rodrigues(np.array([0.0, th, 0.0]))[0]
    T_true = np.array([-0.12, 0.002, 0.003])
    obj, pl, pr = [], [], []
    while len(obj) < 25:
        rv = rng.uniform(-0.45, 0.45, 3)
        tv = np.array([rng.uniform(-0.15, 0.05), rng.uniform(-0.1, 0.05), rng.uniform(0.7, 1.4)])
        Rb = cv2.Rodrigues(rv)[0]
        a, _ = cv2.projectPoints(objp, rv, tv, K, None)
        Rr, tr = R_true @ Rb, R_true @ tv + T_true
        b, _ = cv2.projectPoints(objp, cv2.Rodrigues(Rr)[0], tr, K, None)
        a, b = a.reshape(-1, 2), b.reshape(-1, 2)
        if not (np.all((a > 20) & (a < [W - 20, H - 20])) and np.all((b > 20) & (b < [W - 20, H - 20]))):
            continue
        obj.append(objp)
        pl.append((a + rng.normal(0, 0.3, a.shape)).reshape(-1, 1, 2).astype(np.float32))
        pr.append((b + rng.normal(0, 0.3, b.shape)).reshape(-1, 1, 2).astype(np.float32))
    cal = compute_calibration(obj, pl, pr, (W, H))
    base = float(np.linalg.norm(cal["T"])) * 100
    print(f"   true baseline {np.linalg.norm(T_true) * 100:.2f} cm -> recovered {base:.2f} cm")
    print(f"   true focal {f:.0f}px -> recovered {cal['K1'][0, 0]:.1f}px (left), {cal['K2'][0, 0]:.1f}px (right)")
    print(f"   stereo RMS reprojection error: {cal['rms_stereo']:.3f} px")


# ---------------------------------------------------------------------- main
def _board(s: str):
    c, r = s.lower().split("x")
    return int(c), int(r)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Stereo person measurement.")
    sub = p.add_subparsers(dest="mode", required=True)

    def common(sp):
        sp.add_argument("--cam-left", type=int, default=0)
        sp.add_argument("--cam-right", type=int, default=1)
        sp.add_argument("--width", type=int, default=1280)
        sp.add_argument("--height", type=int, default=720)

    c = sub.add_parser("calibrate", help="calibrate the camera pair with a checkerboard")
    common(c)
    c.add_argument("--board", type=_board, default=(9, 6), help="INNER corners, e.g. 9x6")
    c.add_argument("--square-mm", type=float, default=25.0, help="printed square size, measured")
    c.add_argument("--min-pairs", type=int, default=20)
    c.add_argument("--out", default="stereo_calib.npz")

    t = sub.add_parser("track", help="live measurement")
    common(t)
    t.add_argument("--calib", default="stereo_calib.npz")
    t.add_argument("--pose-model", type=int, default=1, choices=(0, 1, 2),
                   help="pose accuracy: 0 fast, 1 balanced, 2 most accurate (slowest)")
    t.add_argument("--min-vis", type=float, default=0.6, help="min landmark visibility in BOTH views")
    t.add_argument("--max-reproj-px", type=float, default=3.0,
                   help="drop landmarks whose two views disagree by more than this")
    t.add_argument("--height-offset-cm", type=float, default=0.0,
                   help="added to measured height; set from a known-height person to cancel head-top bias")
    t.add_argument("--floor-offset-cm", type=float, default=0.0,
                   help="how far the heel landmark sits above the real floor (added to camera height)")
    t.add_argument("--display-scale", type=float, default=0.6)
    t.add_argument("--log", default=None, help="write every frame's measurements to this CSV")

    sub.add_parser("selftest", help="verify the maths with synthetic cameras (no hardware)")

    # FOR ZED STEREO CAMERA ------------------------------------
    z = sub.add_parser("zed", help="live measurement using a ZED camera")
    z.add_argument("--cam", type=int, default=0)
    z.add_argument("--calib", default="stereo_calib.npz")
    z.add_argument("--pose-model", type=int, default=1, choices=(0, 1, 2))
    z.add_argument("--min-vis", type=float, default=0.6)
    z.add_argument("--max-reproj-px", type=float, default=3.0)
    z.add_argument("--height-offset-cm", type=float, default=0.0)
    z.add_argument("--floor-offset-cm", type=float, default=0.0)
    z.add_argument("--display-scale", type=float, default=0.6)
    # ------------------------------------------------------------

    a = p.parse_args()
    {"calibrate": run_calibrate, "track": run_track, "zed": run_zed}.get(a.mode, lambda _a: run_selftest())(a)