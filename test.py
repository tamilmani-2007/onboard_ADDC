from pupil_apriltags import Detector
import cv2 as cv
from cam.capture import open_capture
from utils.config import VIDEO_SOURCE, VIDEO_SOURCE_PATH, CAMERA_WIDTH, CAMERA_HEIGHT

_apriltag_detector = Detector(
    families="tag25h9",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

cap = open_capture(VIDEO_SOURCE, VIDEO_SOURCE_PATH, CAMERA_WIDTH, CAMERA_HEIGHT)

if cap is None or not cap.isOpened():
    print("Can't open camera")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Can't read frame")
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    results = _apriltag_detector.detect(gray)

    # Guard: only process if a tag was actually found
    if results:
        result = results[0]

        # corners is a 4x2 numpy array: [LT, RT, RB, LB]
        start_point = tuple(result.corners[0].astype(int))
        end_point   = tuple(result.corners[3].astype(int))

        color = (0, 255, 0)
        thickness = 5

        cv.line(frame, start_point, end_point, color, thickness)

        print("Data:", result.tag_id)
    cv.imshow("pupil_apriltag", frame)

    if cv.waitKey(1) & 0xFF == ord("q"):
        print("closing")
        break

cap.release()
cv.destroyAllWindows()   