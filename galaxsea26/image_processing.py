import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import rclpy
from rclpy.node import Node
import os
import time
import urllib.request
from collections import defaultdict, deque



model_path = "/root/roboboat_ws/src/galaxsea26/model.pt"

if not os.path.exists(model_path):
    print(f"Model not found; downloading {MODEL_URL}")
    urllib.request.urlretrieve(MODEL_URL, model_path)

print("Loading model...")
model = YOLO(model_path)
print("Model loaded")

# Constants
DISPLAY_RESOLUTION = (1280, 720)
MODEL_INPUT_DIMENSIONS = (1080, 1080)
DISPLAY_WIDTH, DISPLAY_HEIGHT = DISPLAY_RESOLUTION
MODEL_WIDTH, MODEL_HEIGHT = MODEL_INPUT_DIMENSIONS
X_SCALE_FACTOR = DISPLAY_WIDTH / MODEL_WIDTH
Y_SCALE_FACTOR = DISPLAY_HEIGHT / MODEL_HEIGHT
FRAME_AREA = MODEL_WIDTH * MODEL_HEIGHT

# Pre-made overlay for on-screen display
print("Creating overlay...")
overlay = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
cv2.putText(
    overlay, "Press k to pause", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
)
cv2.putText(
    overlay,
    "Press ESC to exit",
    (10, 75),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.5,
    (0, 255, 0),
    1,
)
cv2.putText(
    overlay,
    "Press r to restart (video cap only)",
    (10, 100),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.5,
    (0, 255, 0),
    1,
)
print("Overlay created")

colors: Dict[str, rgb] = {
    "blue_buoy": rgb(33, 49, 255),
    "dock": rgb(132, 66, 0),
    "green_buoy": rgb(135, 255, 0),
    "green_pole_buoy": rgb(0, 255, 163),
    "misc_buoy": rgb(255, 0, 161),
    "red_buoy": rgb(255, 0, 0),
    "red_pole_buoy": rgb(255, 92, 0),
    "yellow_buoy": rgb(255, 255, 0),
    "black_buoy": rgb(0, 0, 0),
    "red_racquet_ball": rgb(255, 153, 0),
    "yellow_racquet_ball": rgb(204, 255, 0),
    "blue_racquet_ball": rgb(102, 20, 219),
}


class CameraSubscriber(Node):
    def __init__(self):
        print("Initializing node")
        super().__init__("camera_subscriber")

        self.declare_parameter("headless_mode", True)
        self.headless_mode = self.get_parameter("headless_mode").value

        self.start_time = time.perf_counter()
        self.display_time = 1.0
        self.fc = 0
        self.FPS = 0
        self.total_frames = 0
        self.processing_times = []
        self.last_callback_time = time.perf_counter()

        self.cvbridge = CvBridge()
        # Use deque with fixed maxlen for efficient tracking history updates
        self.track_history = defaultdict(lambda: deque(maxlen=30))

        self.create_subscription(Image, "/wamv/sensors/cameras/front_left_camera_sensor/image_raw", self.safe_image_callback, 10)
        self.publisher = self.create_publisher(Image, "/img/labeled", 10)
        print("Node initialized")
    
    def safe_image_callback(self, data: Image):
        try:
            self.image_callback(data)
        except Exception as e:
            print(f"Error in callback: {e}")

    def image_callback(self, data: Image):
        self.get_logger().info("Processing new frame")
        self.get_logger().info(f"Frame rate: {self.FPS}")

        # ------------------------------------------------------------
        # 1. Convert ROS Image → OpenCV BGR image
        # ------------------------------------------------------------
        raw_frame = self.cvbridge.imgmsg_to_cv2(data, "bgr8")

        # ------------------------------------------------------------
        # 2. Resize to display resolution (1280×720)
        # This is the frame used for on-screen preview and annotation.
        # ------------------------------------------------------------
        display_frame = cv2.resize(raw_frame, DISPLAY_RESOLUTION)

        # This will become the annotated output frame
        original_frame = display_frame.copy()

        # ------------------------------------------------------------
        # 3. Resize to YOLO model input size (1080×1080)
        # YOLO requires a square image for best accuracy.
        # ------------------------------------------------------------
        model_frame = cv2.resize(display_frame, MODEL_INPUT_DIMENSIONS)

        # ------------------------------------------------------------
        # 4. Draw FPS on the frame
        # ------------------------------------------------------------
        fps_text = f"FPS: {self.FPS:.2f}"
        cv2.putText(
            original_frame,
            fps_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

        # ------------------------------------------------------------
        # 5. Run YOLO tracking on the model-sized frame
        # ------------------------------------------------------------
        results = model.track(model_frame, persist=True, tracker="bytetrack.yaml")

        # ------------------------------------------------------------
        # 6. Create an Annotator object to draw labels/bounding boxes
        # ------------------------------------------------------------
        annotator = Annotator(original_frame, line_width=1)

        # ------------------------------------------------------------
        # 7. Draw detections
        # ------------------------------------------------------------
        for pred in results:
            if pred.boxes is None:
                continue

            names = pred.names

            for i in range(len(pred.boxes)):
                cls_id = int(pred.boxes.cls[i])
                name = names.get(cls_id, "Unknown")
                confidence = pred.boxes.conf[i]

                # Scaled bbox from model size → display size
                x1, y1, x2, y2 = pred.boxes[i].xyxy[0]
                x1 *= X_SCALE_FACTOR
                y1 *= Y_SCALE_FACTOR
                x2 *= X_SCALE_FACTOR
                y2 *= Y_SCALE_FACTOR

                # Draw labeled bounding box
                label = f"{name} {int(confidence * 100)}%"
                annotator.box_label((int(x1), int(y1), int(x2), int(y2)), label)

        # ------------------------------------------------------------
        # 8. Get final annotated frame
        # ------------------------------------------------------------
        annotated_frame = annotator.result()

        # ------------------------------------------------------------
        # 9. Convert annotated frame → ROS message + publish
        # ------------------------------------------------------------
        img_msg = self.cvbridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    cam_sub = CameraSubscriber()
    try:
        rclpy.spin(cam_sub)
    except KeyboardInterrupt:
        pass
    finally:
        cam_sub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()