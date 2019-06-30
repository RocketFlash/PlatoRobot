import time

import cv2
import numpy as np

from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import get_graph_path, model_wh


fps_time = 0


if __name__ == '__main__':

    # possible models: cmu / mobilenet_thin / mobilenet_v2_large / mobilenet_v2_small
    model = 'mobilenet_thin'
    resized = "432x368"

    w, h = model_wh(resized)
    if w > 0 and h > 0:
        e = TfPoseEstimator(get_graph_path(model), target_size=(w, h))
    else:
        e = TfPoseEstimator(get_graph_path(model), target_size=(432, 368))

    cam = cv2.VideoCapture(0)
    ret_val, image = cam.read()

    while True:
        ret_val, image = cam.read()

        humans = e.inference(image, resize_to_default=(
            w > 0 and h > 0), upsample_size=1)

        image = TfPoseEstimator.draw_humans(image, humans, imgcopy=False)

        cv2.putText(image,
                    "FPS: %f" % (1.0 / (time.time() - fps_time)),
                    (10, 10),  cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2)
        cv2.imshow('tf-pose-estimation result', image)
        fps_time = time.time()
        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
