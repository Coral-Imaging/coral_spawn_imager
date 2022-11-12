#! /usr/bin/python3

import time
from picamera2 import Picamera2, Preview
from picamera2.encoders import JpegEncoder
import datetime

picam2 = Picamera2()
# set max resolution, fps is then limited to 40fps, should still be fast enough
video_config = picam2.create_video_configuration(main={"size": (2028, 1520)})
picam2.configure(video_config)

picam2.start_preview(Preview.QT)
# picam2.start_preview(Preview.QTGL)

encoder = JpegEncoder(q=100)

datestr = datetime.datetime.now()
movie_name = datestr.strftime("%Y%m%d_%H%M%S_%f") + '_mov'


picam2.start_recording(encoder, movie_name + '.mjpeg', pts=movie_name + '_timestamp.txt')

video_duration = 10 # seconds
time.sleep(video_duration)

picam2.stop_recording()
