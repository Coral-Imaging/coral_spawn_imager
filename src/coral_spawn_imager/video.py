#! /usr/bin/python3

import socket
import time
from picamera2 import Picamera2, Preview
from picamera2.encoders import JpegEncoder
import datetime
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput

picam2 = Picamera2()
# set max resolution, fps is then limited to 40fps, should still be fast enough
video_config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(video_config)
encoder = H264Encoder(1000000)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("192.168.1.118", 10001))
    sock.listen()

    picam2.encoders = encoder

    conn, addr = sock.accept()
    stream = conn.makefile("wb")
    encoder.output = FileOutput(stream)
    picam2.start_encoder()
    picam2.start()
    time.sleep(60) # goes for 20 seconds?
    picam2.stop()
    picam2.stop_encoder()
    conn.close()
    
# captures a video
# picam2.start_preview(Preview.QT)
# # picam2.start_preview(Preview.QTGL)

# encoder = JpegEncoder(q=100)

# datestr = datetime.datetime.now()
# movie_name = datestr.strftime("%Y%m%d_%H%M%S_%f") + '_mov'


# picam2.start_recording(encoder, movie_name + '.mjpeg', pts=movie_name + '_timestamp.txt')

# video_duration = 10 # seconds
# time.sleep(video_duration)

# picam2.stop_recording()
