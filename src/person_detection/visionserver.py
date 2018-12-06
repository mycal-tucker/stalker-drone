import cv2
cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
gst = "rtspsrc location=rtsp://192.168.99.1/media/stream2 latency=10 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"
cap = cv2.VideoCapture(gst)
while(cap.isOpened()):
    ret, frame = cap.read()
    # you can add your processing here on the frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()