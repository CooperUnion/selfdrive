import cv2
import threading

class camThread(threading.Thread):
    def __init__(self, previewName, cam_label):
        threading.Thread.__init__(self)
        self.previewName = previewName
        self.cam_label = cam_label
    def run(self):
        print ("Starting " + self.previewName)
        camPreview(self.previewName, self.cam_label)

def camPreview(previewName, cam_label):
    cv2.namedWindow(previewName)
    cam = cv2.VideoCapture(cam_label)
    if cam.isOpened():  # try to get the first frame
        rval, frame = cam.read()
    else:
        rval = False
    while True:
        cam.set(3,640)
        cam.set(4,480)
        frame = cam.read()
        if(frame[0]):
            cv2.imshow(previewName, frame[1])
        key = cv2.waitKey(20)
        if key == 27:  # exit on ESC
            break


        # rval, frame = cam.read()
        # if(rval): 
        #     cv2.imshow(previewName, frame)
        # key = cv2.waitKey(20)
        # cam.release()

    cv2.destroyWindow(previewName)

# Create two threads as follows
thread1 = camThread("Camera 1", "/dev/video1")
thread2 = camThread("Camera 2", "/dev/video3")
thread1.start()
thread2.start()