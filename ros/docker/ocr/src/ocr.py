import argparse
import copy
import os

from PIL import Image
import numpy as np
import cv2

import pytesseract
from pytesseract import Output

import rospy
from sensor_msgs.msg import Image as img_msg

import utils as utils

from pdb import set_trace as bp


pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'
config = r'--psm 11 --oem 3 --tessdata-dir /app/ocr/data/tessdata'

class OCR(object):
    def __init__(self, args):
        self.debug = args.debug
        self.conf = args.conf
        self.annotated_img = None


    # Callback function to process the received image
    def image_callback(self, msg):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)[..., :3]
            img_preprocessed = self.preprocess(img)
            if img_preprocessed is not None:
                self.run_ocr(img, img_preprocessed)

        except Exception as e:
            rospy.logerr(e)
    
    def depth_callback(self, msg):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)[..., 3]
            cv2.imshow('depth', img)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(e)


    def read_img(self):
        rospy.init_node('zed_node', anonymous=True)
        rate = rospy.Rate(2)
        
        # Create a subscriber for the image topic
        rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', img_msg, self.image_callback)
        # Create a subscriber for the depth image topic
        
        # rospy.spin()
        while not rospy.is_shutdown():
            rate.sleep()

    
    def preprocess(self, img):
        # Apply red mask
        red_mask, _ = utils.red_mask(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
        red_mask = cv2.dilate(red_mask, np.ones((6, 6)))

        if self.debug:
            cv2.imshow('red mask', red_mask)
            cv2.waitKey(1)

        # Make the binary image from sat image (0 or 255)
        _, sat_img, _ = cv2.split((cv2.cvtColor(img, cv2.COLOR_BGR2HSV)))
        _,bin_img = cv2.threshold(sat_img,127,255,cv2.THRESH_BINARY)
        if self.debug:
            cv2.imshow('sat img', sat_img)
            cv2.waitKey(1)
        if self.debug:
            cv2.imshow('bin img',bin_img)
            cv2.waitKey(1)

        # Get octagon mask from sat image
        oct_mask = utils.get_oct(red_mask, img, self.debug)

        # If no octagon is detected, return False
        if oct_mask is False:
            return None
        
        oct_mask = cv2.erode(oct_mask, np.ones((10, 10)))
        if self.debug:
            cv2.imshow('octagon mask', oct_mask)
            cv2.waitKey(1)

        # Make 0 and 1 mask
        add_mask = np.array(np.logical_not(oct_mask) * 255, dtype=np.uint8)
        ocr_img = np.array(add_mask + bin_img)
        if self.debug:
            cv2.imshow('cropped binary mask', ocr_img)
            cv2.waitKey(1)
        

        # Apply dilation
        ocr_img = cv2.dilate(ocr_img, np.ones((2, 2)))
        if self.debug:
            cv2.imshow('fully preprocessed', ocr_img)
            cv2.waitKey(1)

        return ocr_img


    def run_ocr(self, img, img_preprocessed):
        # Run ocr
        data = pytesseract.image_to_data(Image.fromarray(img_preprocessed), output_type=Output.DICT, config=config)
        amount_boxes = len(data['text'])

        # Prepare annotated image
        annotated_img = copy.deepcopy(img)
        for i in range(amount_boxes):
            if float(data['conf'][i]) > self.conf:
                if self.debug:
                    print(f'Text recognized: {data["text"][i]} with conf {data["conf"][i]}')
                
                if data["text"][i].lower() == 'stop':
                    print('STOP recognized!')
                    (x, y, width, height) = (data['left'][i], data['top'][i], data['width'][i], data['height'][i])
                    annotated_img = cv2.rectangle(annotated_img, (x, y), (x + width, y + height), (0, 255, 0), 2)
                    annotated_img = cv2.putText(annotated_img, data['text'][i], (x, y + height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if data["text"][i].lower() == 'igvc' or data["text"][i].lower() == 'icvc' or data["text"][i].lower() == 'soup':
                    print('LOL you are trying to trick us!')
                    (x, y, width, height) = (data['left'][i], data['top'][i], data['width'][i], data['height'][i])
                    annotated_img = cv2.rectangle(annotated_img, (x, y), (x + width, y + height), (0, 255, 0), 2)
                    annotated_img = cv2.putText(annotated_img, data['text'][i], (x, y + height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        
        # cv2.imwrite('/app/ocr/data/out_annotated.png', self.annotated_img)
        
        cv2.namedWindow('annotated', cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty('annotate', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        cv2.imshow('annotated', annotated_img)
        cv2.waitKey(1)
    
    def clean(self):
        os.system(f'rm {os.getcwd()}/ocr/data/temp.png')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='OCR for CUber',
                                     description='Reads STOP sign.')
    
    parser.add_argument('--debug', action='store_true', help='When set to True, saves the images that are created during the preprocessing phase.')
    parser.add_argument('--src_fpath', type=str, default='/app/ocr/data/IMG_9185.jpg', help='Specify the file path to the input image. Defaults to \'/app/ocr/data/IMG_9185.jpg\'.')
    parser.add_argument('--conf', type=int, default=60, help='Confidence level threshold. An integer value between 0 and 100. Defaults to 60.')
    args = parser.parse_args()

    ocr = OCR(args)
    # ocr.preprocess()
    # ocr.run_ocr()
    ocr.read_img()
    # ocr.clean()
