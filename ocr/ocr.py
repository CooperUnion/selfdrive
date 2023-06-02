from PIL import Image
import pytesseract
import cv2
from pytesseract import Output
import numpy as np
import argparse
import copy
import os

import utils as utils

pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'
config = r'--psm 11 --oem 3 --tessdata-dir /app/ocr/data/tessdata'

class OCR(object):
    def __init__(self, args):
        self.debug = args.debug
        self.conf = args.conf
        self.src_fpath = args.src_fpath
        self.target_fpath = '/app/ocr/data/preprocessed.png'
        self.annotated_img = None
    
    def preprocess(self):
        # Convert the image to png
        utils.to_png(self.src_fpath, '/app/ocr/data/temp.png')
        true_img = cv2.imread('/app/ocr/data/temp.png')
        self.annotated_img = copy.deepcopy(true_img)

        # Get hsv, sat, and grayscale image
        hsv_img, sat_img, gray_img = utils.hsv_filter(true_img)
        if self.debug:
            cv2.imwrite('/app/ocr/data/hsv.png', hsv_img)
            cv2.imwrite('/app/ocr/data/sat.png', sat_img)
            cv2.imwrite('/app/ocr/data/grayhsv.png', gray_img)

        # Make the binary image from sat image (0 or 255)
        _,bin_img = cv2.threshold(sat_img,127,255,cv2.THRESH_BINARY)
        if self.debug:
            cv2.imwrite('/app/ocr/data/bin_img.png',bin_img)

        # Get octagon mask from sat image
        oct_mask = utils.get_oct(sat_img, self.debug)
        if self.debug:
            cv2.imwrite('/app/ocr/data/oct.png', oct_mask)


        # Make 0 and 1 mask
        add_mask = np.array(np.logical_not(oct_mask), dtype=np.int8) * 255
        ocr_img = add_mask + bin_img
        if self.debug:
            cv2.imwrite('/app/ocr/data/img2.png',add_mask)
            cv2.imwrite('/app/ocr/data/img3.png', ocr_img)
        

        # Apply dilation
        dilated =utils.dilate(ocr_img)

        # Save the preprocessed image
        cv2.imwrite(self.target_fpath, dilated)


    def run(self):
        # Run ocr
        data = pytesseract.image_to_data(Image.open(self.target_fpath), output_type=Output.DICT, config=config)
        amount_boxes = len(data['text'])
        for i in range(amount_boxes):
            if float(data['conf'][i]) > self.conf:
                print(f'Text recognized: {data["text"][i]} with conf {data["conf"][i]}')
                (x, y, width, height) = (data['left'][i], data['top'][i], data['width'][i], data['height'][i])
                self.annotated_img = cv2.rectangle(self.annotated_img, (x, y), (x + width, y + height), (0, 255, 0), 10)
                self.annotated_img = cv2.putText(self.annotated_img, data['text'][i], (x, y + height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        
        cv2.imwrite('/app/ocr/data/out_annotated.png', self.annotated_img)
    
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
    ocr.preprocess()
    ocr.run()
    ocr.clean()
