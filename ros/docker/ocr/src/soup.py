from PIL import Image
import cv2
import numpy as np
import copy

import utils as utils

import pytesseract
from pytesseract import Output

pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'
config = r'--psm 11 --oem 3 --tessdata-dir /app/ocr/data/tessdata'

class SOUP(object):
    def __init__(self, fname):
        self.img = cv2.imread(f'/app/ocr/data/{fname}')
    

    def run_ocr(self):
        

        # Source image
        cv2.imshow('src img', self.img)

        # Apply HSV filter
        hsv, s, v = utils.hsv_filter(self.img)
        cv2.imshow('hsv img', hsv)

        # Apply red mask
        mask, img = utils.red_mask(hsv)
        cv2.imshow('red mask', mask)
        # cv2.imshow('mask applied', img)

        # Dilate and erode to get rid of noise
        mask = cv2.erode(mask, np.ones((3, 3)))
        mask = cv2.dilate(mask, np.ones((7, 7)))
        # mask = cv2.dilate(mask, np.ones((3, 3)))

        cv2.imshow('dilate & erode mask', mask)

        # Get octagon
        mask = utils.get_oct(mask, self.img, True)

        # Mask out
        result = cv2.bitwise_and(self.img.copy(), self.img.copy(), mask=mask)
        cv2.imshow('mask applied', result)

        # Run ocr
        data = pytesseract.image_to_data(Image.fromarray(result), output_type=Output.DICT, config=config)
        amount_boxes = len(data['text'])

        # Prepare annotated image
        annotated_img = copy.deepcopy(img)
        for i in range(amount_boxes):
            if float(data['conf'][i]) > 50:
                if self.debug:
                    print(f'Text recognized: {data["text"][i]} with conf {data["conf"][i]}')
                
                if data["text"][i].lower() == 'stop':
                    print('STOP recognized!')
                (x, y, width, height) = (data['left'][i], data['top'][i], data['width'][i], data['height'][i])
                annotated_img = cv2.rectangle(annotated_img, (x, y), (x + width, y + height), (0, 255, 0), 2)
                annotated_img = cv2.putText(annotated_img, data['text'][i], (x, y + height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        
        # cv2.imwrite('/app/ocr/data/out_annotated.png', self.annotated_img)
        cv2.imshow('annotated', annotated_img)

        cv2.waitKey(0)


if __name__ == '__main__':
    soup = SOUP('IGVC.png')
    soup.run_ocr()