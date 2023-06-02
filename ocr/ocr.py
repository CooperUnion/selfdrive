from PIL import Image
import pytesseract
import cv2
from pytesseract import Output
import numpy as np

import utils as utils

pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'
config = r'--psm 11 --oem 3 --tessdata-dir /app/ocr/data/tessdata'

src_fpath = '/app/ocr/data/IMG_9185.jpg'
target_fpath = '/app/ocr/data/img.png'

# Convert the image to png
utils.to_png(src_fpath, target_fpath)

# Read image
true_img = cv2.imread(target_fpath) # np.ndarray

# #increase contrast
# img = utils.inc_cont(img)
# cv2.imwrite('/app/ocr/data/contrastup.png', img)

# Get hsv, sat, and grayscale image
hsv_img, sat_img, gray_img = utils.hsv_filter(true_img)
cv2.imwrite('/app/ocr/data/hsv.png', hsv_img)
cv2.imwrite('/app/ocr/data/sat.png', sat_img)
#cv2.imwrite('/app/ocr/data/grayhsv.png', gray_img)


#Get octagon mask
gauss_blur_img = utils.gauss_blur(sat_img)
_, gauss_blur_binary_img =cv2.threshold(gauss_blur_img, 90, 255, cv2.THRESH_BINARY)
cv2.imwrite('/app/ocr/data/blur.png', gauss_blur_binary_img)

oct_mask = utils.get_oct(true_img, gauss_blur_binary_img)
cv2.imwrite('/app/ocr/data/oct.png', oct_mask)

#mask out only octagon

oct_mask = oct_mask[:,:,0]
#cv2.imwrite('/app/ocr/data/oct.png', oct_mask)
oct_sat = cv2.bitwise_and(sat_img, oct_mask)
cv2.imwrite('/app/ocr/data/octsat.png', oct_sat)

# #get red mask 
# red_mask = utils.get_red(hsv_img)
# cv2.imwrite('app/ocr/data/redmask.png',red_mask)


# img = cv2.bitwise_and(img, mask, img)
# cv2.imwrite('/app/ocr/data/out_mask.png', img)

# img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
# cv2.imwrite(target_fpath, img)

# # Annotate boxescv2.imwrite('/app/ocr/data/hsv2.png', hsv_img)
_,octsatbin = cv2.threshold(oct_sat,127,255,cv2.THRESH_BINARY)
cv2.imwrite('/app/ocr/data/octsatbin.png',octsatbin)

#make 0 and 1 mask
add_mask = np.array(np.logical_not(oct_mask), dtype=np.int8) * 255
ocr_img = add_mask + octsatbin
cv2.imwrite('/app/ocr/data/img2.png',add_mask)
cv2.imwrite('/app/ocr/data/img3.png', ocr_img)

erode =utils.erosion(ocr_img)
cv2.imwrite('/app/ocr/data/erode.png', erode)

target_fpath = '/app/ocr/data/erode.png'
data = pytesseract.image_to_data(Image.open(target_fpath), output_type=Output.DICT, config=config)
amount_boxes = len(data['text'])
for i in range(amount_boxes):
    if float(data['conf'][i]) > 10:
        print(f'Text recognized: {data["text"][i]} with conf {data["conf"][i]}')
        (x, y, width, height) = (data['left'][i], data['top'][i], data['width'][i], data['height'][i])
        sat_img = cv2.rectangle(sat_img, (x, y), (x + width, y + height), (0, 255, 0), 10)
        sat_img = cv2.putText(sat_img, data['text'][i], (x, y + height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

cv2.imwrite('/app/ocr/data/out_annotated.png', sat_img)
