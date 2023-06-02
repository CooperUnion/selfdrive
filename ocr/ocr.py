from PIL import Image
import pytesseract
import cv2
from pytesseract import Output

import utils as utils

pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'
config = r'--psm 11 --oem 3 --tessdata-dir /app/ocr/data/tessdata'

src_fpath = '/app/ocr/data/IMG_9184.jpg'
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
cv2.imwrite('/app/ocr/data/grayhsv.png', gray_img)

#img = utils.gray_binary(gray_img)
#cv2.imwrite('/app/ocr/data/binary.png', img)

#Apply gaussian blur
gauss_blur_img = utils.gauss_blur(sat_img)
_, gauss_blur_binary_img =cv2.threshold(gauss_blur_img, 90, 255, cv2.THRESH_BINARY)
cv2.imwrite('/app/ocr/data/blur.png', gauss_blur_binary_img)

mask = utils.get_oct(true_img, gauss_blur_binary_img)
cv2.imwrite('/app/ocr/data/oct.png', mask)
# img = cv2.bitwise_and(img, mask, img)
# cv2.imwrite('/app/ocr/data/out_mask.png', img)

# img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
# cv2.imwrite(target_fpath, img)

# # Annotate boxes
# data = pytesseract.image_to_data(Image.open(target_fpath), output_type=Output.DICT, config=config)
# amount_boxes = len(data['text'])
# for i in range(amount_boxes):
#     if float(data['conf'][i]) > 70:
#         print(f'Text recognized: {data["text"][i]} with conf {data["conf"][i]}')
#         (x, y, width, height) = (data['left'][i], data['top'][i], data['width'][i], data['height'][i])
#         img = cv2.rectangle(img, (x, y), (x + width, y + height), (0, 255, 0), 10)
#         img = cv2.putText(img, data['text'][i], (x, y + height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

# cv2.imwrite('/app/ocr/data/out_annotated.png', img)
