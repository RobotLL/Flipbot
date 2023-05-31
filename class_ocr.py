# pip install opencv-python
# pip install pytesseract
# sudo apt update
# sudo apt install tesseract-ocr
# sudo apt install libtesseract-dev

import cv2
import pytesseract
import numpy as np
from PIL import Image, ImageOps
import matplotlib.pyplot as plt
from class_l515 import L515


class OCR:
    def __init__(self, x, y, h):
        # Set the path to the Tesseract OCR binary
        # Replace with the path to tesseract binary on your system
        pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'
        self.x = x
        self.y = y
        self.h = h

    def extract_page_number(self, image_array):
        # 设定码在图片中的位置
        image_array = image_array[self.y-self.h:self.y +
                                  self.h, self.x-self.h:self.x+self.h]

        # Convert the image to grayscale
        gray = cv2.cvtColor(image_array, cv2.COLOR_RGB2GRAY)
        # gray = cv2.resize(gray, (100, 100))

        alpha = 2  # Contrast control (1.0-3.0)
        beta = 20  # Brightness control (0-100)
        gray = cv2.convertScaleAbs(gray, alpha=alpha, beta=beta)

        text = pytesseract.image_to_string(
            gray, config='--psm 7 --oem 3 -c tessedit_char_whitelist=0123456789')
        return int(text)

    def roi(self, image_array):
        # 设定码在图片中的位置
        image_array = image_array[self.y-self.h:self.y +
                                  self.h, self.x-self.h:self.x+self.h]

        # Convert the image to grayscale
        gray = cv2.cvtColor(image_array, cv2.COLOR_RGB2GRAY)
        # gray = cv2.resize(gray, (100, 100))
        alpha = 2  # Contrast control (1.0-3.0)
        beta = 20  # Brightness control (0-100)
        gray = cv2.convertScaleAbs(gray, alpha=alpha, beta=beta)
        return gray


# %%
if __name__ == '__main__':
    ocr_right = OCR(x=1575, y=946, h=30)
    l515_right = L515(device_id='f1371789')
    # Replace with your own image array
    depth_image_right, color_image_right, verts_480_640_right, verts_1080p_right = l515_right.get_aligned_verts()
    plt.imshow(ocr_right.roi(color_image_right))
    # Extract page number
    text_right = ocr_right.extract_page_number(color_image_right)
    # Print the page number
    print("Page number_right:", text_right)
    l515_right.stop_streaming()
# %%
    ocr_left = OCR(x=1700, y=680, h=40)
    l515_left = L515(device_id='f1381233')

    # Replace with your own image array
    depth_image_left, color_image_left, verts_480_640_left, verts_1080p_left = l515_left.get_aligned_verts()
    plt.imshow(ocr_left.roi(color_image_left))
    # Extract page number
    text_left = ocr_left.extract_page_number(color_image_left)
    # Print the page number
    print("Page number_left:", text_left)
    l515_left.stop_streaming()
