#! /usr/bin/env python3
import cv2
import csv
import numpy as np
import os
import pyqrcode
import random
import string
import matplotlib.pyplot as plt
from numpy.linalg import inv

from random import randint
from PIL import Image, ImageFont, ImageDraw

# Taken from https://github.com/ENPH353/2022_competition/blob/main/enph353/enph353_gazebo/scripts/plate_generator.py 
# to ensure accurate liscence plates
path = os.path.dirname(os.path.realpath(__file__)) + "/"

with open(path + "plates.csv", 'w') as plates_file:
    csvwriter = csv.writer(plates_file) # creates a writer object
    # this object converts the users data into delimited strings
    # on the given file object
    print("here")
    for i in range(0, 16):

        # Pick two random letters
        plate_alpha = ""
        for _ in range(0, 2):
            plate_alpha += (random.choice(string.ascii_uppercase))
        num = randint(0, 99)

        # Pick two random numbers
        plate_num = "{:02d}".format(num)

        # Save plate to file
        csvwriter.writerow([plate_alpha+plate_num]) #

        # Write plate to image
        blank_plate = cv2.imread(path+'blank_plate.png')
        file_name1 = os.path.join(os.path.dirname(__file__), 'p_image.jpg')
        assert os.path.exists(file_name1)
        img1 = cv2.imread(file_name1, 0)          # queryImage
        plt.imshow(img1, 'gray'),plt.show()

        # To use monospaced font for the license plate we need to use the PIL
        # package.
        # Convert into a PIL image (this is so we can use the monospaced fonts)
        blank_plate_pil = Image.fromarray(blank_plate)
        # Get a drawing context
        draw = ImageDraw.Draw(blank_plate_pil)
        monospace = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf", 200)
        draw.text((48, 105),plate_alpha + " " + plate_num, (255,0,0), font=monospace)
        # Convert back to OpenCV image and save
        blank_plate = np.array(blank_plate_pil)

        # cv2.putText(blank_plate,
        #             plate_alpha + " " + plate_num, (45, 360),
        #             cv2.FONT_HERSHEY_PLAIN, 11, (255, 0, 0), 7, cv2.LINE_AA)

        # Create parking spot label
        s = "P" + str(i+1)
        parking_spot = 255 * np.ones(shape=[600, 600, 3], dtype=np.uint8)
        cv2.putText(parking_spot, s, (30, 450), cv2.FONT_HERSHEY_PLAIN, 28,
                    (0, 0, 0), 30, cv2.LINE_AA)
        
        # Creates parking spot picture
        img2 = gray = cv2.cvtColor(np.concatenate((parking_spot, blank_plate), axis=0, ), cv2.COLOR_BGR2GRAY)

        # Creates long parking spot picture
        # img2 = np.concatenate((255 * np.ones(shape=[600, 600, 3],
        #                             dtype=np.uint8), spot_w_plate), axis=0)

        cv2.imwrite(os.path.join(path+"unlabelled/",
                                 s+"_" + plate_alpha+plate_num + ".png"), img2)
        plt.imshow(img2, 'gray'),plt.show()

        # Want to perspective transform but currently doesnt work
        # # Locate points of the documents
        # # or object which you want to transform
        # # Input image, points taken from plt.imshow
        # pts1 = np.float32([[95, 153], [237, 189],
        #                 # [127, 676], [478, 663],
        #                 [127, 900], [478, 913]])
        # # Output image, i.e destination
        # pts2 = np.float32([[293, 383], [485, 394],
        #                 # [331, 636], [722, 636],
        #                 [331, 756], [722, 755]])
        
        # # Apply Perspective Transform Algorithm
        # matrix = cv2.getPerspectiveTransform(pts1, pts2)
        # result = cv2.warpPerspective(img2, matrix, (500, 600))
        
        # # Wrap the transformed image
        # cv2.imshow('frame', img2) # Initial Capture
        # cv2.imshow('frame1', result) # Transformed Capture
        # plt.imshow(result, 'gray'),plt.show()