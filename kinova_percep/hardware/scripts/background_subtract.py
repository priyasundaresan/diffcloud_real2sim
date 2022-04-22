import cv2
import os
import numpy as np

def mask_robot(img, background_img):
    diff = cv2.absdiff(img,background_img)
    #cv2.imshow('vis', diff)
    #cv2.waitKey(0)
    lower = np.array([0,0,0])
    upper = np.array([55,55,55])
    #upper = np.array([80,80,80])
    mask = cv2.inRange(diff, lower, upper)
    mask = cv2.bitwise_not(mask)
    diff = cv2.bitwise_and(diff,diff,mask=mask)
    _, diff = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY)
    return diff
    #cv2.imshow('vis', diff)
    #cv2.waitKey(0)


if __name__ == '__main__':
    #video_dir = os.path.join('video')
    video_dir = os.path.join('run8')
    background_img = cv2.imread('background.jpg')
    output_dir = 'robot_masks'
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    for f in os.listdir(video_dir):
        print(os.path.join(video_dir, f))
        img = cv2.imread(os.path.join(video_dir, f))
        mask = mask_robot(img, background_img)
        cv2.imwrite(os.path.join(output_dir, f), mask)




