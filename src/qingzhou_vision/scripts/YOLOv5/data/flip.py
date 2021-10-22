import cv2 as cv
import os
from string import Template

img_temp = Template("./images/$num.jpg")
txt_temp = Template("./labels/$num.txt")


def get_nums():
    nums = []
    for root, dir, files in os.walk('./images'):
        for file in files:
            num = int(file[:-4])
            nums += [num]
    return nums


def gen_newnums(nums):
    return [num+1000 for num in nums]


nums = get_nums()
new_nums = gen_newnums(nums)

# 0 右转直行
# 1 左转
# 2 直行
# 3 右转
# 4 左转直行


def make_flip_img(num, newnum):
    imgpath = img_temp.substitute(num=num)
    img = cv.imread(imgpath)
    img = cv.flip(img, 1)
    newimgpath = img_temp.substitute(num=newnum)
    cv.imwrite(newimgpath, img)


def make_flip_line(line):
    label, x, y, w, h = line.strip().split(' ')
    x = str(1-float(x))
    if label == '0':
        label = '4'
        return ' '.join([label, x, y, w, h])
    elif label == '1':
        label = '3'
        return ' '.join([label, x, y, w, h])
    else:
        return line.strip()


def make_flip_label(num, newnum):
    txtpath = txt_temp.substitute(num=num)
    newtxtpath = txt_temp.substitute(num=newnum)
    with open(txtpath, 'r') as origin, open(newtxtpath, 'w') as flip:
        for line in origin:
            newline = make_flip_line(line)
            flip.write(newline + '\n')


for num in nums:
    make_flip_img(num, num+1000)
    make_flip_label(num, num+1000)
