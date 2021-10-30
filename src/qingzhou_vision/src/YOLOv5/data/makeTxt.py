import os
import random


trainval_percent = 0.9
train_percent = 0.9

img_list = []

# 遍历images中所有图片,构造图片路径加入img_list
for root, dirs, files in os.walk('./images'):
    for file in files:
        img_list.append('data/images/' + file)


# 将其分为8:1:1的train,validate,test数据集
num = len(img_list)
list = range(num)
tv = int(num * trainval_percent)
tr = int(tv * train_percent)

trainval = random.sample(list, tv)
train = random.sample(trainval, tr)

# 分别写入train.txt,val.txt,test.txt中以便后续训练使用
with open('train.txt', 'w') as trainlist, \
        open('val.txt', 'w') as vallist, \
        open('test.txt', 'w') as testlist:
    for i in list:
        imgpath = img_list[i] + '\n'

        if i in trainval:
            if i in train:
                trainlist.write(imgpath)
            else:
                vallist.write(imgpath)
        else:
            testlist.write(imgpath)
