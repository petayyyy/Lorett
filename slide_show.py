import cv2 
import os

def pictures_named(img, name, color_text = (0, 0, 0), color_background = (255, 255, 255), size_x = 1400, size_y = 800, put_in_topic = False, resize_pictures = True, background = False):
    x, y = 23*len(name), 35
    if resize_pictures: img = cv2.resize(img, (size_x, size_y))
    if background:  cv2.rectangle(img,(0,0),(x,y),color_background,-1)
    
    cv2.line(img, (0,y), (x,y), color_text, 2) # Line left to right
    cv2.line(img, (x,0), (x,y), color_text, 2) # Line up to down
    cv2.putText(img, name, (10, y-10), cv2.FONT_HERSHEY_TRIPLEX, 1, color_text)
    if put_in_topic:
        pass
    else:
        cv2.imshow('img', img)
        cv2.waitKey(0)
def search_images(path= "C:/Users/ilyah/Downloads/1111"):
    images, flag = [], False
    if os.path.exists(path):
        for file in os.listdir(path):
            print(file)
            if os.path.isdir(path+"/"+file):
                for pictures in os.listdir(path+"/"+file):
                    if ".png" in pictures or ".jpg" in pictures:
                        images.append(file+"/"+ str(pictures))
                        flag = True            
        if flag == False:
            print("yours path is empty")
        return images

def start_show(path = "C:/Users/ilyah/Downloads/1111"):
    images = search_images(path)
    for name in images:
        try:
            img = cv2.imread(path+"/"+name)
            print("Took picture from file {}".format(name))
            pictures_named(img, name)
        except Exception as e: print(e)
start_show()

