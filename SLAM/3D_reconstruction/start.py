import cv2 as cv
import numpy as np
#from igraph import *
from scene_graph import Scene_graph, Image

def hi():
    capture = cv.VideoCapture('house_tour.mp4')
    orb = cv.ORB_create()
    while True:
        isTrue, frame = capture.read()
        kp = orb.detect(frame, None)
        kp, des = orb.compute(frame, kp)
        img2 = cv.drawKeypoints(frame, kp, None, color=(0,255,0), flags=0)
        cv.imshow('Video', img2)
        if cv.waitKey(20) & 0xFF == ord('d'):
            break
    capture.release()
    cv.destroyAllWindows()



def save_frames(graph):
    capt = cv.VideoCapture('house_tour.mp4')
    orb = cv.ORB_create()
    imgs = {}
    count = 0
    for i in range(0, 100000, 4000):
        isTrue, frame = capt.read()
        kp = orb.detect(frame, None)
        kp, des = orb.compute(frame, kp)
        img2 = cv.drawKeypoints(frame, kp, None, color=(0,255,0), flags=0)
        img = Image(i, kp, des)
        graph.add_frame(img)
        #imgs[i] = [kp, des]
        cv.imwrite('/Users/kobikwlemen/Documents/programming/python stuff/open_test/frames/f' + str(i) + '.jpg', img2)
        count += 1
    capt.release()
    return imgs, count
    # 1. use igraph library to make un associated graph corresponding to image frames
    # 2. use key points detected with orb to first brute force match which images overlap
    # 3. update graph edges to reflect this
    # 4. continue with YT lecture -> RANSAC and bag of worlds

def image_correspondances(graph, imgs, no_images=25):
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

    #for image in range(no_images):
    #    imgs.append(cv.imread('/Users/kobikelemen/Documents/programming/python stuff/open_test/frames/f'+str(image)+'.jpg'))

    imgs = graph.get_frames()
    for img in imgs:
        
        for check_img in imgs:
            # print(imgs[img][1], 'yooo')
            # print(imgs[check_img][1], 'yooo')
            if imgs[check_img][1].any() == imgs[img][1].any():
                matches = bf.match(imgs[check_img][1], imgs[img][1])
                matches = sorted(matches, key=lambda x: x.distance)
                if len(matches) > 0:
                    #print('hi')
                    print('pair: ', img, check_img)




if __name__ == '__main__':
    scene_graph = Scene_graph(path_='/Users/kobikelemen/Documents/programming/python stuff/open_test/frames')
    imgs, count = save_frames(scene_graph)
    image_correspondances(scene_graph, imgs, count)