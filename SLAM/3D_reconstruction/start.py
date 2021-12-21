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
    # for index, i in enumerate(range(0, 3000000, 80000)):
    for i in range(1000):
        isTrue, frame = capt.read()

        if i % 30 == 0:
            
            kp = orb.detect(frame, None)
            kp, des = orb.compute(frame, kp)
            img2 = cv.drawKeypoints(frame, kp, None, color=(0,255,0), flags=0)
            img = Image(count, kp, des)
            graph.add_frame(img)
            #imgs[i] = [kp, des]
            cv.imwrite('/Users/kobikelemen/Documents/programming/python stuff/open_test/frames/f' + str(count) + '.jpg', img2)
            count += 1
    capt.release()
    return imgs, count
    # 1. use igraph library to make un associated graph corresponding to image frames
    # 2. use key points detected with orb to first brute force match which images overlap
    # 3. update graph edges to reflect this
    # 4. continue with YT lecture -> RANSAC and bag of worlds


def image_correspondances(scene_graph, imgs, no_images=25):
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

    #for image in range(no_images):
    #    imgs.append(cv.imread('/Users/kobikelemen/Documents/programming/python stuff/open_test/frames/f'+str(image)+'.jpg'))

    imgs = scene_graph.get_frames()
    for img in imgs:
        for check_img in imgs:
            if img != check_img:
                # if check_img.desc.any() == img.desc.any():
                matches = bf.match(check_img.desc, img.desc)
                matches = sorted(matches, key=lambda x: x.distance)
                print('checking frame '+ str(img.frame_no) + ' with frame ' + str(check_img.frame_no))
                print('len(matches): ', len(matches))
                print('smallest distance: ', matches[0].distance)

                # good = []
                # for n,m in matches:
                #     if m.distance < 0.75 * n.distance:
                #         scene_graph.add_connection(img, check_img)
                if matches[0].distance < 20:
                    scene_graph.add_connection(img, check_img)
                    
    scene_graph.print_graph()


    # img_test = cv.drawMatchesBF(scene_graph.frames[0] )





if __name__ == '__main__':
    scene_graph = Scene_graph(path_='/Users/kobikelemen/Documents/programming/python stuff/open_test/frames')
    imgs, count = save_frames(scene_graph)
    image_correspondances(scene_graph, imgs, count)