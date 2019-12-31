import cv2
import time
from firebase import firebase
firebase = firebase.FirebaseApplication("https://secudell.firebaseio.com/", None)

video = cv2.VideoCapture("footage.avi")
first_frame = None
frames = 1
in_start = 0
thersh = 70
start = time.time()
while(True):
    check, frame = video.read()
    if not check:
        break
    frames = frames + 1

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rotated = cv2.GaussianBlur(gray, (21,21), 0)
    #frame = cv2.resize(frame, (500, 350))
    #rotated = cv2.rotate(gray, cv2.ROTATE_90_CLOCKWISE)
    #time.sleep(0.02)
    if first_frame is None:
        first_frame = rotated
        #cv2.imshow("first",first_frame)
        continue
    delta_frame = cv2.absdiff(first_frame, rotated)
    thresh_delta = cv2.threshold(delta_frame, thersh, 255, cv2.THRESH_BINARY)[1]
    thresh_delta = cv2.dilate(thresh_delta, None, iterations = 0)
    (cnts,_) = cv2.findContours(thresh_delta.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in cnts:
        if cv2.contourArea(contour) < 1000:
            continue
        if in_start == 0 :
            in_start = time.ctime(int(time.time()))  #time.strftime("%H:%M", time.localtime(time.time()))
            print("INTRUDER: "+in_start)
        (x, y, w, h) = cv2.boundingRect(contour)
        cv2.rectangle(rotated, (x,y), (x+w, y+h), (0, 255, 0),  3)
    
    if (int(time.time() - start) > 50):
        first_frame = rotated
    cv2.imshow('capturing',rotated)
    #cv2.imshow('delta', delta_frame)
    cv2.imshow("thresh_delta", thresh_delta)
    key = cv2.waitKey(35
        )
    if key == ord('q'):
        break
total_time = time.time() - start


print(firebase.get('/Users',None))
print(total_time)
print(frames)
print('fps:'+str(frames/total_time))
video.release()
cv2.destroyAllWindows


