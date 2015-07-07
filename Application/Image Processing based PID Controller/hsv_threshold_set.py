import numpy as np
import cv2

# Initialize camera 

############################################
cap=cv2.VideoCapture(1)

def nothing(x):
    pass

# Creating a window for later use
cv2.namedWindow('Hsv_Color')

# Starting with 100's to prevent error while masking
Lh,Ls,Lv = [0, 0, 0]
Uh,Us,Uv = [120, 120, 120]

# Creating track bar
cv2.createTrackbar('Lh', 'Hsv_Color', Lh, 179, nothing)
cv2.createTrackbar('Ls', 'Hsv_Color', Ls, 255, nothing)
cv2.createTrackbar('Lv', 'Hsv_Color', Lv, 255, nothing)
cv2.createTrackbar('Uh', 'Hsv_Color', Uh, 179, nothing)
cv2.createTrackbar('Us', 'Hsv_Color', Us, 255, nothing)
cv2.createTrackbar('Uv', 'Hsv_Color', Uv, 255, nothing)

while(1):
    # Read the image
    ret,img=cap.read()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
    Lh = cv2.getTrackbarPos('Lh', 'Hsv_Color')
    Ls = cv2.getTrackbarPos('Ls', 'Hsv_Color')
    Lv = cv2.getTrackbarPos('Lv', 'Hsv_Color')
    Uh = cv2.getTrackbarPos('Uh', 'Hsv_Color')
    Us = cv2.getTrackbarPos('Us', 'Hsv_Color')
    Uv = cv2.getTrackbarPos('Uv', 'Hsv_Color')

    # Normal masking algorithm
    lower = np.array([Lh, Ls, Lv])
    upper = np.array([Uh, Us, Uv])

    mask = cv2.inRange(hsv, lower, upper)
    cv2.imshow('mask',mask)
    res = cv2.bitwise_and(img, img, mask= mask)
    # cv2.imshow('res',res)
    
    if cv2.waitKey(100)==27:
        break

############################################
## Close and exit
cv2.waitKey(0)
cv2.destroyAllWindows()
