import serial
from pprint import pprint
import cv2

ser = serial.Serial("/dev/ttyACM0", 9600)

def get_img():
    """Get Image From USB Camera
    Returns:
        numpy.array: Image numpy array
    """
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        print("Camera Error")
        exit(-1)
    ret, img = cam.read()
    cam.release()
    return img

def crop_img(img, size_dict):
    x = size_dict["x"]
    y = size_dict["y"]
    w = size_dict["width"]
    h = size_dict["height"]
    img = img[y : y + h, x : x + w]
    return img

cnt = 0
while 1:
    data = ser.read()
    print(data)
    if data == b"0":
        img = get_img()
        # crop_info = None
        crop_info = {"x": 200, "y": 100, "width": 300, "height": 300}
        if crop_info is not None:
            img = crop_img(img, crop_info)
        cv2.imshow("", img)
        cv2.waitKey(1)
        output_path = f"/home/rokey/Downloads/errorset/{cnt}.jpg"
        cv2.imwrite(output_path, img)
        ser.write(b"1")
        cnt+=1
    else:
        pass