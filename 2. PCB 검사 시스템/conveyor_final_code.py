import serial
import requests
import numpy
from pprint import pprint
from requests.auth import HTTPBasicAuth
import cv2
from collections import Counter
import uuid
from datetime import datetime
import sqlite3

ser = serial.Serial("/dev/ttyACM0", 9600)

# API endpoint(L model)
api_url = "https://suite-endpoint-api-apne2.superb-ai.com/endpoints/cf415ecc-30f6-43ab-aa69-a125b2a9c521/inference"

# ACCESS_KEY = "KOrRhrpazp8LoTl4DRk7ca8kpIIqFyb14ZT17edw" => (필수)API 접근하기 위한 KEY

# 데이터베이스 연결
conn = sqlite3.connect('B2_Nigeria.db')
cursor = conn.cursor()

# 클래스별 고유 색상 정의
class_colors = {
    "BOOTSEL": (0, 0, 255),       # 빨강
    "USB": (255, 0, 255),         # 보라
    "CHIPSET": (0, 255, 0),       # 초록
    "OSCILLATOR": (255, 255, 0),  # 하늘색
    "RASPBERRY PICO": (255, 0, 0), # 진한 보라
    "HOLE": (0, 255, 255),        # 노랑
    "DEBUG": (125, 0, 125)
}

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

def inference_reqeust(img: numpy.array, api_rul: str):
    global ACCESS_KEY
    """_summary_
    Args:
        img (numpy.array): Image numpy array
        api_rul (str): API URL. Inference Endpoint
    """
    success, img_encoded = cv2.imencode(".jpg", img)
    if not success:
        print("Failed to encode image")
        return
    
    try:
        response = requests.post(
            url=api_url,
            auth=HTTPBasicAuth("kdt2024_1-7", ACCESS_KEY),
            headers={"Content-Type": "image/jpeg"},
            data=img_encoded.tobytes(),
            )
        
        if response.status_code == 200:
            print("Image sent successfully")
            pprint(response.json())
            return response.json()
        
        else:
            print(f"Failed to send image. Status code: {response.status_code}")
    
    except requests.exceptions.RequestException as e:
        print(f"Error sending request: {e}")


# 데이터 삽입 함수
def insert_data(datetime_value, uuid_value, is_defective, defect_reason=None, captured_image=None):
    insert_query = '''
    INSERT INTO 제품 (datetime, uuid, is_defective, defect_reason, captured_image)
    VALUES (?, ?, ?, ?, ?)
    '''
    cursor.execute(insert_query, (datetime_value, uuid_value, is_defective, defect_reason, captured_image))
    conn.commit()

cnt = 0
while 1:
    data = ser.read()
    print(data)
    if data == b"0":
        img = get_img()
        
        result = inference_reqeust(img, api_url) # result = response.json()
        
        # 클래스별 개수 카운팅
        class_counts = Counter(obj["class"] for obj in result["objects"])
        
        # 클래스별 개수를 우측 상단에 표시
        y_offset = 20  # 텍스트 시작 y 좌표 (초기값)
        for class_name, count in class_counts.items():
            color = class_colors.get(class_name, (255, 255, 255))
            text = f"{class_name}: {count}"
            cv2.putText(img, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
            y_offset += 12  # 다음 텍스트의 y 좌표를 아래로 이동
        
        # 박스와 클래스 표시
        for obj in result["objects"]:
            box = obj["box"]
            class_name = obj["class"]
            score = obj["score"]

            # 좌표 추출
            x_min, y_min, x_max, y_max = box

            # 색상 선택 (클래스별 고유 색상, 기본값은 흰색)
            color = class_colors.get(class_name, (255, 255, 255))

            # 사각형 그리기
            cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color, 1)

            # 클래스와 점수 텍스트 추가
            text = f"{class_name} ({score:.2f})"
            cv2.putText(img, text, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        


        # 결과 이미지 저장
        output_path = f"/home/rokey/Image/Output/{cnt}.jpg"  # 저장할 경로 지정
        cv2.imwrite(output_path, img)
        cv2.imshow("",img)
        
        # SQL 저장 부분
        # 이미지를 바이너리 데이터로 변환
        _, buffer = cv2.imencode('.jpg', img)  # JPEG 형식으로 인코딩
        blob_data = buffer.tobytes()  # 바이너리 변환

        # 초기 데이터값 세팅
        datetime_value = datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # 현재 날짜와 시간
        uuid_value = str(uuid.uuid4())  # UUID 생성
        is_defective = 0
        defect_reason = None
        captured_image = blob_data

        # HOLE 문제 처리
        if class_counts['HOLE'] == 4:
            pass
        else:
            is_defective = 1  # 불량품인 경우 (1: 불량, 0: 양품)
            defect_reason = (defect_reason or "") + "HOLE "
        
        # BOOTSEL 문제 처리
        if class_counts['BOOTSEL'] == 1:
            pass
        else:
            is_defective = 1  # 불량품인 경우 (1: 불량, 0: 양품)
            defect_reason = (defect_reason or "") + "BOOTSEL "
        
        # CHIPSET 문제 처리
        if class_counts['CHIPSET'] == 1:
            pass
        else:
            is_defective = 1  # 불량품인 경우 (1: 불량, 0: 양품)
            defect_reason = (defect_reason or "") + "CHIPSET "
        
        # OSCILLATOR 문제 처리
        if class_counts['OSCILLATOR'] == 1:
            pass
        else:
            is_defective = 1  # 불량품인 경우 (1: 불량, 0: 양품)
            defect_reason = (defect_reason or "") + "OSCILLATOR "
        
        # USB 문제 처리
        if class_counts["USB"] == 1:
            pass
        else:
            is_defective = 1  # 불량품인 경우 (1: 불량, 0: 양품)
            defect_reason = (defect_reason or "") + "USB "

        # DEBUG 문제 처리
        if class_counts["DEBUG"] == 1:
            pass
        else:
            is_defective = 1  # 불량품인 경우 (1: 불량, 0: 양품)
            defect_reason = (defect_reason or "") + "DEBUG "
        
        # RASPBERRY PICO 문제 처리
        if class_counts["RASPBERRY PICO"] == 1:
            pass
        else:
            is_defective = 1  # 불량품인 경우 (1: 불량, 0: 양품)
            defect_reason = (defect_reason or "") + "RASPBERRY PICO "
        if is_defective==1:
            cv2.waitKey(0)
        else:
            cv2.waitKey(1)
        # 데이터 삽입 호출
        insert_data(datetime_value, uuid_value, is_defective, defect_reason, captured_image)
        ser.write(b"1")
    else:
        pass