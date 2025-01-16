# create sql db
import sqlite3

# 데이터베이스 연결 (파일로 저장되며, 파일 이름은 예시로 'example.db')
conn = sqlite3.connect('B2_Nigeria.db')
cursor = conn.cursor()


create_table_query = '''
CREATE TABLE IF NOT EXISTS 제품 (
    id INTEGER PRIMARY KEY AUTOINCREMENT, -- 고유 ID (자동 증가)
    datetime TEXT,                        -- 날짜 및 시간 (ISO 형식으로 저장: "YYYY-MM-DD HH:MM:SS")
    uuid TEXT UNIQUE,                     -- UUID (고유 식별자)
    is_defective INTEGER,                 -- 제품 상태 (0: 양품, 1: 불량품)
    defect_reason TEXT,                    -- 불량 이유 (불량인 경우에만 이유 저장)
    captured_image BLOB                            -- 이미지 저장을 위한 필드
)
'''

# 테이블 생성
cursor.execute(create_table_query)

# 변경 사항 저장
conn.commit()