import logging
import os
from datetime import datetime

def setup_logger():
    # 로그 디렉토리 생성
    log_dir = 'logs'
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # 로그 파일명에 날짜 포함
    log_file = f'logs/robot_{datetime.now().strftime("%Y%m%d")}.log'
    
    # 로거 설정
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler()
        ]
    )
    
    return logging.getLogger() 