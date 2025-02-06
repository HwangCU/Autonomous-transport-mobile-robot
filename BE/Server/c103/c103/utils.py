from itsdangerous import URLSafeTimedSerializer
from django.conf import settings

# 시크릿 키 설정
SECRET_KEY = settings.SECRET_KEY
TOKEN_EXPIRATION = 3600  # 1시간 (초 단위)


def generate_email_token(email):
    """이메일 인증 토큰 생성"""
    serializer = URLSafeTimedSerializer(SECRET_KEY)
    return serializer.dumps(email, salt="email-confirm")


def verify_email_token(token):
    """이메일 인증 토큰 검증"""
    serializer = URLSafeTimedSerializer(SECRET_KEY)
    try:
        email = serializer.loads(token, salt="email-confirm", max_age=TOKEN_EXPIRATION)
        return email
    except Exception:
        return None
