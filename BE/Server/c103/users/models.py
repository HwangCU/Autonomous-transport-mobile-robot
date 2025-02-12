from django.db import models


class User(models.Model):
    """모든 사용자 계정의 기본 모델"""

    LOGINSOURCE_CHOICES = [
        ("local", "Local"),
        ("kakao", "Kakao"),
        ("google", "Google"),
    ]

    loginsource = models.CharField(
        max_length=10, choices=LOGINSOURCE_CHOICES
    )  # 계정 제공자 구분
    email = models.EmailField(unique=True)  # 이메일 (로컬/소셜 계정 공통)
    username = models.CharField(max_length=150, null=True, blank=True, default="홍길동")
    address = models.CharField(
        max_length=255, null=True, blank=True, default=None
    )  # 주소
    is_active = models.BooleanField(default=False)  # 활성화 여부

    def __str__(self):
        return f"{self.email} ({self.loginsource})"


class LocalUser(models.Model):
    """로컬 사용자 계정 (앱 회원가입)"""

    user = models.OneToOneField(
        User, on_delete=models.CASCADE, related_name="local_user"
    )
    password = models.CharField(max_length=255)  # bcrypt 해시 저장

    def __str__(self):
        return self.email


class KakaoUser(models.Model):
    """카카오 사용자 계정"""

    user = models.OneToOneField(
        User, on_delete=models.CASCADE, related_name="kakao_user"
    )
    key = models.CharField(max_length=1000)
    usernum = models.CharField(unique=True, max_length=1000)  # 카카오의 고유 유저 ID

    def __str__(self):
        return f"{self.email} (Kakao)"


class GoogleUser(models.Model):
    """구글 사용자 계정"""

    user = models.OneToOneField(
        User, on_delete=models.CASCADE, related_name="google_user"
    )
    key = models.CharField(max_length=1000)
    usernum = models.CharField(unique=True, max_length=1000)  # 구글의 고유 유저 ID

    def __str__(self):
        return f"{self.email} (Google)"
