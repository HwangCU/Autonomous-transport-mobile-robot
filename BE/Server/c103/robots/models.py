from django.db import models
from users.models import User  # User 모델의 경로에 맞게 조정


class Robot(models.Model):
    # 한 사용자가 여러 로봇을 등록할 수 있도록 ForeignKey 사용
    # 한 사용자당 하나의 로봇만 허용하려면 OneToOneField로 변경 가능
    user = models.ForeignKey(User, on_delete=models.CASCADE, related_name="robots")
    robot_id = models.CharField(max_length=100)
    # 필요한 추가 필드 (예: 로봇 모델, 설명 등) 추가 가능

    def __str__(self):
        return f"{self.robot_id} - {self.user.username}"
