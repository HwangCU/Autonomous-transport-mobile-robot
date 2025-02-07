from django.db import models
from users.models import User  # User 모델의 경로에 맞게 조정


class Robot(models.Model):
    user = models.ForeignKey(User, on_delete=models.CASCADE, related_name="robots")
    robot_id = models.CharField(max_length=100)

    def __str__(self):
        return f"{self.robot_id} - {self.user.email}"
