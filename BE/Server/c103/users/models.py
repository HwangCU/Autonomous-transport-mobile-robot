from django.db import models


class User(models.Model):
    username = models.CharField(max_length=150, unique=True)
    password = models.CharField(max_length=255)  # bcrypt 해시 저장
    email = models.EmailField(unique=True)
    address = models.CharField(max_length=255, null=True, blank=True, default=None)

    def __str__(self):
        return self.username
