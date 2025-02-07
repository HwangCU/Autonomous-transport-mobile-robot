from django.db import models


class SocialUser(models.Model):
    email = models.CharField(max_length=150, unique=True)
    address = models.CharField(max_length=255, null=True, blank=True, default=None)
    social = models.CharField(max_length=255)

    def __str__(self):
        return self.email
