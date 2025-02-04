# login/urls.py
from django.urls import path
from .views import robot_regist

urlpatterns = [
    path("robot_regist/", robot_regist, name="robot_regist"),
]
