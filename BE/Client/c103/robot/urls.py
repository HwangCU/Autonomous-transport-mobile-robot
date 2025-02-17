# login/urls.py
from django.urls import path
from .views import robot_regist, robot_location, robot_mode_change

urlpatterns = [
    path("robot_regist/", robot_regist, name="robot_regist"),
    path("robot_location/", robot_location, name="robot_location"),
    path("robot_mode_change/", robot_mode_change, name="robot_mode_change"),
]
