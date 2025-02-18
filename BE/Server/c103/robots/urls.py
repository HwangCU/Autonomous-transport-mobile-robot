# login/urls.py
from django.urls import path
from .views import weight_view, get_gps_view

urlpatterns = [
    path("weight/", weight_view, name="weight"),
    path("get_gps/", get_gps_view, name="get_gps"),
]
