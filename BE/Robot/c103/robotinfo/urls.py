# login/urls.py
from django.urls import path
from .views import get_current_location_view

urlpatterns = [
    path("location/", get_current_location_view, name="robot_location"),
]
