# login/urls.py
from django.urls import path
from .views import (
    footpath_view,
    send_footpath_view,
    home_sweet_home_view,
)  # , destination_view

urlpatterns = [
    path("footpath/", footpath_view, name="footpath"),
    path("send_footpath/", send_footpath_view, name="send_footpath"),
    path("home_sweet_home/", home_sweet_home_view, name="home_sweet_home"),
    # path("destination/", destination_view, name="destination_view"),
]
