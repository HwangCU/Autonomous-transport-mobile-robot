# login/urls.py
from django.urls import path
from .views import footpath_view, send_footpath_view  # , destination_view

urlpatterns = [
    path("footpath/", footpath_view, name="footpath"),
    path("send_footpath/", send_footpath_view, name="send_footpath"),
    # path("destination/", destination_view, name="destination_view"),
]
