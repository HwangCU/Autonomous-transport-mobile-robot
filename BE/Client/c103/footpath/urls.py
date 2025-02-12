# login/urls.py
from django.urls import path
from .views import footpath_view  # , destination_view

urlpatterns = [
    path("footpath/", footpath_view, name="footpath"),
    # path("destination/", destination_view, name="destination_view"),
]
