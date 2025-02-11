# login/urls.py
from django.urls import path
from .views import footpath_view

urlpatterns = [
    path("footpath/", footpath_view, name="footpath"),
]
