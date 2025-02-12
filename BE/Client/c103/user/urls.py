# login/urls.py
from django.urls import path
from .views import get_profile_view

urlpatterns = [
    path("get_profile/", get_profile_view, name="get_profile"),
]
