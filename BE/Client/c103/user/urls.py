# login/urls.py
from django.urls import path
from .views import get_profile_view, edit_profile_view

urlpatterns = [
    path("get_profile/", get_profile_view, name="get_profile"),
    path("edit_profile/", edit_profile_view, name="edit_profile"),
]
