# login/urls.py
from django.urls import path
from .views import kakaologin_view

urlpatterns = [
    path("kakako_login/", kakaologin_view, name="kakao_login"),
]
