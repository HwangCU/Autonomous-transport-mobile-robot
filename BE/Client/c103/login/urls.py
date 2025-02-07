# login/urls.py
from django.urls import path
from .views import login_view, sns_login_view

urlpatterns = [
    path("login/", login_view, name="login"),
    path("sns_login/", sns_login_view, name="sns_login"),
]
