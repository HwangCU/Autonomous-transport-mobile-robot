# c103/urls.py
from django.contrib import admin
from django.urls import path, include

urlpatterns = [
    path("admin/", admin.site.urls),
    path("api/", include("login.urls")),  # FE는 /api/login/으로 호출
    path("api/", include("register.urls")),
]
