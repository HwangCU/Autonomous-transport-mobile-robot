# client/urls.py
from django.urls import path
from .views import main_dispatch_view

urlpatterns = [
    path("main/", main_dispatch_view, name="main_dispatch"),
]
