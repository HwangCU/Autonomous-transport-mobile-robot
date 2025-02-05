from django.urls import path
from .views import get_address

urlpatterns = [
    path("getaddress/", get_address, name="get_address"),
]
