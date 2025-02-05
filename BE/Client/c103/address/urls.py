from django.urls import path
from .views import get_address, put_address

urlpatterns = [
    path("getaddress/", get_address, name="get_address"),  #
    path("putaddress/", put_address, name="put_address"),
]
