from django.urls import path
from . import views

app_name = 'artii'

urlpatterns = [
    path('ping/', views.ping),
    path('pong/', views.pong),
]