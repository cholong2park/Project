from django.urls import path
from . import views

urlpatterns = [
    path('', views.policy_list),
]