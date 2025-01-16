from django.urls import path, include
from . import views

urlpatterns = [
    path('deposits/', views.deposit_list),
    path('savings/', views.saving_list),
]