from django.db import models
from django.contrib.auth.models import AbstractUser


# Create your models here.
class User(AbstractUser):
    pass
    

class UserInfo(models.Model):
    userinfo = models.ForeignKey(User, on_delete=models.CASCADE)
    gender = models.CharField(max_length=100)
    birth_date = models.DateField()
    phone_number = models.CharField(max_length=15)
    address = models.CharField(max_length=255)
    monthly_income = models.DecimalField(max_digits=10, decimal_places=2)
    # profile_image = models.ImageField(upload_to='profile_images/', blank=True, null=True)
    # created_at = models.DateTimeField(auto_now_add=True)
