from django.shortcuts import render
from rest_framework.response import Response
from rest_framework import status
from rest_framework.decorators import api_view
from .models import UserInfo, User
# Create your views here.

@api_view(['GET'])
def index(request):
    print(request.data)
    # userinfo model에 저장을 해야함
    UserInfo.objects.create(
        userinfo = User.objects.get(pk=request.data.get('userinfo')),
        gender = request.data.get('gender'),
        birth_date = request.data.get('birth_date'),
        phone_number = request.data.get('phone_number'),
        address = request.data.get('address'),
        monthly_income = request.data.get('monthly_income'),
    )
    return Response({'message': '성공'}, status=status.HTTP_200_OK)