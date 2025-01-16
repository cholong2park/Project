from rest_framework.response import Response
from rest_framework.decorators import api_view
from rest_framework import status

# permission Decorators
from rest_framework.decorators import permission_classes
from rest_framework.permissions import IsAuthenticated

from django.shortcuts import get_object_or_404, get_list_or_404

from .serializers import DepositSerializer, SavingSerializer
from .models import Deposit, Depositoption, Saving, Savingoption
# Create your views here.

@api_view(['GET'])
# @permission_classes([IsAuthenticated])
def deposit_list(request):
    if request.method == 'GET':
        deposits = get_list_or_404(Deposit)
        serializer = DepositSerializer(deposits, many=True)
        return Response(serializer.data)
    
@api_view(['GET'])
# @permission_classes([IsAuthenticated])
def saving_list(request):
    if request.method == 'GET':
        savings = get_list_or_404(Saving)
        serializer = SavingSerializer(savings, many=True)
        return Response(serializer.data)