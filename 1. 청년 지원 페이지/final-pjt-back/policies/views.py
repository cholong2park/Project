from rest_framework.response import Response
from rest_framework.decorators import api_view
from rest_framework import status

# permission Decorators
from rest_framework.decorators import permission_classes
from rest_framework.permissions import IsAuthenticated

from django.shortcuts import get_object_or_404, get_list_or_404

from .serializers import PolicySerializer
from .models import Policy
# # Create your views here.

@api_view(['GET'])
# @permission_classes([IsAuthenticated])
def policy_list(request):
    if request.method == 'GET':
        policies = get_list_or_404(Policy)
        serializer = PolicySerializer(policies, many=True)
        return Response(serializer.data)