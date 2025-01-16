from rest_framework import serializers
from .models import Deposit, Depositoption, Saving, Savingoption

class DepositOptionSerializer(serializers.ModelSerializer):
    class Meta:
        model = Depositoption
        fields = '__all__'

class DepositSerializer(serializers.ModelSerializer):
    options = DepositOptionSerializer(many=True, read_only=True, source='depositoption_set')
    class Meta:
        model = Deposit
        fields = '__all__'

class SavingOptionSerializer(serializers.ModelSerializer):
    class Meta:
        model = Savingoption
        fields = '__all__'

class SavingSerializer(serializers.ModelSerializer):
    options = SavingOptionSerializer(many=True, read_only=True, source='savingoption_set')
    class Meta:
        model = Saving
        fields = '__all__'