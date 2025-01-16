from django.db import models

# Create your models here.
class Deposit(models.Model):
    fin_co_no_fin_prdt_cd = models.CharField(max_length=100, primary_key=True)
    fin_prdt_cd = models.CharField(max_length=100) # 금융상품코드
    dcls_month = models.CharField(max_length=10) # 공시 제출월
    fin_co_no = models.CharField(max_length=15) # 금융회사 코드
    kor_co_nm = models.CharField(max_length=100) # 금융회사 명
    fin_prdt_nm = models.CharField(max_length=100) # 금융 상품명
    join_way = models.TextField(null=True) # 가입 방법
    mtrt_int = models.TextField() # 만기 후 이자율
    spcl_cnd = models.TextField() # 우대조건
    join_deny = models.CharField(max_length=10) # 가입제한(1, 2, 3)
    join_member = models.CharField(max_length=200) # 가입대상
    etc_note = models.TextField() # 기타 유의사항
    max_limit = models.CharField(max_length=100, null=True) # 최고한도
    dcls_strt_day = models.CharField(max_length=30, null=True) # 공시 시작일
    dcls_end_day = models.CharField(max_length=30, null=True) # 공시 종료일
    fin_co_subm_day = models.CharField(max_length=30, null=True) # 금융회사 제출일

class Depositoption(models.Model):
    deposit = models.ForeignKey(Deposit, on_delete=models.CASCADE)
    intr_rate_type = models.CharField(max_length=10) # 저축 금리 유형
    intr_rate_type_nm = models.CharField(max_length=10) # 저축 금리 유형명
    save_trm = models.IntegerField() # 저축 기간[단위: 개월]
    intr_rate = models.FloatField(null=True) # 저축 금리 [소수점 2자리]
    max_intr_rate = models.FloatField() # 최고 우대금리 [소수점 2자리]

class Saving(models.Model):
    fin_co_no_fin_prdt_cd = models.CharField(max_length=100, primary_key=True) # pk 대신 사용할 값
    fin_prdt_cd = models.CharField(max_length=100) # 금융상품코드
    dcls_month = models.CharField(max_length=100) # 공시 제출월
    fin_co_no = models.CharField(max_length=100) # 금융회사 코드
    kor_co_nm = models.CharField(max_length=100) # 금융회사 명
    fin_prdt_nm = models.CharField(max_length=100) # 금융 상품명
    join_way = models.TextField(null=True) # 가입 방법
    mtrt_int = models.TextField() # 만기 후 이자율
    spcl_cnd = models.TextField() # 우대조건
    join_deny = models.CharField(max_length=10) # 가입제한(1, 2, 3)
    join_member = models.CharField(max_length=200) # 가입대상
    etc_note = models.TextField() # 기타 유의사항
    max_limit = models.CharField(max_length=100, null=True) # 최고한도
    dcls_strt_day = models.CharField(max_length=30, null=True) # 공시 시작일
    dcls_end_day = models.CharField(max_length=30, null=True) # 공시 종료일
    fin_co_subm_day = models.CharField(max_length=30, null=True) # 금융회사 제출일

class Savingoption(models.Model):
    saving = models.ForeignKey(Saving, on_delete=models.CASCADE)
    intr_rate_type = models.CharField(max_length=100) # 저축 금리 유형
    intr_rate_type_nm = models.CharField(max_length=100) # 저축 금리 유형명
    rsrv_type = models.CharField(max_length=100) # 적립 유형
    rsrv_type_nm = models.CharField(max_length=100) # 적립 유형명
    save_trm = models.IntegerField() # 저축 기간[단위: 개월]
    intr_rate = models.FloatField() # 저축 금리 [소수점 2자리]
    max_intr_rate = models.FloatField() # 최고 우대금리 [소수점 2자리]