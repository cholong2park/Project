from django.db import models

# Create your models here.
class Policy(models.Model):
    polyBizSecd = models.CharField(max_length=15) # 기관코드
    polyBizTy = models.CharField(max_length=30) # 기관 및 지자체 구분
    polyBizSjnm = models.CharField(max_length=50) # 정책명
    polyItcnCn = models.TextField() # 정책소개
    sporCn = models.TextField() # 지원내용
    sporScvl = models.CharField(max_length=100, null=True) # 지원규모
    bizPrdCn = models.CharField(max_length=100, null=True) # 사업운영기간내용
    prdRpttSecd = models.CharField(max_length=6) # 사업신청기간반복구분코드
    rqutPrdCn = models.TextField() # 사업신청기간내용
    ageInfo = models.CharField(max_length=100) # 연령 정보
    majrRqisCn = models.CharField(max_length=100, null=True) # 전공요건내용
    empmSttsCn = models.CharField(max_length=100, null=True) # 취업상태내용
    splzRlmRqisCn = models.CharField(max_length=100, null=True) # 특화분야내용
    accrRqisCn = models.CharField(max_length=100, null=True) # 학력요건내용
    prcpCn = models.TextField() # 거주지 및 소득조건내용
    aditRscn = models.TextField() # 추가단서사항내용
    prcpLmttTrgtCn = models.CharField(max_length=300, null=True) # 참여제한대상내용
    rqutProcCn = models.TextField() # 신청절차내용
    pstnPaprCn = models.TextField(null=True) # 제출서류내용
    jdgnPresCn = models.TextField(null=True) # 심사발표내용
    rqutUrla = models.CharField(max_length=300, null=True) # 신청사이트주소
    rfcSiteUrla1 = models.CharField(max_length=500, null=True) # 참고사이트URL주소1
    rfcSiteUrla2 = models.CharField(max_length=500, null=True) # 참고사이트URL주소2
    mngtMson = models.CharField(max_length=100, null=True) # 주관부처명
    mngtMrofCherCn = models.CharField(max_length=50, null=True) # 주관부처담당자이름
    cherCtpcCn = models.CharField(max_length=50, null=True) # 주관부처담당자연락처
    cnsgNmor = models.CharField(max_length=50, null=True) # 운영기관명
    tintCherCn = models.CharField(max_length=50, null=True) # 운영기관담당자이름
    tintCherCtpcCn = models.CharField(max_length=300, null=True) # 운영기관담당자연락처
    etct = models.TextField(null=True) # 기타사항
    polyRlmCd = models.CharField(max_length=6) # 정책분야코드