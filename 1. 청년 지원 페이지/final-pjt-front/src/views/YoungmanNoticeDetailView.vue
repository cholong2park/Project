<template>
  <div class="container">
    <div class="title-container">
      <h1 class="main-title">정책 상세 정보</h1>
    </div>
    <div v-if="loading" class="loading">Loading...</div>
    <div v-else>
      <h2 class="product-title">{{ policy.polyBizSjnm }}</h2>
      <div class="product-container">
        <table class="product-details">
          <tr>
            <td class="detail-label"><strong>정책소개</strong></td>
            <td class="detail-value">{{ policy.polyItcnCn }}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>지원내용</strong></td>
            <td class="detail-value">{{ policy.sporCn }}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>지원규모</strong></td>
            <td class="detail-value">
              {{ policy.sporScvl !== 'null' ? policy.sporScvl : '-' }}
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>사업운영기간</strong></td>
            <td class="detail-value">
              {{ policy.bizPrdCn !== 'null' ? policy.bizPrdCn : '-' }}
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>사업신청기간</strong></td>
            <td class="detail-value">{{ policy.rqutPrdCn }}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>연령 정보</strong></td>
            <td class="detail-value">{{ policy.ageInfo }}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>전공요건</strong></td>
            <td class="detail-value">{{ policy.majrRqisCn }}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>취업상태</strong></td>
            <td class="detail-value">{{ policy.empmSttsCn }}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>특화분야</strong></td>
            <td class="detail-value">{{ policy.splzRlmRqisCn }}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>학력요건</strong></td>
            <td class="detail-value">{{ policy.accrRqisCn }}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>거주지 및 소득조건</strong></td>
            <td class="detail-value">{{ policy.prcpCn !== 'null' ? policy.prcpCn : '-'}}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>추가단서사항</strong></td>
            <td class="detail-value">{{ policy.aditRscn !== 'null' ? policy.aditRscn : '-'}}</td>
          </tr>

          <tr>
            <td class="detail-label"><strong>참여제한대상</strong></td>
            <td class="detail-value">{{ policy.prcpLmttTrgtCn !== 'null' ? policy.prcpLmttTrgtCn : '-'}}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>신청절차</strong></td>
            <td class="detail-value">{{ policy.rqutProcCn !== 'null' ? policy.rqutProcCn : '-'}}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>제출서류</strong></td>
            <td class="detail-value">{{ policy.pstnPaprCn !== 'null' ? policy.pstnPaprCn : '-'}}</td>
          </tr>
          <tr>
            <td class="detail-label"><strong>심사발표</strong></td>
            <td class="detail-value">
              {{ policy.jdgnPresCn !== 'null' ? policy.jdgnPresCn : '-' }}
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>신청사이트 주소</strong></td>
            <td class="detail-value">
              <template v-if="policy.rqutUrla === 'null' || policy.rqutUrla === '-'">
                <p>-</p>
              </template>
              <template v-else>
                <a :href="policy.rqutUrla">신청사이트</a>
              </template>
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>참고사이트 URL 주소1</strong></td>
            <td class="detail-value">
              <template v-if="policy.rfcSiteUrla1 === 'null' || policy.rfcSiteUrla1 === '-'">
                <p>-</p>
              </template>
              <template v-else>
                <a :href="policy.rfcSiteUrla1">참고사이트1</a>
              </template>
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>참고사이트 URL 주소2</strong></td>
            <td class="detail-value">
              <template v-if="policy.rfcSiteUrla2 === 'null' || policy.rfcSiteUrla2 === '-'">
                <p>-</p>
              </template>
              <template v-else>
                <a :href="policy.rfcSiteUrla2">참고사이트2</a>
              </template>
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>주관부처명</strong></td>
            <td class="detail-value">
              {{ policy.mngtMson !== 'null' ? policy.mngtMson : '-' }}
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>주관부처담당자이름</strong></td>
            <td class="detail-value">
              {{
                policy.mngtMrofCherCn !== 'null' ? policy.mngtMrofCherCn : '-'
              }}
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>주관부처담당자연락처</strong></td>
            <td class="detail-value">
              {{ policy.cherCtpcCn !== 'null' ? policy.cherCtpcCn : '-' }}
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>운영기관명</strong></td>
            <td class="detail-value">
              {{ policy.cnsgNmor !== 'null' ? policy.cnsgNmor : '-' }}
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>운영기관담당자이름</strong></td>
            <td class="detail-value">
              {{ policy.tintCherCn !== 'null' ? policy.tintCherCn : '-' }}
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>운영기관담당자연락처</strong></td>
            <td class="detail-value">
              {{
                policy.tintCherCtpcCn !== 'null' ? policy.tintCherCtpcCn : '-'
              }}
            </td>
          </tr>
          <tr>
            <td class="detail-label"><strong>기타사항</strong></td>
            <td class="detail-value">
              {{ policy.etct !== 'null' ? policy.etct : '-' }}
            </td>
          </tr>
        </table>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue';
import { onMounted } from 'vue';
import { useCounterStore } from '@/stores/counter';
import { useRoute } from 'vue-router';

const store = useCounterStore();
const route = useRoute();

const policy = ref({});
const id = ref('');
const loading = ref(true);

onMounted(() => {
  id.value = route.params.id;
  store.selectpolicy = store.policies.filter(
    (policy) => policy.id === Number(id.value)
  );
  policy.value = store.selectpolicy[0];
  loading.value = false;
});
</script>

<style scoped>
.container {
  font-family: 'Arial', sans-serif;
  padding: 10px 20px;
  max-width: 800px;
  margin: 0 auto;
}

.title-container {
  display: flex;
  justify-content: center;
}

.main-title {
  font-size: 36px;
  font-weight: bold;
  color: #2c3e50;
  text-align: center;
  margin: 10px 0;
  padding: 10px;
  position: relative;
  display: inline-block;
}

.main-title::after {
  content: '';
  position: absolute;
  bottom: 0;
  left: 50%;
  transform: translateX(-50%);
  border-bottom: 3px solid #3498db;
  width: 300px;
}

.loading {
  font-size: 24px;
  text-align: center;
  margin-top: 50px;
}

.product-title {
  font-size: 28px;
  color: #3498db;
  margin-bottom: 40px;
  text-align: center;
  white-space: nowrap;
}

.product-container {
  background-color: #f9f9f9;
  border: 1px solid #ddd;
  border-radius: 8px;
  max-width: 1200px;
  padding: 20px;
  margin-bottom: 20px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
  transition: transform 0.3s, box-shadow 0.3s;
}

.product-details {
  width: 100%;
  border-collapse: collapse;
  margin-bottom: 20px;
  text-align: center;
  background-color: #eeeeee;
}

.product-details td {
  padding: 8px; /* 셀 패딩 줄이기 */
  width: 15%;
  border: 1px solid #ddd;
}

.product-details td:first-child {
  width: 1%; /* 원하는 너비로 조정 */
}

.detail-label {
  white-space: nowrap; /* 텍스트가 한 줄로만 표시 */
  width: 100%;
  height: 50px;
  font-weight: bold;
}

.detail-value {
  white-space: normal; /* 텍스트가 넘치는 경우 다음 줄에 이어서 표시*/
  word-wrap: break-word; /* 긴 단어를 여러 줄로 나눕니다 */
  background-color: #fff;
  max-width: 900px;
}

.note-list {
  padding: 0;
  margin: 0;
}

.note-list li {
  list-style-type: none;
  text-align: left; /* 좌측 정렬 */
}
</style>
