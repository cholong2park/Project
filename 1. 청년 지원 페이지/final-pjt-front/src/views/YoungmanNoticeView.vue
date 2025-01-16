<template>
 <div class="title-container">
    <h1 class="main-title">청년지원제도 통합검색</h1>
  </div>

  <div class="table-container">
    <table class="table">
      <th></th>
      <tr class="spaced-row">
        <td class="table-heading">지역</td>
        <td class="checkbox-cell">
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRegion('003002001')" /> 서울
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRegion('003002008')" /> 경기
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRegion('003002009')" /> 강원
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRegion('003002011')" /> 충남
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRegion('003002010')" /> 충북
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRegion('003002013')" /> 전남
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRegion('003002012')" /> 전북
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRegion('003002015')" /> 경남
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRegion('003002014')" /> 경북
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRegion('003002016')" /> 제주
          </label>
        </td>
      </tr>
      <tr class="spaced-row">
        <td class="table-heading">관심분야</td>
        <td class="checkbox-cell">
          <label>
            <input type="checkbox" class="checkbox" @change="toggleKeyword('023010')" /> 일자리
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleKeyword('023020')" /> 주거
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleKeyword('023030')" /> 교육
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleKeyword('023040')" /> 복지/문화
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleKeyword('023050')" /> 참여/권리
          </label>
        </td>
      </tr>
    </table>
  </div>
  <!-- 여기서부터 목록 출력 시작 -->
  <div class="policy-list">
      <template v-for="policy in store.policies" :key="policy.id">
        <div v-if="isRegionKeywordSelected(policy.polyBizSecd, policy.polyRlmCd)" class="policy-item">
          <router-link :to="{ name: 'youngmandetail', params: { id: policy.id }}" class="policy-link">
            {{ policy.polyBizSjnm }}
          </router-link>
          <p class="policy-period">신청기간 : {{ policy.rqutPrdCn }}</p>
        </div>
      </template>
    </div>
</template>

<script setup>
import { ref, onMounted } from "vue"
import { useCounterStore } from '@/stores/counter'
import { RouterLink } from 'vue-router'

const store = useCounterStore()
const selectedRegions = ref(new Set())
const selectedKeywords = ref(new Set())

onMounted(() => {
  store.getPolicies()
})

const toggleRegion = (regioncode) => {
  if (selectedRegions.value.has(regioncode)) {
    selectedRegions.value.delete(regioncode)
  } else {
    selectedRegions.value.add(regioncode)
  }
}

const toggleKeyword = (keywordcode) => {
  if (selectedKeywords.value.has(keywordcode)) {
    selectedKeywords.value.delete(keywordcode)
  } else {
    selectedKeywords.value.add(keywordcode)
  }
}

const isRegionKeywordSelected = (regioncode, keywordcode) => {
  return (selectedRegions.value.size === 0 || selectedRegions.value.has(regioncode)) && (selectedKeywords.value.size === 0 || selectedKeywords.value.has(keywordcode))
}

</script>

<style scoped>
.title-container {
  display: flex;
  justify-content: center;
}

.main-title {
  font-size: 36px;
  font-weight: bold;
  color: #2c3e50;
  text-align: center;
  margin: 20px 0;
  padding: 10px;
  border-bottom: 3px solid transparent; /* 초기에는 투명한 선으로 설정 */
  position: relative; /* 부모 요소로부터 상대적 위치 설정 */
}

.main-title::after {
  content: ""; /* 가상 요소 생성 */
  position: absolute; /* 절대 위치 설정 */
  bottom: 0; /* 하단 정렬 */
  left: 50%; /* 가운데 정렬 */
  transform: translateX(-50%); /* 가운데 정렬을 위한 이동 변환 */
  border-bottom: 3px solid #3498db; /* 밑줄 색상 및 두께 설정 */
  width: 400px; /* 직접 밑줄의 길이를 조정합니다. */
}

.table-container {
  display: flex;
  justify-content: center;
}

.table {
  width: 95% !important;
  margin: 15px;
  padding: 30px;
  background-color: aliceblue;
}

.table-heading {
  font-size: large;
  font-weight: bold;
}

.checkbox-cell {
  font-size: large;
}

.spaced-row td {
  padding-top: 10px;
  padding-bottom: 10px;
}

.checkbox {
  margin-right: 5px;
}


.policy-list {
  margin-top: 30px;
}

.policy-item {
  border: 1px solid #ddd;
  padding: 10px;
  margin-bottom: 10px;
  margin-left: 40px;
  margin-right: 40px;
}

.policy-link {
  color: #3498db;
  text-decoration: none;
  font-weight: bold;
}

.policy-period {
  margin-top: 5px;
}

.policy-item:hover {
  /* transform: translateY(-3px); */
  border-color: #afdfff;
  box-shadow: 0 0 5px rgba(72, 174, 242, 0.5);
}
</style>
