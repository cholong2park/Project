<template>
<div title-container>
    <h1 class="main-title">추천 서비스 결과</h1>
    <h1>금융상품</h1>
    <div class="cards-container">
      <div v-for="ds in store.deposit_saving" :key="ds.fin_co_no_fin_prdt_cd" class="card">
        <RouterLink :to="{ name: 'savingdetail', params: { name: ds.fin_co_no_fin_prdt_cd }}">
          <h3>{{ ds.fin_prdt_nm }}</h3>
        </RouterLink>
        <p>{{ ds.kor_co_nm }}</p>
      </div>
    </div>
    <br>
    <h1>청년지원제도</h1>
    <div class="cards-container">
      <div v-for="p in store.policy" :key="p.id" class="card">
        <RouterLink :to="{ name: 'youngmandetail', params: { id: p.id }}">
          <h3>{{ p.polyBizSjnm }}</h3>
        </RouterLink>
      </div>
    </div>
</div>

</template>

<script setup>
import { useCounterStore } from '@/stores/counter'
import { RouterLink } from 'vue-router'

const store = useCounterStore()
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
  width: 290px; /* 직접 밑줄의 길이를 조정합니다. */
}
.table-container {
  display: flex;
  flex-direction: column;
  align-items: center;
}

.description {
  font-size: 20px;
  text-align: center;
  margin-left: 10px;
  margin-top: -10px;
  padding-top: 20px;
}
  
.table {
  width: 40% !important;
  padding: 30px 30px 50px 30px;
  background-color: aliceblue;
  border-collapse: separate;
  border-spacing: 0 10px;
}

h1 {
  text-align: center;
  padding-top: 40px;
}

p {
  font-weight: bolder;
}

.empty-space {
  height: 20px;
}

.checkbox-group {
  padding-left: 10px;
}

.button-container {
  display: flex;
  justify-content: center;
  margin-top: 10px;
}

.result-button {
  padding: 10px 20px;
  margin-top: 15px;
  background-color: #4CAF50;
  color: white;
  border: none;
  border-radius: 5px;
  font-size: 16px;
  cursor: pointer;
}

.result-button:hover {
  color: #f3f3f3;
  transition: color 0.3s;
  background-color: #45A049;
  outline: none;
}

/* card 관련 스타일 */
.cards-container {
  display: flex;
  justify-content: center;
  gap: 30px;
  width: 90%;
  padding-left: 80px;
  padding-top: 50px;
  gap: 20px;
}

.card {
  background-color: white;
  border: 1px solid #ddd;
  border-radius: 8px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
  padding: 20px;
  width: 250px;
  height: 120px;
  text-align: center;
  transition: transform 0.3s, box-shadow 0.3s, border 0.3s;
  font-size: 14px;
  margin: 0 20px
}

.card:hover {
  transform: translateY(-5px);
  border-color: #afdfff;
  box-shadow: 0 0 5px rgba(72, 174, 242, 0.5);
}

.card h3 {
  font-size: 1.5em;
  color: #3498db;
  margin-bottom: 10px;
}

.card p {
  font-size: 1.2em;
  color: #2c3e50;
}

.card a {
  text-decoration: none;
  color: inherit;
}

.card a:hover {
  color: #3498db;
}
</style>