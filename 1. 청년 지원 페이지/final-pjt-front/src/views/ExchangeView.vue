<!-- 규림아 현재 내가 값을 입력하고 바꾸고 싶은 나라의 통화 버튼을 누르고 환전 버튼을 클릭하면 결과값이 나오게 했어 -->
<!-- 디자인적 요소랑 혹시 변환 과정이 좀더 자연스러울 수 있는지 확인 좀 -->
<template>
  <div class="title-container">
    <h1 class="main-title">환율 계산</h1>
  </div>
  <div class="exchange-container">
    <div class="exchange-box">
      <div class="input-group">
        <div class="dropdown" @click="toggleDropdown">
          <button class="dropbtn">{{ selectedCountryName }}</button>
          <div class="dropdown-content" :class="{ show: dropdownOpen }">
            <button
              v-for="exchange in Exchage_list"
              :key="exchange.cur_unit"
              @click="select(exchange)"
            >
              {{ exchange.cur_nm }}
            </button>
          </div>
        </div>
        <input
          type="text"
          v-model="money"
          placeholder="금액 입력"
          class="input-field"
        />
        <button @click="input" class="exchange-button">환전</button>
      </div>
    </div>
    <!-- 환전된 금액 표시 -->
    <h2 class="result">환전결과: {{ convertedMoney }} {{ selectedCountryCode }}</h2>
  </div>
  <div class="card-container">
    <div v-for="exchange in Exchage_list" :key="exchange.cur_unit" class="card">
      <p class="contry-name"> {{ exchange.cur_nm }}</p>
      <p>통화코드: {{ exchange.cur_unit }}</p>
      <p>매매 기준율: {{ exchange.deal_bas_r }}</p>
    </div>
  </div>
</template>


<script setup>
import { onMounted, ref } from 'vue';
import axios from 'axios';

// 화면에 표시할 데이터를 저장할 반응형 참조 변수
const money = ref('');
const convertedMoney = ref(0); // 환전된 금액을 저장할 변수
const Exchage_list = ref([]);
const selectedRate = ref(0); // 선택된 환율 정보를 저장할 반응형 객체
const selectedCountryName = ref('나라선택 ▾'); // 선택된 나라의 이름을 저장할 변수
const selectedCountryCode = ref(''); // 선택된 나라의 코드를 저장할 변수
const dropdownOpen = ref(false); // 드롭다운의 열림/닫힘 상태를 저장할 변수

onMounted(async () => {
  try {
    const response = await axios({
      method: 'GET',
      url: '/api/site/program/financial/exchangeJSON',
      params: {
        authkey: '1mskQmvLIK0nO7DyYtqzgcFI3RqIZuCe',
        data: 'AP01',
      },
    });
    console.log(response);
    Exchage_list.value = response.data; // response.data는 현재 환율 정보가 담겨있는 list
  } catch (error) {
    console.error(error);
  }
});

const input = () => {
  // 입력된 금액을 선택된 환율로 환전
  convertedMoney.value = parseFloat(money.value) / selectedRate.value;
};

const select = (exchange) => {
  // 선택된 환율 정보 업데이트
  selectedRate.value = parseFloat(exchange.deal_bas_r.replace(/,/g, ''));
  // 선택된 나라의 이름 업데이트
  selectedCountryName.value = exchange.cur_nm;
  // 선택된 나라의 통화 코드 업데이트
  selectedCountryCode.value = exchange.cur_unit;

  // 드롭다운 닫기
  dropdownOpen.value = true;
};

const toggleDropdown = () => {
  dropdownOpen.value = !dropdownOpen.value;
};

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
  content: ''; /* 가상 요소 생성 */
  position: absolute; /* 절대 위치 설정 */
  bottom: 0; /* 하단 정렬 */
  left: 50%; /* 가운데 정렬 */
  transform: translateX(-50%); /* 가운데 정렬을 위한 이동 변환 */
  border-bottom: 3px solid #3498db; /* 밑줄 색상 및 두께 설정 */
  width: 200px; /* 직접 밑줄의 길이를 조정합니다. */
}

.exchange-container {
  max-width: 700px;
  margin: 32px auto;
  padding: 30px;
  background-color: aliceblue;
  /* border-radius: 10px; */
  /* box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); */
}

.exchange-box {
  padding: 20px;
  border-radius: 10px;
  text-align: center;
  margin-bottom: 20px;
}

.contry-name {
  color: rgb(36, 54, 153);
  font-weight: bold;
  font-size: 20px;
}

.input-field {
  width: 50%;
  padding: 15px;
  margin-bottom: 20px;
  border: 1px solid #ddd;
  border-radius: 5px;
  margin: 5px;
  font-size: 15px;
}

.exchange-button {
  background-color: #3498db;
  color: white;
  padding: 12px 16px;
  border: none;
  border-radius: 5px;
  cursor: pointer;
  font-size: 17px;
}

.input-group {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 1px; /* 요소 간의 간격 조정 */
  padding-right: 30px;
}

.result {
  margin-top: 20px;
  font-size: 18px;
  font-weight: bold;
  color: #0d161f;
  text-align: center;
}

.input-field:focus {
  outline: none; /* 기본 포커스 스타일 제거 */
  border-color: #afdfff; /* 원하는 색상으로 변경 */
  box-shadow: 0 0 5px rgba(72, 174, 242, 0.5); /* 원하는 스타일 추가 */
}

.dropdown {
  position: relative;
  display: inline-block;
  
}

.dropbtn {
  background-color: rgb(245, 245, 245);
  color: rgb(0, 0, 0);
  padding: 12px 16px;
  font-size: 16px;
  border: none;
  border-radius: 5px;
  cursor: pointer;
}

.dropdown-content {
  display: none;
  position: absolute;
  background-color: #ffffff;
  min-width: 170px;
  box-shadow: 0 8px 16px 0 rgba(0, 0, 0, 0.2);
  z-index: 1;
  max-height: 160px; /* 최대 높이 지정 */
  overflow-y: auto; /* 세로 스크롤바 표시 */
}

.dropdown-content button {
  display: block;
  width: 150px; /* 버튼의 너비 설정 */
  padding: 8px 8px; /* 내용과의 간격 설정 */
  color: black;
  padding: 8px 10px;
  background-color: white; /* 배경색 흰색으로 지정 */
  border: none;
}

.dropdown-content button:hover {
  background-color: #f1f1f1;
  width: 150px;
}

.dropdown-content.show {
  display: block;
}

.card-container {
  display: flex;
  flex-wrap: wrap;
  gap: 50px; /* 카드 간 간격 설정 */
  justify-content: center;
  margin: 80px 200px;
}

.card {
  max-width: 250px; /* 최대 가로 길이 설정 */
  width: 100%; /* 최대 가로 길이를 넘어갈 경우 줄 바꿈 */
  height: 150px;
  border: 1px solid #ddd;
  border-radius: 5px;
  padding: 10px;
  display: flex;
  flex-direction: column; /* 요소들을 세로로 배치 */
  justify-content: center; /* 내용을 수직 가운데 정렬 */
  box-shadow: 2px 2px 5px rgb(185, 185, 185);
}

.card p {
  margin: 5px 0; /* 기본 마진 제거 */
  flex: 1; /* 내용이 많아도 카드 높이에 따라 동적으로 조절됨 */
  display: flex;
  align-items: center; /* 내용을 수직 가운데 정렬 */
  justify-content: center; /* 내용을 수평 가운데 정렬 */
}

</style>
