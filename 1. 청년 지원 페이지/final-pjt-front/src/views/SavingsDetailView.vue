<template>

<div class="title-container">
    <h1 class="main-title">예적금 상세정보</h1>
  </div>
  <div class="container">
    <div v-if="loading" class="loading">Loading...</div>
    <div v-else>
      <div v-if="deposit.length > 0" class="product-container">
        <h2 class="product-title">{{ deposit[0].fin_prdt_nm }}</h2>
        <table class="product-details">
          <tbody>
            <tr>
              <td class="detail-label"><strong>유형</strong></td>
              <td class="detail-value">정기예금</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>금융회사</strong></td>
              <td class="detail-value">{{ deposit[0].kor_co_nm }}</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>가입 방법</strong></td>
              <td class="detail-value">{{ deposit[0].join_way }}</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>만기 후, 이자율</strong></td>
              <td class="detail-value">
                <ul class="note-list">
                  <li
                    v-for="noteLine in deposit[0].mtrt_int
                      .split('-')
                      .filter((item) => item.trim() !== '')"
                    :key="noteLine"
                  >
                    - {{ noteLine.trim() }}
                  </li>
                </ul>
              </td>
            </tr>
            <tr>
              <td class="detail-label"><strong>우대 조건</strong></td>
              <td class="detail-value">
                <ul class="note-list">
                  <li
                    v-for="noteLine in deposit[0].spcl_cnd
                      .split('-')
                      .filter((item) => item.trim() !== '')"
                    :key="noteLine"
                  >
                    - {{ noteLine.trim() }}
                  </li>
                </ul>
              </td>
            </tr>
            <tr>
              <td class="detail-label"><strong>가입 대상</strong></td>
              <td class="detail-value">{{ deposit[0].join_member }}</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>기타 유의사항</strong></td>
              <td class="detail-value">

                <!-- 기타 유의사항을 '-'를 기준으로 분리하여 배열로 변환하고 각 줄을 반복하여 표시 -->
                <ul class="note-list">
                  <li
                    v-for="noteLine in deposit[0].etc_note
                      .split('-')
                      .filter((item) => item.trim() !== '')"
                    :key="noteLine"
                  >
                    - {{ noteLine.trim() }}
                  </li>
                </ul>
              </td>
            </tr>
            <tr>
              <td class="detail-label"><strong>최고 한도</strong></td>
              <td class="detail-value" v-if="deposit[0].max_limit">
                {{ deposit[0].max_limit }}
              </td>
              <td class="detail-value" v-else>X</td>
            </tr>
          </tbody>
        </table>
        <h4 class="options-title">Option</h4>
        <div class="options-container">
          <div
            v-for="option in deposit[0].options"
            class="option-card"
            :key="option.save_trm"
          >
            <ul>
              <li>
                <strong>저축 금리 유형명:</strong>
                {{ option.intr_rate_type_nm }}
              </li>
              <li>
                <strong>저축 기간 [단위: 개월]:</strong> {{ option.save_trm }}
              </li>
              <li><strong>저축 금리:</strong> {{ option.intr_rate }}</li>
              <li>
                <strong>최고 우대금리:</strong> {{ option.max_intr_rate }}
              </li>
            </ul>
          </div>
        </div>
      </div>
      <div v-else class="product-container">
        <h2 class="product-title">{{ saving[0].fin_prdt_nm }}</h2>
        <table class="product-details">
          <tbody>
            <tr>
              <td class="detail-label"><strong>유형</strong></td>
              <td class="detail-value">적금</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>금융회사</strong></td>
              <td class="detail-value">{{ saving[0].kor_co_nm }}</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>가입 방법</strong></td>
              <td class="detail-value">{{ saving[0].join_way }}</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>만기 후 이자율</strong></td>
              <td class="detail-value">{{ saving[0].mtrt_int }}</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>우대 조건</strong></td>
              <td class="detail-value">{{ saving[0].spcl_cnd }}</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>가입 대상</strong></td>
              <td class="detail-value">{{ saving[0].join_member }}</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>기타 유의사항</strong></td>
              <td class="detail-value">{{ saving[0].etc_note }}</td>
            </tr>
            <tr>
              <td class="detail-label"><strong>최고 한도</strong></td>
              <td class="detail-value">{{ saving[0].max_limit }}</td>
            </tr>
          </tbody>
        </table>
        <h4 class="options-title">Option</h4>
        <div class="options-container">
          <div
            v-for="option in saving[0].options"
            class="option-card"
            :key="option.save_trm"
          >
            <ul>
              <li>
                <strong>저축 금리 유형명:</strong>
                {{ option.intr_rate_type_nm }}
              </li>
              <li><strong>적립 유형명:</strong> {{ option.rsrv_type_nm }}</li>
              <li>
                <strong>저축 기간 [단위: 개월]:</strong> {{ option.save_trm }}
              </li>
              <li><strong>저축 금리:</strong> {{ option.intr_rate }}</li>
              <li>
                <strong>최고 우대금리:</strong> {{ option.max_intr_rate }}
              </li>
            </ul>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from "vue";
import { onMounted } from "vue";
import { useCounterStore } from "@/stores/counter";
import { useRoute } from "vue-router";

const store = useCounterStore();
const route = useRoute();

const deposit = ref([]);
const saving = ref([]);
const name = ref("");
const loading = ref(true);

onMounted(() => {
  name.value = route.params.name;
  deposit.value = store.deposits.filter(
    (deposit) => deposit.fin_co_no_fin_prdt_cd === name.value
  );
  saving.value = store.savings.filter(
    (saving) => saving.fin_co_no_fin_prdt_cd === name.value
  );
  loading.value = false;
});
</script>



<style scoped>

.main-title {
  font-size: 36px;
  font-weight: bold; 
  color: #2c3e50; 
  text-align: center; 
  margin: 20px 0; 
  padding: 10px; 
  border-bottom: 3px solid transparent;
  display: inline-block;
  position: relative;
}

.main-title::after {
  content: "";
  position: absolute;
  bottom: 0;
  left: 50%;
  transform: translateX(-50%);
  border-bottom: 3px solid #3498db;
  width: 300px;
}

.title-container {
  display: flex;
  justify-content: center;
}

.container {
  font-family: "Arial", sans-serif;
  padding: 50px 20px;
  max-width: 1000px;
  margin: 0 auto;
}

.loading {
  font-size: 24px;
  text-align: center;
  margin-top: 50px;
}

.product-container {
  background-color: #f9f9f9;
  border: 1px solid #ddd;
  border-radius: 8px;
  padding: 20px;
  margin-bottom: 20px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
  transition: transform 0.3s, box-shadow 0.3s;
}

.product-title {
  font-size: 28px;
  color: #3498db;
  margin-bottom: 40px;
  text-align: center;
  display: flex;
  justify-content: center;
  align-items: center;
}

.product-details {
  width: 80%;
  border-collapse: collapse;
  margin-bottom: 20px;
  text-align: center;
  margin: 0 auto;
}

.product-details td {
  padding: 10px;
  border: 1px solid #ddd;
}

.detail-label {
  padding: 30px; /* 셀 내부 여백 조정 */
  width: 18%; /* 셀 너비 조정 */
  height: 50px;
  box-sizing: border-box;
}

.detail-value {
  background-color: #fff;
}

.options-title {
  font-size: 24px;
  color: #2c3e50;
  margin-top: 20px;
  text-align: center;
  padding-top: 30px;
}

.options-container {
  display: flex;
  flex-wrap: wrap;
  gap: 20px;
  justify-content: center;
}

.option-card {
  background-color: #ffffff;
  border: 1px solid #ddd;
  border-radius: 8px;
  padding: 15px;
  width: 250px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  transition: transform 0.3s, box-shadow 0.3s;
}

.option-card:hover {
  /* transform: translateY(-5px); */
  box-shadow: 0 0 5px rgba(72, 174, 242, 0.7);
}

.option-card ul {
  list-style: none;
  padding: 0;
}

.option-card ul li {
  margin-bottom: 8px;
}

.option-card ul li strong {
  color: #3498db;
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
