<template>
  <div class="title-container">
    <h1 class="main-title">예적금 통합검색</h1>
  </div>
  <div class="table-container">
    <table class="table">
      <tr class="spaced-row">
        <td class="table-heading">유형</td>
        <td class="checkbox-cell">
          <label>
            <input type="checkbox" class="checkbox" v-model="showDeposits" /> 예금
          </label>
          <label>
            <input type="checkbox" class="checkbox" v-model="showSavings" /> 적금
          </label>
        </td>
      </tr>
      <tr class="spaced-row">
        <td class="table-heading">저축 금리 유형명</td>
        <td class="checkbox-cell">
          <label>
            <input type="checkbox" class="checkbox" @change="toggleIRType('S')" /> 단리
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleIRType('M')" /> 복리
          </label>
        </td>
      </tr>
      <tr v-if="showSavings" class="spaced-row">
        <td class="table-heading">적립 유형명</td>
        <td class="checkbox-cell">
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRType('S')" /> 정액적립식
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRType('F')" /> 자유적립식
          </label>
        </td>
      </tr>
      <tr class="spaced-row">
        <td class="table-heading">최고 우대금리</td>
        <td class="checkbox-cell">
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRate(1.0, 2.0)" /> 1.0~2.0%
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRate(2.0, 3.0)" /> 2.0~3.0%
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRate(3.0, 4.0)" /> 3.0~4.0%
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRate(4.0, 5.0)" /> 4.0~5.0%
          </label>
          <label>
            <input type="checkbox" class="checkbox" @change="toggleRate(5.0, 100.0)" /> 5.0%~
          </label>
        </td>
      </tr>
    </table>
  </div>
  <div class="cards-container">
    <div v-if="!showDeposits && !showSavings" class="cat-container">
      <img src="@/assets/cat2.png" alt="고양이 이미지" class="cat">
    </div>
    <template v-if="showDeposits" v-for="deposit in store.deposits" :key="deposit.fin_co_no_fin_prdt_cd">         
      <template v-if="isDepositOptionSelected(deposit.options)">
        <div class="card">
          <RouterLink :to="{ name: 'savingdetail', params: { name: deposit.fin_co_no_fin_prdt_cd }}">
            <h3>{{ deposit.fin_prdt_nm }}</h3>
          </RouterLink>
          <p>{{ deposit.kor_co_nm }}</p>
        </div>
      </template>
    </template>
    <template v-if="showSavings" v-for="saving in store.savings" :key="saving.fin_co_no_fin_prdt_cd">
      <template v-if="isSavingOptionSelected(saving.options)">
        <div class="card">
          <RouterLink :to="{ name: 'savingdetail', params: { name: saving.fin_co_no_fin_prdt_cd }}">
            <h3>{{ saving.fin_prdt_nm }}</h3>
          </RouterLink>
          <p>{{ saving.kor_co_nm }} </p>
        </div>
      </template>
    </template>
  </div>
</template>

<script setup>
import { ref, computed, onMounted } from "vue"
import { useCounterStore } from '@/stores/counter'
import { RouterLink } from 'vue-router'

const store = useCounterStore()

const showDeposits = ref(false)
const showSavings = ref(false)
const selectedDIRType = ref(new Set())
const selectedSIRType = ref(new Set())
const selectedRType = ref(new Set())
const selectedRate = ref({ min_rate: new Set(), max_rate: new Set() })

onMounted(() => {
  store.getDeposits()
  store.getSavings()
  // console.log(store.deposits)
  // console.log(store.savings)
})

const filteredDeposits = computed(() => {
  return showDeposits.value ? store.deposits : []
})

const filteredSavings = computed(() => {
  return showSavings.value ? store.savings : []
})

const toggleIRType = (IRType) => {
  if (selectedDIRType.value.has(IRType)) {
    selectedDIRType.value.delete(IRType)
  } else {
    selectedDIRType.value.add(IRType)
  }
  if (selectedSIRType.value.has(IRType)) {
    selectedSIRType.value.delete(IRType)
  } else {
    selectedSIRType.value.add(IRType)
  }
}

const toggleRType = (RType) => {
  if (selectedRType.value.has(RType)) {
    selectedRType.value.delete(RType)
  } else {
    selectedRType.value.add(RType)
  }
}

const isRateInRange = (rate) => {
  let min_value = Math.min(...selectedRate.value.min_rate)
  // console.log(min_value)
  let max_value = Math.max(...selectedRate.value.max_rate)
  if (rate >= min_value && rate <= max_value) {
      return true;
  }
  return false;
}

const toggleRate = (min, max) => {
  if (selectedRate.value.min_rate.has(min)) {
    selectedRate.value.min_rate.delete(min)
  } else {
    selectedRate.value.min_rate.add(min)
  }

  if (selectedRate.value.max_rate.has(max)) {
    selectedRate.value.max_rate.delete(max)
  } else {
    selectedRate.value.max_rate.add(max)
  }
}

const isDepositOptionSelected = (depositoptions) => {
  let count1 = 0 // 내가 체크 박스 체크
  let count2 = 0 // 내가 체크 박스 체크 안한거
  let rateCount1 = 0; // 금리가 범위 내에 있는 경우
  let rateCount2 = 0; // 금리가 범위 내에 없는 경우
  
  for (const option of depositoptions) {
    if (selectedDIRType.value.has(option.intr_rate_type)) {
      count1++
    }
    else {
      count2++
    }

    if (isRateInRange(option.max_intr_rate)) {
      rateCount1++;
    } else {
      rateCount2++;
    }
  }
  return (selectedDIRType.value.size === 0 || (count1 && !count2)) && (selectedRate.value.min_rate.size === 0 || (rateCount1 && !rateCount2))
}

const isSavingOptionSelected = (savingoptions) => {
  let count1 = 0
  let count2 = 0
  let count3 = 0
  let count4 = 0
  let rateCount1 = 0
  let rateCount2 = 0
  for (const option of savingoptions) {
    if (selectedSIRType.value.has(option.intr_rate_type)) {
      count1++
    }
    else {
      count2++
    }
    if (selectedRType.value.has(option.rsrv_type)) {
      count3++
    }
    else {
      count4++
    }
    if (isRateInRange(option.max_intr_rate)) {
      rateCount1++
    } else {
      rateCount2++
    }
  }
  if (selectedSIRType.value.size > 0 && selectedRType.value.size === 0) {
    return (selectedSIRType.value.size === 0 || (count1 && !count2)) &&
           (selectedRate.value.min_rate.size === 0 || (rateCount1 && !rateCount2));
  }
  
  if (selectedSIRType.value.size === 0 && selectedRType.value.size > 0) {
    return (selectedRType.value.size === 0 || (count3 && !count4)) &&
           (selectedRate.value.min_rate.size === 0 || (rateCount1 && !rateCount2));
  }
  
  if (selectedSIRType.value.size > 0 && selectedRType.value.size > 0) {
    return (selectedSIRType.value.size === 0 || (count1 && !count2)) &&
           (selectedRType.value.size === 0 || (count3 && !count4)) &&
           (selectedRate.value.min_rate.size === 0 || (rateCount1 && !rateCount2));
  }
  
  if (selectedSIRType.value.size === 0 && selectedRType.value.size === 0) {
    return ((selectedSIRType.value.size === 0 || (count1 && !count2)) ||
            (selectedRType.value.size === 0 || (count3 && !count4))) &&
           (selectedRate.value.min_rate.size === 0 || (rateCount1 && !rateCount2));
  }
}

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
  width: 200px;
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

/* 변경된 부분 시작 */
.cards-container {
  display: grid;
  grid-template-columns: repeat(5, 1fr); /* 한 줄에 5개씩 */
  gap: 30px;
  justify-items: center;
  width: 90%;
  padding-left: 40px;
  padding-top: 50px;
}

.card {
  background-color: white;
  border: 1px solid #ddd;
  border-radius: 8px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
  padding: 20px;
  width: 250px;
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

.cat {
  width: 400px;
  margin-top: 100px;
  margin-left: 600px
}

.cat-container {
  display: flex;
  justify-content: center;
  align-items: center;
  height: 400px; /* 이미지 높이에 맞게 조절 */
}
</style>