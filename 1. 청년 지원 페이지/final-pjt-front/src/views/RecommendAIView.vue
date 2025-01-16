<template>
  <div title-container>
    <h1 class="main-title">추천 서비스</h1>
    <div class="table-container">
      <p class="description">"당신에게 딱 맞는 상품을 추천해드려요!"</p>
      <div class="robot-image-container">
        <img class="robot-image" src="@/assets/robot.png" >
      </div>
      <table class="table">
        <tr>
          <td>
            <div>
              <p>• 은행으로 적합하다고 생각되는 색깔은?!</p>
            </div>
          </td>
        </tr>
        <tr>
          <td>
            <div class="checkbox-group">
              <input type="radio" name="BankColor" @change="toggleBankColor('red')" />빨 
              <input type="radio" name="BankColor" @change="toggleBankColor('orange')" />주
              <input type="radio" name="BankColor" @change="toggleBankColor('yellow')" />노 
              <input type="radio" name="BankColor" @change="toggleBankColor('green')" />초
              <input type="radio" name="BankColor" @change="toggleBankColor('blue')" />파
              <input type="radio" name="BankColor" @change="toggleBankColor('indigo')" />남
              <input type="radio" name="BankColor" @change="toggleBankColor('purple')" />보
            </div>
          </td>
        </tr>
        <tr class="empty-space"></tr>
        <tr>
          <td>
            <div>
              <p>• 전자기기 잘 다루시나요?</p>
            </div>
          </td>
        </tr>
        <tr>
          <td>
            <div class="checkbox-group">
              <input type="radio" name="Adopter" @change="toggleAdopter('yes')"/> 얼리어답터~
              <input type="radio" name="Adopter" @change="toggleAdopter('no')" /> 아뇨
            </div>
          </td>
        </tr>
        <tr class="empty-space"></tr>
        <tr>
          <td>
            <div>
              <p>• 충동 구매 자주 하시나요?</p>
            </div>
          </td>
        </tr>
        <tr>
          <td>
            <div class="checkbox-group">
              <input type="radio" name="Purchase" @change="togglePurchase('yes')" /> 네!
              <input type="radio" name="Purchase" @change="togglePurchase('no')"/> 아니요!
            </div>
          </td>
        </tr>
        <template v-if="purchaseAnswer">
          <tr class="empty-space"></tr>
          <tr>
            <td>
              <div>
                <p>• 현재 돈 좀 있으신가요?</p>
              </div>
            </td>
          </tr>
          <tr>
            <td>
              <div class="checkbox-group">
                <input type="radio" name="Money"  @change="toggleMoney('yes')" /> 네!
                <input type="radio" name="Money"  @change="toggleMoney('no')"/> 아니요...
              </div>
            </td>
          </tr>
        </template>
        <tr class="empty-space"></tr>
        <tr>
          <td>
            <div>
              <p>• 나의 거주지는 어디인가요?</p>
            </div>
          </td>
        </tr>
        <tr>
          <td>
            <div class="checkbox-group">
              <input type="radio" name="Region" @change="toggleRegion('003002001')" /> 서울
              <input type="radio" name="Region" @change="toggleRegion('003002008')" /> 경기도
              <input type="radio" name="Region" @change="toggleRegion('003002009')" /> 강원도 
              <input type="radio" name="Region" @change="toggleRegion('003002011')" />충청남도
              <input type="radio" name="Region" @change="toggleRegion('003002010')" />충청북도<br>
              <input type="radio" name="Region" @change="toggleRegion('003002013')" /> 전라남도
              <input type="radio" name="Region" @change="toggleRegion('003002012')" /> 전라북도 
              <input type="radio" name="Region" @change="toggleRegion('003002015')" /> 경상남도
              <input type="radio" name="Region" @change="toggleRegion('003002014')" /> 경상북도
              <input type="radio" name="Region" @change="toggleRegion('003002016')" /> 제주도
            </div>
          </td>
        </tr>
        <tr class="empty-space"></tr>
        <tr>
          <td>
            <div>
              <p>• 현재 관심사가 뭐에요?</p>
            </div>
          </td>
        </tr>
        <tr>
          <td>
            <div class="checkbox-group">
              <input type="radio" name="Interest" @change="toggleInterest('023010')" /> 취업하고 싶어요 ㅠㅠ (일자리)<br>
              <input type="radio" name="Interest" @change="toggleInterest('023020')" /> 살만한 곳을 찾을래요! (주거)<br>
              <input type="radio" name="Interest" @change="toggleInterest('023030')" /> 유용한 것들을 배워보고 싶어요 (교육)<br>
              <input type="radio" name="Interest" @change="toggleInterest('023040')" /> 지친 나에게 휴식이 필요해요! (복지/문화)<br>
              <input type="radio" name="Interest" @change="toggleInterest('023050')" /> 새로운 활동과 대회에 나가보고 싶어요! (참여/권리)<br>
            </div>
          </td>
        </tr>
      </table>
    </div>
    <div class="button-container">
      <RouterLink :to="{ name: 'recommendresult' }">
        <button class="result-button" @click="submitAnswer()">결과확인</button>
      </RouterLink>
    </div>
  </div>
</template>

<script setup>
import { ref, computed } from 'vue'
import { useCounterStore } from '@/stores/counter'
import { RouterLink } from 'vue-router'

// 색깔별 은행
const red_bank = ['부산은행', '경남은행', '오에스비저축은행', '조은저축은행', '에스비아이저축은행', '다올저축은행', '고려저축은행', '유니온상호저축은행', '엠에스상호저축은행', '남양저축은행', '부림저축은행', '평택저축은행', '페퍼저축은행', '청주저축은행', '한성저축은행', '센트럴저축은행', '대아상호저축은행', '머스트삼일저축은행', '대원상호저축은행', '동원제일저축은행', '삼호저축은행', '대신저축은행', '비엔케이저축은행', '웰컴저축은행']
const orange_bank = ['다올저축은행', '모아저축은행', '엠에스상호저축은행', '부림저축은행', '영진저축은행', '세람상호저축은행', '한화저축은행', '조흥저축은행', '진주저축은행', '대신저축은행', '웰컴저축은행', '오케이저축은행']
const yellow_bank = ['국민은행', '농협은행주식회사', '주식회사 카카오뱅크', '다올저축은행', '모아저축은행', '부림저축은행', '엔에이치저축은행', '대신저축은행', '케이비저축은행', '오케이저축은행']
const green_bank = ['한국스탠다드차타드은행', '하나은행', '애큐온저축은행', '디비저축은행', '푸른상호저축은행', '다올저축은행', '국제저축은행', '디에이치저축은행', '금화저축은행', '인천저축은행', '유니온상호저축은행', '안국저축은행', '안양저축은행', '상상인저축은행', '대명상호저축은행', '상상인플러스저축은행', '아산저축은행', '대한저축은행', '진주저축은행', '대신저축은행', '하나저축은행']
const blue_bank = ['우리은행', '한국스탠다드차타드은행', '대구은행', '광주은행', '제주은행', '전북은행', '중소기업은행', '한국산업은행', '신한은행', '수협은행', '토스뱅크 주식회사', '푸른상호저축은행', 'HB저축은행', '에스비아이저축은행', '다올저축은행', '유안타저축은행', '디에이치저축은행', '우리저축은행', '인성저축은행', '삼정저축은행', '융창저축은행', 'CK저축은행', '우리금융저축은행', '오투저축은행', '동양저축은행', '더블저축은행', '드림저축은행', '참저축은행', '솔브레인저축은행', '제이티저축은행', '대신저축은행', '아이비케이저축은행', '제이티친애저축은행', '신한저축은행']
const indigo_bank = ['스카이저축은행', '민국저축은행', '키움예스저축은행', '더케이저축은행', '바로저축은행', '다올저축은행', '인성저축은행', '대백저축은행', '키움저축은행', '세람상호저축은행', '스타저축은행', '스마트저축은행', '한국투자저축은행', '라온저축은행', '드림저축은행', '오성저축은행', '에스앤티저축은행', '대신저축은행', '아이비케이저축은행']
const purple_bank = ['주식회사 케이뱅크', '다올저축은행', '고려저축은행', '흥국저축은행', '부림저축은행', '키움저축은행', '한성저축은행', '진주저축은행', '예가람저축은행', '대신저축은행']

// 영업점인가?!
const go_out = ['영업점,인터넷,스마트폰,전화(텔레뱅킹)', '영업점,인터넷,스마트폰,기타', '영업점,인터넷,스마트폰', '영업점,인터넷', '영업점,스마트폰', '영업점']
const go_phone = ['인터넷,스마트폰,전화(텔레뱅킹)', '인터넷,스마트폰', '인터넷', '영업점,인터넷,스마트폰,전화(텔레뱅킹)', '영업점,인터넷,스마트폰,기타', '영업점,인터넷,스마트폰', '영업점,인터넷', '영업점,스마트폰', '스마트폰', '인터넷,스마트폰,기타', '스마트폰,전화(텔레뱅킹)', '모집인', '']

const store = useCounterStore()

const deposit_saving = ref([])
const policy = ref([])

// 1번 문항
const bank_color = ref('')
const toggleBankColor = (color) => {
  bank_color.value = color
}

// 2번 문항
const early_adopter = ref('')
const toggleAdopter = (answer) => {
  early_adopter.value = answer
}

// 3번 문항
const impulse_purchase = ref('')
const togglePurchase = (answer) => {
  impulse_purchase.value = answer
}

// 3-1번 문항 (답이 no인 경우)
const purchaseAnswer = computed(() => {
  return impulse_purchase.value === 'no' ? 1 : 0
})

const now_money = ref('')
const toggleMoney = (answer) => {
  now_money.value = answer
}

// 4번 문항
const region = ref('')
const toggleRegion = (answer) => {
  region.value = answer
}

// 5번 문항
const interest = ref('')
const toggleInterest = (answer) => {
  interest.value = answer
}

const submitAnswer = () => {
  store.deposit_saving = []
  store.policy = []
  // 은행 색깔 구분
  if (bank_color.value === 'red') {
    deposit_saving.value = deposit_saving.value.concat(store.deposits.filter(deposit => red_bank.includes(deposit.kor_co_nm)))
    deposit_saving.value = deposit_saving.value.concat(store.savings.filter(saving => red_bank.includes(saving.kor_co_nm)))
  } else if (bank_color.value === 'orange') {
    deposit_saving.value = deposit_saving.value.concat(store.deposits.filter(deposit => orange_bank.includes(deposit.kor_co_nm)))
    deposit_saving.value = deposit_saving.value.concat(store.savings.filter(saving => orange_bank.includes(saving.kor_co_nm)))
  } else if (bank_color.value === 'yellow') {
    deposit_saving.value = deposit_saving.value.concat(store.deposits.filter(deposit => yellow_bank.includes(deposit.kor_co_nm)))
    deposit_saving.value = deposit_saving.value.concat(store.savings.filter(saving => yellow_bank.includes(saving.kor_co_nm)))
  } else if (bank_color.value === 'green') {
    deposit_saving.value = deposit_saving.value.concat(store.deposits.filter(deposit => green_bank.includes(deposit.kor_co_nm)))
    deposit_saving.value = deposit_saving.value.concat(store.savings.filter(saving => green_bank.includes(saving.kor_co_nm)))
  } else if (bank_color.value === 'blue') {
    deposit_saving.value = deposit_saving.value.concat(store.deposits.filter(deposit => blue_bank.includes(deposit.kor_co_nm)))
    deposit_saving.value = deposit_saving.value.concat(store.savings.filter(saving => blue_bank.includes(saving.kor_co_nm)))
  } else if (bank_color.value === 'indigo') {
    deposit_saving.value = deposit_saving.value.concat(store.deposits.filter(deposit => indigo_bank.includes(deposit.kor_co_nm)))
    deposit_saving.value = deposit_saving.value.concat(store.savings.filter(saving => indigo_bank.includes(saving.kor_co_nm)))
  } else {
    deposit_saving.value = deposit_saving.value.concat(store.deposits.filter(deposit => purple_bank.includes(deposit.kor_co_nm)))
    deposit_saving.value = deposit_saving.value.concat(store.savings.filter(saving => purple_bank.includes(saving.kor_co_nm)))
  }
  // 영업점 vs 스마트폰
  if (early_adopter.value === 'yes') {
    deposit_saving.value = deposit_saving.value.filter(ds => go_phone.includes(ds.join_way))
  } else {
    deposit_saving.value = deposit_saving.value.filter(ds => go_out.includes(ds.join_way) )
  }
  // 예금 vs 적금 - 1
  if (impulse_purchase.value === 'yes') {
    deposit_saving.value = deposit_saving.value.filter(ds => store.savings.includes(ds))
  } else {
    // 예금 vs 적금 - 2
    if (now_money.value === 'yes') {
      deposit_saving.value = deposit_saving.value.filter(ds => store.deposits.includes(ds))
    } else {
      deposit_saving.value = deposit_saving.value.filter(ds => store.savings.includes(ds))
    }
  }
  
  // 청년지원제도 - 거주지
  if (region.value === '003002001') {
    policy.value = store.policies.filter(policy => policy.polyBizSecd === '003002001')
  } else if (region.value === '003002008') {
    policy.value = store.policies.filter(policy => policy.polyBizSecd === '003002008')
  } else if (region.value === '003002009') {
    policy.value = store.policies.filter(policy => policy.polyBizSecd === '003002009')
  } else if (region.value === '003002011') {
    policy.value = store.policies.filter(policy => policy.polyBizSecd === '003002011')
  } else if (region.value === '003002010') {
    policy.value = store.policies.filter(policy => policy.polyBizSecd === '003002010')
  } else if (region.value === '003002013') {
    policy.value = store.policies.filter(policy => policy.polyBizSecd === '003002013')
  } else if (region.value === '003002012') {
    policy.value = store.policies.filter(policy => policy.polyBizSecd === '003002012')
  } else if (region.value === '003002015') {
    policy.value = store.policies.filter(policy => policy.polyBizSecd === '003002015')
  } else if (region.value === '003002014') {
    policy.value = store.policies.filter(policy => policy.polyBizSecd === '003002014')
  } else {
    policy.value = store.policies.filter(policy => policy.polyBizSecd === '003002016')
  }
  // 청년지원제도 - 관심사
  if (interest.value === '023010') {
    policy.value = policy.value.filter(p => p.polyRlmCd === '023010')
  } else if (interest.value === '023020') {
    policy.value = policy.value.filter(p => p.polyRlmCd === '023020')
  } else if (interest.value === '023030') {
    policy.value = policy.value.filter(p => p.polyRlmCd === '023030')
  } else if (interest.value === '023040') {
    policy.value = policy.value.filter(p => p.polyRlmCd === '023040')
  } else {
    policy.value = policy.value.filter(p => p.polyRlmCd === '023050')
  }
  
  store.deposit_saving = deposit_saving.value.slice(0, 3)
  store.policy = policy.value.slice(0, 3)
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
  width: 200px; /* 직접 밑줄의 길이를 조정합니다. */
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

.robot-image {
  width: 100px;
  height: 110px;
}

/* card 관련 스타일 */
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


@media screen and (max-width: 1400px) {
  .robot-image-container {
    display: none;
  }
}
</style>
