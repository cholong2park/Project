<template>
  <div class="container">
    <div class="text-container">
      <h1 class="title">We When Benefit</h1>
      <h3 class="subtitle">
        최고의 <span class="highlight">예적금상품</span>과
        <span class="highlight">청년지원제도</span>를 한 눈에!
      </h3>
    </div>
    <div class="router-container">
      <RouterLink to="/saving" class="nav-link ">
        예금/적금
        <span class="arrow">⟶</span>
      </RouterLink>
      <RouterLink to="/youngman" class="nav-link">
        청년지원제도
        <span class="arrow">⟶</span>
      </RouterLink>
    </div>
  </div>
</template>

<script setup></script>

<style scoped>

.container {
  display: flex;
  flex-direction: column; /* 요소를 세로 배치 */
  justify-content: center; /* 수평 가운데 정렬 */
  align-items: center; /* 수직 가운데 정렬 */
  padding-top: 200px;
  background-size: cover; /* 배경 이미지가 화면을 덮도록 조절*/
  background-repeat: no-repeat; /* 배경 이미지 반복하지 않음 */
}

.text-container {
  display: flex;
  flex-direction: column; /* 요소를 세로 배치 */
  justify-content: center; /* 텍스트 수직 중앙 정렬 */
  align-items: center;
  text-align: center;
  background: rgba(255, 255, 255, 0.8); /* 텍스트 배경 투명도 설정 */
  padding: 20px;
  border-radius: 10px;
}

.router-container {
  display: flex;
  justify-content: center; /* 수평 가운데 정렬 */
  align-items: center; /* 수직 가운데 정렬 */
  gap: 50px; /* 링크 사이 간격 */
  margin-top: 20px; /* 텍스트 컨테이너와의 간격 조정 */
}

.title {
  color: rgb(137, 192, 220);
  font-size: 80px; /* 제목의 크기를 줄임 */
  margin: 0; /* 제목의 외부 마진 없애기 */
  padding-bottom: 30px;
}

.subtitle {
  font-size: 25px;
  margin-top: 0px; /* 부제목과 제목의 간격 조정 */
}

.highlight {
  color: rgb(0, 118, 190);
  font-size: 26px;
}

.nav-link {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  width: 200px;
  height: 100px; /* 크기를 고정 */
  padding: 20px; /* 내부 여백 설정 */
  font-size: 20px;
  font-weight: bold;
  color: white;
  background-color: rgb(57, 148, 204); /* 버튼 배경색 설정 */
  text-decoration: none; /* 링크 밑줄 없애기 */
  border-radius: 5px; /* 버튼 모서리 둥글게 설정 */
  transition: background-color 0.3s ease; /* 호버 효과를 위한 전환 설정 */
}

.nav-link .arrow {
  margin-top: 10px; /* 화살표와 텍스트 간격 */
  font-size: 20px;
}
.nav-link:hover,
.nav-link:focus {
  outline: none;
  border-color: #f5f5f5;
  box-shadow: 0 0 10px rgba(116, 227, 255, 0.8);
  transform: translateY(-3px);

}

</style>
