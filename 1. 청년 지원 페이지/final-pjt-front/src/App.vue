<template>
  <div class="navbar">
    <div class="nav-links-left">
      <RouterLink to="/" class="nav-link home-link">WWB</RouterLink>
      <RouterLink to="/saving" class="nav-link">예금/적금</RouterLink>
      <RouterLink to="/youngman" class="nav-link">청년지원제도</RouterLink>
      <RouterLink to="/exchange" class="nav-link">환율계산기</RouterLink>
      <RouterLink to="/bank" class="nav-link">은행위치</RouterLink>
      <button class="nav-link nav-button" @click="toggleClick">금융용어검색</button>
      <RouterLink to="/recommend" class="nav-link">추천서비스</RouterLink>
      <RouterLink :to="{ name: 'ArticleView' }" class="nav-link">자유게시판</RouterLink>
    </div>
    <div class="nav-links-right">
      <RouterLink :to="{ name: 'SignUpView' }" class="nav-link2 signup-button">Sign Up</RouterLink>
      <RouterLink :to="{ name: 'LogInView' }" class="nav-link2 login-button">Login</RouterLink>
      <button class="nav-link2 logout-button" @click="logOut">Logout</button>



    </div>
  </div>
  <!-- 메인 콘텐츠 영역, 검색창이 열리면 with-search 클래스를 추가 -->
  <div :class="{ 'main-content': true, 'with-search': clickVisible }">
    <RouterView></RouterView>
  </div>
  <!-- 검색창, clickVisible이 true일 때만 표시 -->
  <div v-if="clickVisible" class="click-container">
    <ClickSearch />
  </div>
</template>

<script setup>
import { ref } from 'vue';
import { RouterLink, RouterView, useRouter } from 'vue-router';
import ClickSearch from '@/components/ClickSearch.vue';
import { useCounterStore } from './stores/counter';

const toggleClick = () => {
  clickVisible.value = !clickVisible.value; // clickVisible의 값을 토글
};
const clickVisible = ref(false); // clickVisible을 false로 초기화

const store = useCounterStore()
const router = useRouter()
const logOut = function () {
  store.token = null
  router.push({ name: 'LogInView' }); // 로그인 페이지로 리다이렉트
}
</script>

<style scoped>
.navbar {
  width: 100%; /* 네비게이션 바의 너비를 화면 너비의 100%로 설정 */
  background-color: rgb(0, 118, 190); /* 배경색을 파란색으로 설정 */
  padding: 1rem; /* 패딩 설정 */
  display: flex; /* flexbox 레이아웃 사용 */
  position: fixed; /* 화면 위에 고정 */
  top: 0; /* 화면 위쪽에 위치 */
  left: 0; /* 화면 왼쪽에 위치 */
  z-index: 1000; /* 다른 요소 위에 표시되도록 설정 */
  justify-content: space-between;
  align-items: center;
  flex-wrap: wrap; /* 네비게이션 바가 작은 화면에서 줄바꿈 되도록 설정 */
}

.nav-link {
  color: white; /* 텍스트 색상을 흰색으로 설정 */
  text-decoration: none; /* 밑줄 없음 */
  font-size: 1rem;
  font-weight: bold;
  margin: 10px 50px;
  outline: none; /* 포커스 시 아웃라인 없음 */
}
.nav-link2 {
  color: white; /* 텍스트 색상을 흰색으로 설정 */
  text-decoration: none; /* 밑줄 없음 */
  font-size: 1rem;
  font-weight: bold;
  outline: none; /* 포커스 시 아웃라인 없음 */
  
}

.nav-links-left {
  gap: 20px;
  flex-wrap: wrap; /* 작은 화면에서 줄바꿈을 허용 */
}
.nav-links-right {
  margin-right: 30px;
  margin-top: 5px;
  gap: 10px;
  flex-wrap: wrap; /* 작은 화면에서 줄바꿈을 허용 */
}
.home-link {
  font-size: 1.5rem; /* 폰트 크기를 더 크게 설정 */
  color: #ffffcf; /* 눈에 띄는 연한 노란색으로 설정 */
}

.nav-link:hover,
.home-link:hover {
  color: #d4eeff; /* 호버 시 색상 변경 */
  transition: color 0.3s; /* 부드러운 색상 전환 효과 */
  outline: none; /* 포커스 시 아웃라인 없음 */
  background-color: rgb(0, 118, 190); /* 호버 시 배경색 설정 */
}

.nav-var-center {
  position: center; /* 위치 설정 */
}

.nav-button {
  background: none; /* 배경 없음 */
  border: none; /* 테두리 없음 */
  cursor: pointer; /* 커서 모양을 포인터로 설정 */
}

.main-content {
  width: auto;
  /* display: flex; flexbox 레이아웃 사용 */
  flex-direction: column; /* 세로 방향으로 정렬 */
  justify-content: flex-start; /* 수직 방향으로 페이지 상단에 정렬 */
  /* align-items: center; 수평 방향으로 중앙에 정렬 */
  transition: margin-right 0.1s; /* 부드러운 전환 효과 */
  margin-top: 80px; /* 네비게이션 바 아래에 위치하도록 설정 */
  padding: 10px; /* 패딩 설정 */
}

.with-search {
  margin-right: 400px; /* .click-container의 너비만큼 우측 마진 추가 */
}

.click-container {
  width: 400px;
  height: auto; /* 높이 auto로 설정 */
  position: fixed; /* 화면에 고정 */
  top: 90px; /* 상단에서 80px 아래에 위치 */
  right: 20px; /* 오른쪽에서 20px 안쪽에 위치 */
  background-color: white; /* 배경색을 흰색으로 설정 */
  overflow-y: auto; /* 세로 스크롤을 자동으로 표시하도록 설정 */
  border: 1px solid #ccc; /* 테두리 설정 */
  border-radius: 5px; /* 테두리를 둥글게 설정 */
  padding: 15px 30px 30px 30px; /* 패딩 설정 */
  z-index: 999; /* 다른 요소 위에 표시되도록 설정 */
}

.signup-button,
.login-button {
  background-color: rgba(221, 221, 221, 0.2);
  color: rgb(255, 255, 255);
  border: 0px solid rgb(0, 118, 190);
  border-radius: 5px;
  padding: 5px 10px;
  font-size: 12px;
  /* text-transform: uppercase; */
  transition: background-color 0.3s, color 0.3s;
  margin: 0 4px;
}

.logout-button {
  background-color: rgba(221, 221, 221, 0.2);
  color: rgb(255, 255, 255);
  border: 0px solid rgb(0, 118, 190);
  border-radius: 5px;
  padding: 6.5px 10px;
  font-size: 12px;
  transition: background-color 0.3s, color 0.3s;
  margin: 0 4px;
}

.signup-button:hover,
.login-button:hover,
.logout-button:hover {
  background-color: rgb(226, 226, 226, 0.3);
  color: white;
}


</style>
