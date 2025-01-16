<template>
    <div class="search-container">
      <h1>Search</h1>
      <div class="search-box">
        <input
          type="text"
          placeholder="검색어를 입력하세요"
          class="search-input"
          v-model="query"
        />
        <button class="search-button" @click="search">
          <img
            class="search-icon"
            src="https://s3.ap-northeast-2.amazonaws.com/cdn.wecode.co.kr/icon/search.png"
          />
        </button>
      </div>
      <br />
      <div v-if="showResult" class="information">
        <div v-for="(item, index) in items" :key="index">
          <div>
            <p>{{ item.description }}</p>
            <a :href="item.link">네이버 백과서전으로 이동하기</a>
          </div>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref } from "vue";
  import axios from "axios";
  
  const query = ref(""); // 사용자 입력값을 저장할 ref
  const items = ref([]); // 검색에 대한 3개의 결과값을 저장할 ref
  const showResult = ref(false); // 결과를 표시할지 여부를 나타내는 ref
  
  const search = () => {
    // Axios 요청을 보내는 함수
    axios({
      method: "GET",
      url: "/v1/search/encyc.json",
      headers: {
        "X-Naver-Client-Id": "qx3qugw1GZonHuNBvq6o",
        "X-Naver-Client-Secret": "BcDZJetQ8R",
      },
      params: {
        query: query.value, // 사용자가 입력한 검색어
        display: 1,
      },
    })
      .then(function (response) {
        console.log(response.data.items);
        for (let item of response.data.items) {
          // HTML 태그 제거
          item.description = item.description.replace(/<\/?b>/g, "");
  
          // 첫 번째 '.' 앞의 글자 추출
          // const firstPeriodIndex = item.description.indexOf(".");
          // item.description = item.description.substring(0, firstPeriodIndex + 1); // 마침표 포함
        }
        items.value = response.data.items;
        showResult.value = true; // 결과를 표시
      })
      .catch(function (error) {
        // 에러 처리
        console.error(error);
      });
  };
  </script>
  
  <style scoped>
  .search-container {
    display: flex; /* 요소들을 flexbox 컨테이너로 설정 */
    flex-direction: column; /* 요소들을 세로로 정렬 */
    align-items: center; /* 자식 요소들을 중앙에 정렬 */
  }
  
  .search-box {
    display: flex; 
    width: 270px; 
    align-items: center; 
    border: 2px solid rgb(0, 118, 190); 
    border-radius: 25px; /* 둥근 모서리를 위해 radius 값 크게 설정 */
    overflow: hidden; /* 컨테이너에서 넘치는 내용을 숨김 */
  }
  
  .search-input {
    border: none; /* 입력 필드의 기본 테두리 제거 */
    padding: 10px 15px; /* 입력 필드 내부 여백 설정 */
    outline: none; /* 포커스 시 외곽선 제거 */
    flex-grow: 1; /* 남은 공간을 모두 차지하게 설정 */
    border-top-left-radius: 25px; /* 좌측 상단 모서리를 둥글게 설정 */
    border-bottom-left-radius: 25px; /* 좌측 하단 모서리를 둥글게 설정 */
  }
  
  .search-button {
    height: 20px; 
    border: 1px; 
    padding: 20px 15px; /* 버튼 내부 여백 설정 */
    display: flex; /* 버튼을 flexbox 컨테이너로 설정 */
    align-items: center; /* 자식 요소들을 중앙에 정렬 */
    justify-content: center; /* 자식 요소들을 중앙에 정렬 */
    cursor: pointer; /* 마우스를 올렸을 때 포인터 커서 표시 */
    border-top-right-radius: 25px; 
    border-bottom-right-radius: 25px; 
  }
  
  .search-icon {
    width: 23px; 
    height: 23px; 
    opacity: 70%; 
  }
  
  .information {
    width: auto;
    height: auto;
    background-color: rgb(245, 244, 244);
    padding: 10px;
    display: flex;
    flex-direction: column;
    align-items: center;
  }
  
  </style>
  