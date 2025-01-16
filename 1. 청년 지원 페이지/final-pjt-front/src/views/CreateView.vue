<template>
  <div class="container">
    
    <h1>게시글 작성</h1>
  
    <form @submit.prevent="createArticle">
      <div class="form-group">
        <label for="title" class="form-label">제목</label>
        <input type="text" v-model.trim="title" id="title" class="input-title" />
      </div>
      <div class="form-group">
        <label for="content" class="form-label">내용</label>
        <textarea v-model.trim="content" id="content" class="textarea-content"></textarea>
      </div>
      <input type="submit" value="등록하기" class="submitt-button" />
    </form>
  </div>
</template>

<script setup>
import axios from 'axios';
import { ref } from 'vue';
import { useCounterStore } from '@/stores/counter';
import { useRouter } from 'vue-router';

const store = useCounterStore();
const title = ref(null);
const content = ref(null);
const router = useRouter();

const createArticle = function () {
  axios({
    method: 'post',
    url: `${store.API_URL}/api/v1/articles/`,
    data: {
      title: title.value,
      content: content.value,
    },
    headers: {
      Authorization: `Token ${store.token}`,
    },
  })
    .then((response) => {
      router.push({ name: 'ArticleView' });
    })
    .catch((error) => {
      console.log(error);
    });
};
</script>

<style scoped>
.container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
}

/* 제목 입력칸과 내용 입력칸을 띄우기 (간격 생성) */
.form-group {
  margin-bottom: 10px;
}

/* 제목: 내용: 글씨를 입력칸의 위쪽에 위치하도록 만들기 */
.form-label {
  display: block;
  margin-bottom: 5px;
  font-weight: bold;
}

/* 제목 입력칸 크기 */
.input-title {
  width: 500px;
  height: 50px;
  font-size: 18px;
  padding: 10px;
  margin-bottom: 20px;
  border: 2px solid #215a80; /* 파란색 테두리 */
  border-radius: 5px; /* 테두리 둥글게 */
  /* background-color: rgb(245, 245, 245) */
}

/* 내용 입력칸 크기 */
.textarea-content {
  width: 500px;
  height: 300px;
  font-size: 16px;
  padding: 10px;
  border: 2px solid #215a80; /* 파란색 테두리 */
  border-radius: 5px; /* 테두리 둥글게 */
}

/* 버튼 태그에 스타일 입히기 */
.submitt-button {
  background-color: #226a9b; 
  color: white ; 
  padding: 10px 15px ; 
  border: none ; 
  border-radius: 5px ; 
  cursor: pointer ; 
  font-size: 14px ; 
  align-self: center ; 
  margin-top: 20px ; 
  margin-left: 220px;
}

/* 버튼 태그에 마우스를 올렸을 때 색깔 바꾸기 */
.submitt-button:hover {
  color: rgb(238, 238, 238);
}

h1 {
  padding-top: 40px;
  margin-bottom: 40px;
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
</style>
