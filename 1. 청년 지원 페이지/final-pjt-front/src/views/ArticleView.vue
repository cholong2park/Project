<template>
  <h1 class="main-title">자유게시판</h1>
  <div class="main-container">
    <div class="button-container">
      <RouterLink :to="{ name: 'CreateView' }" class="write-button">
        글쓰기
      </RouterLink>
    </div>
  </div>

  
    <ArticleList />

</template>

<script setup>
import { onMounted } from 'vue';
import { useCounterStore } from '@/stores/counter';
import { RouterLink } from 'vue-router';
import ArticleList from '@/components/ArticleList.vue';

const store = useCounterStore();

onMounted(() => {
  store.getArticles();
});
</script>

<style scoped>
.main-container {
  width: 75%;
  display: flex;
  flex-direction: column;
  align-items: center;
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
  content: '';
  position: absolute;
  bottom: 0;
  left: 50%;
  transform: translateX(-50%);
  border-bottom: 3px solid #3498db;
  width: 210px;
}
.write-button {
  display: inline-block;
  margin: 10px auto;
  padding: 10px 20px;
  font-size: 16px;
  font-weight: bold;
  color: white;
  background-color: #3498db;
  border: none;
  border-radius: 5px;
  text-align: center;
  text-decoration: none;
  transition: background-color 0.3s, transform 0.3s;
}

.write-button:hover {
  /* background-color: #4f5ff0; */
  /* transform: translateY(-1px); */
  color: rgb(220, 238, 253);
}

.button-container {
  align-self: end; /* 버튼 컨테이너를 오른쪽으로 정렬합니다. */
  margin-right: 20px; /* 오른쪽 여백을 추가합니다. */
}
</style>
