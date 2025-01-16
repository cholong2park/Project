<template>
  
  <div class="article-list-container">
    <!-- <div class="title-container">제목</div> -->
    <div class="divider"></div>
    <!-- <h3>게시글 목록</h3> -->
    <ArticleListItem 
      v-for="article in store.articles"
      :key="article.id"
      :article="article"
    />
  </div>
</template>

<script setup>
import { useCounterStore } from '@/stores/counter'
import ArticleListItem from '@/components/ArticleListItem.vue'

const store = useCounterStore()
</script>

<style scoped>


.article-list-container {
  max-width: 900px; 
  margin: 30px auto; /* 가운데 정렬합니다. */
  box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.1); /* 그림자 효과 추가 */
  /* padding: 10px; */
  border: 1px solid lightgrey;
  /* border-radius: 20px; */
}

.title-container {
  max-width: 800px; 
  margin: 13px auto; /* 가운데 정렬합니다. */
  padding-top: 15px;
}

.divider {
  width: 100%; /* 전체 너비 */
  height: 1px; /* 선의 높이 */
  /* background-color: #ccc;  회색 선 */
  margin-bottom: 10px; /* 선 아래 간격 */
}


</style>
