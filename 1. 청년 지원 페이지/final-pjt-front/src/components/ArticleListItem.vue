<template>
  <div class="article-list">
    <!-- <h5>게시글 번호: {{ article.id }}</h5> -->

    <RouterLink
      :to="{ name: 'DetailView', params: { id: article.id } }"
      class="title-link"
    >
      <p class="title">
        <span class="title-color">제목</span> : {{ article.title }}
      </p>
    </RouterLink>
    <p class="content" v-if="article.content.length > 60">
      {{ article.content.substring(0, 60) }}...
    </p>
    <p class="content" v-else>
      {{ article.content }}
    </p>

    <!-- <RouterLink
      class="detail-link"
      :to="{ name: 'DetailView', params: { id: article.id }}"
    >
      상세보기
    </RouterLink> -->
  </div>
</template>

<script setup>
import { RouterLink } from 'vue-router';

defineProps({
  article: Object,
});
</script>

<style scoped>
.article-list {
  border-bottom: 1px solid #ccc; /* 목록 항목 간 구분선 */
  border-bottom-width: 100%;
  padding-bottom: 20px; /* 구분선과 항목 내용 사이 간격 */
}
.detail-link {
  display: inline-block;
  padding: 4px 8px;
  background-color: #4caf50; /* 초록색 배경 */
  color: white; /* 흰색 글자 */
  text-decoration: none; /* 밑줄 없앰 */
  border-radius: 5px; /* 둥근 테두리 */
  transition: background-color 0.3s, transform 0.3s; /* 부드러운 전환 효과 */
}

.title {
  padding-left: 40px;
  padding-top: 10px;
  color: #272727;
  font-weight: bold;
}
.title-link {
  text-decoration: none; /* 밑줄 제거 */
  cursor: pointer; /* 커서를 포인터로 변경하여 클릭 가능한 것으로 표시 */
}

.title:hover {
  color: #000000;
  text-decoration: underline;
}

.title-color {
  color: rgb(17, 48, 150);
  font-weight: bold;
}

.content {
  padding-left: 40px;
  padding-top: 10px;
  color: #000;
}
</style>
