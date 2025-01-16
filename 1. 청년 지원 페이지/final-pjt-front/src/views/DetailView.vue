<template>
  <h1 class="main-title">자유게시판</h1>
  <div style="padding-top: 50px;">
  <div class="article-detail">
    <div v-if="article">
      <div class="article-details-container">
        <h1 class="title">{{ article.title }}</h1>
        <div class="article-detail-item">
          <p class="detail-content">{{ article.content }}</p>
        </div>
        <div class="article-detail-item">
          <p class="detail-date-content">{{ formatDate(article.created_at) }}</p>
        </div>
      </div>
      <button @click="deleteArticle(article.id)" class="delete-button">Delete</button>
      <!-- <button @click="editArticle(article.id)" class="edit-button">Edit</button> -->
    </div>
  </div>
  </div>
  <div class="comments-section">
          <h2 class="comments-title">댓글</h2>
          <div class="comments-wrapper">
            <div v-if="comments.length" class="comments-list">
              <div v-for="comment in comments" :key="comment.id" class="comment-item">
                {{ comment }}
                <!-- <p class="comment-author">{{ comment.author }}</p>
                <p class="comment-content">{{ comment.content }}</p>
                <p class="comment-date">{{ formatDate(comment.created_at) }}</p> -->
                <!-- <button @click="deleteComment(comment.id)" class="delete-comment-button">삭제</button> -->
              </div>
            </div>
            <div v-else class="no-comments">댓글이 없습니다.</div>
            <div class="add-comment">
              <textarea v-model="commentContent" placeholder="댓글을 작성하세요" rows="5" style="width: 780px"></textarea>
              <button @click="createComment">등록</button>
            </div>
          </div>
        </div> 
</template>

<script setup>
import axios from 'axios';
import { onMounted, ref } from 'vue';
import { useCounterStore } from '@/stores/counter';
import { useRoute, useRouter } from 'vue-router';

const store = useCounterStore();
const route = useRoute();
const article = ref(null);

const router = useRouter();
const deleteArticle = (id) => {
  axios({
    method: 'delete',
    url: `${store.API_URL}/api/v1/articles/${route.params.id}/`,
  })
    .then((response) => {
      router.push({ name: 'ArticleView' });
    })
    .catch((error) => {
      console.log(error);
    });
};

const formatDate = (dateString) => {
  const date = new Date(dateString);
  const year = date.getFullYear();
  const month = String(date.getMonth() + 1).padStart(2, '0');
  const day = String(date.getDate()).padStart(2, '0');
  const hours = String(date.getHours()).padStart(2, '0');
  const minutes = String(date.getMinutes()).padStart(2, '0');
  const seconds = String(date.getSeconds()).padStart(2, '0');
  return `${year}-${month}-${day} ${hours}:${minutes}:${seconds}`;
};

onMounted(() => {
  axios({
    method: 'get',
    url: `${store.API_URL}/api/v1/articles/${route.params.id}/`,
  })
    .then((response) => {
      article.value = response.data;
    })
    .catch((error) => {
      console.log(error);
    });
});

const comments = ref([])
// 빈 문자열로 받기
const commentContent = ref('')

const createComment = function () {
  // 장고에 요청을 보내는 것
  // http 요청을 axios로 만든 것
  axios({
    method: 'post',
    // counter.js에 있는 API_URL을 가져오려고 하는 것
    // store에 있는 API_URL를 불러오는 것
    // axios를 통해서 요청을 보낼 주소
    url: `${store.API_URL}/api/v1/articles/${route.params.id}/commentcreate/`,
    headers: {
      // 로그인 된 상태를 확인
      // counter.js에 있는 store가 인증된 사용자인지 확인하는 것
      Authorization: `Token ${store.token}`
    },
    data: {
      content: commentContent.value
    }
  })
    .then((res) => {
      comments.value.push(res.data.content)
      console.log(res.data);
      commentContent.value = ''
      setTimeout(() => {
        window.scrollTo({ left: 0, top: document.body.scrollHeight+100, behavior: "smooth" });
      }, 200)
      
    })
    .catch((err) => {
      console.log(err)
    })
}

// const deleteComment = function (commentId) {
//   const answer = window.confirm('정말 삭제하시겠습니까?')

//   if (answer) {
//     axios({
//       method: 'delete',
//       url: `${store.API_URL}/posts/${postId}/comments/${commentId}/`,
//       headers: {
//         Authorization: `Token ${store.token}`
//       }
//     })
//       .then((res) => {
//         comments.value = comments.value.filter((comment) => comment.id != commentId)
//       })
//       .catch((err) => {
//         console.log(err)
//       })
//   }
// }
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
  position: relative;
}

.main-title::after {
  content: "";
  position: absolute;
  bottom: 0;
  left: 50%;
  transform: translateX(-50%);
  border-bottom: 3px solid #3498db;
  width: 210px;
}

.title {
  margin-left: 50px;
}

.article-detail {
  max-width: 800px;
  margin: 0 auto;
  padding: 5px;
  padding-bottom: 20px;
  background-color: #f0f0f0; /* 회색 배경 추가 */
  border-radius: 10px; /* 모서리를 둥글게 */
}

.article-details-container {
  border: 3px solid #ffffff;
  padding: 10px;
  margin: 20px 20px; /* 내부 컨테이너의 여백을 줄여서 회색 박스가 더 크게 보이도록 설정 */
  background-color: white; /* 내부 컨테이너의 배경색 */
  border-radius: 5px; /* 모서리를 둥글게 */
  border: 1px solid lightgrey;
}

.article-detail-item {
  display: flex;
  margin-bottom: 10px;
  flex-direction: column;
}

.detail-date-content {
  align-self: flex-end;
  font-size: 14px;
  color: #777;
  margin-right: 20px;
}

.detail-content {
  flex: 3;
  padding: 10px 10px;
  margin: 0px 40px;
}

.delete-button {
  background-color: #6b6b6b;
  color: white;
  border: none;
  border-radius: 5px;
  padding: 10px 15px;
  margin-left: 690px;
  font-size: 16px;
  transition: background-color 0.3s, transform 0.3s;
}

.delete-button:hover {
  color: #f1f1f1;
}

.edit-button {
  background-color: #3498db;
  color: white;
  border: none;
  border-radius: 5px;
  padding: 10px 15px;
  margin-left: 10px;
  font-size: 16px;
  transition: background-color 0.3s, transform 0.3s;
}

.edit-button:hover {
  color: #f1f1f1;
}

.comments-title {
  text-align: left;
  margin-left: 540px; /* 왼쪽 여백 추가 */
  font-size: 24px;
  font-weight: bold;
  color: #2c3e50;
}

.comments-section {
  margin-top: 50px;
  text-align: center; /* 댓글 섹션을 가운데 정렬하기 위해 */
}

.comments-wrapper {
  display: inline-block; /* 댓글 목록을 가운데로 정렬하기 위해 */
  text-align: left; /* 댓글 내용을 왼쪽 정렬하기 위해 */
  max-width: 800px;
  width: 100%;
}

.no-comments {
  margin-top: 20px;
  color: #999;
}

.add-comment {
  margin-top: 20px;
  text-align: center; /* 댓글 입력란과 버튼을 가운데 정렬하기 위해 */
}

.comments-list {
  margin-top: 20px;
}

.comment-item {
  border: 1px solid #ddd;
  border-radius: 5px;
  padding: 10px;
  margin-bottom: 10px;
}

.comment-item:last-child {
  margin-bottom: 0;
}

.comment-author {
  font-weight: bold;
  color: #333;
}

.comment-content {
  margin-top: 5px;
  color: #666;
}

.comment-date {
  font-size: 12px;
  color: #999;
}

.add-comment {
  margin-top: 20px;
}

.add-comment textarea {
  width: calc(100% - 20px);
  padding: 10px;
  border: 1px solid #ddd;
  border-radius: 5px;
  resize: vertical;
}

.add-comment button {
  margin-top: 10px;
  padding: 10px 20px;
  background-color: #3498db;
  color: #fff;
  border: none;
  border-radius: 5px;
  cursor: pointer;
}

.add-comment button:hover {
  background-color: #2980b9;
}
</style>
