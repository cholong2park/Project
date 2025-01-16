<template>

  <div class="signup-container">
    <h1 class="signup-title">Sign Up to <span class="highlight">WWB</span></h1>
    <h4 class="signup-subtitle">가입을 통해 더 다양한 서비스를 만나보세요!</h4>
    <form @submit.prevent="signUp" class="signup-form">
      <div class="form-group">
        <label for="username">아이디</label>
        <input
          type="text"
          v-model.trim="username"
          id="username"
          class="input-field"
        />
      </div>
      <div class="form-group">
        <label for="password1">비밀번호</label>
        <input
          type="password"
          v-model.trim="password1"
          id="password1"
          class="input-field"
        />
      </div>
      <div class="form-group">
        <label for="password2">비밀번호 확인</label>
        <input
          type="password"
          v-model.trim="password2"
          id="password2"
          class="input-field"
        />
      </div>
      <input type="submit" class="submit-button" value="가입하기" />
    </form>
  </div>
</template>

<script setup>
import { ref } from "vue";
import { useCounterStore } from "@/stores/counter";

const username = ref(null);
const password1 = ref(null);
const password2 = ref(null);
const store = useCounterStore();

const signUp = function () {
  const payload = {
    username: username.value,
    password1: password1.value,
    password2: password2.value,
  };
  store.signUp(payload);
};
</script>

<style scoped>

.signup-container {
  max-width: 400px;
  margin: 80px auto;
  padding: 20px;
  border-radius: 8px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.2);
  background-color: #fcfcfc;
}

.signup-title {
  text-align: center;
  font-family: "Nunito", sans-serif;
  font-size: 2em;
}

.signup-subtitle {
  text-align: center;
  color: #0d3e72;
  font-size: 17px;
  font-weight: bold;
}

.signup-form {
  display: flex;
  flex-direction: column;
  padding-top: 20px;
}

.form-group {
  width: 100%;
}

.form-group label {
  font-weight: bold;
  display: block;
  margin-bottom: 5px;
}

.input-field {
  width: calc(100% - 22px); /* Adjust width to consider padding and border */
  padding: 10px;
  font-size: 14px;
  border: 1px solid #ccc;
  border-radius: 4px;
  transition: border-color 0.3s;
}

.input-field:focus {
  border-color: rgb(0, 118, 190);
  outline: none;
}

.submit-button {
  width: 100px; /* Adjust width as needed */
  margin: 10px auto;
  padding: 10px 15px;
  font-size: 16px;
  color: #fff;
  background-color: rgb(0, 118, 190);
  border: none;
  border-radius: 4px;
  cursor: pointer;
  transition: background-color 0.3s;
}

.submit-button:hover {
  background-color: #0056b3;
}

.highlight {
  color: #0056b3;
  font-size: 36px;
}
</style>
