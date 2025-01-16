<template>
  <div class="login-container">
    <!-- <h1 class="login-title">Sign In to <span class="highlight">WWB</span></h1> -->
    <h1 class="login-title">Login to your account</h1>
    <form @submit.prevent="logIn" class="login-form">
      <div class="form-group">
        <label for="username" class="login-label">아이디</label>
        <input type="text" v-model.trim="username" id="username" class="login-input">
      </div>
      <div class="form-group">
        <label for="password" class="login-label">비밀번호</label>
        <input type="password" v-model.trim="password" id="password" class="login-input">
      </div>
      <input type="submit" class="login-submit" value="로그인">
    </form>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import { useCounterStore } from '@/stores/counter'

const username = ref(null)
const password = ref(null)
const store = useCounterStore()

const logIn = function () {
  const payload = {
    username: username.value,
    password: password.value
  }
  store.logIn(payload)
}
</script>

<style scoped>
.login-container {
  max-width: 400px;
  margin: 80px auto;
  padding: 20px;
  padding-bottom: 30px;
  border-radius: 8px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.2);
  background-color: #fcfcfc;
}

.login-title {
  text-align: center;
  font-family: "Nunito", sans-serif;
  font-size: 2em;
  padding-bottom: 10px;
}

.login-form {
  display: flex;
  flex-direction: column;
  align-items: center; /* Center items horizontally */
}

.form-group {
  width: 100%;
  margin-bottom: 15px;
  text-align: left;
}

.login-label {
  font-weight: bold;
  display: block;
  margin-bottom: 5px;
}

.login-input {
  width: calc(100% - 22px); /* Adjust width to consider padding and border */
  padding: 10px;
  font-size: 14px;
  border: 1px solid #ccc;
  border-radius: 4px;
  transition: border-color 0.3s;
}

.login-input:focus {
  border-color: rgb(0, 118, 190);
  outline: none;
}

.login-submit {
  padding: 10px 15px; 
  font-size: 16px; 
  color: #fff;
  background-color: rgb(0, 118, 190);
  border: none;
  border-radius: 4px;
  cursor: pointer;
  transition: background-color 0.3s;
  margin-top: 15px;
  width: auto; 
}

.login-submit:hover {
  background-color: #0056b3;
}
</style>
