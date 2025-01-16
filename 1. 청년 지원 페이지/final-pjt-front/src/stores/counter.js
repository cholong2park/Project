import { ref, computed } from 'vue'
// 계층이 깊어지면 힘들기 때문에 공통으로 쓰일 액션, 함수, 데이터, 상태 등을 중앙저장소에 저장한 후에 어떤 컴포넌트에서든지 직접 접근해서 가져올 수 있게끔 하는 것
import { defineStore } from 'pinia'
import axios from 'axios'
import { useRouter } from 'vue-router'

export const useCounterStore = defineStore('counter', () => {
  const policies = ref([])
  const selectpolicy = ref({})
  const deposits = ref([])
  const savings = ref([])
  const deposit_saving = ref([]) // 추천 금융 상품들 저장될 것
  const policy = ref([]) // 추천 지원제도들 저장될 것

  // 청년 정책 가져오기
  const getPolicies = function () {
    axios({
      method: 'GET',
      url: `${API_URL}/policies/`
    })
    .then(response => {
      // console.log(response)
      // console.log(response.data)
      policies.value = response.data
    })
    .catch(error => {
      console.log(error)
    })
  }
  
  // 정기예금 가져오기
  const getDeposits = function () {
    axios({
      method: 'GET',
      url: `${API_URL}/deposits_savings/deposits/`
    })
    .then(response => {
      // console.log(response)
      // console.log(response.data)
      deposits.value = response.data
    })
    .catch(error => {
      console.log(error)
    })
  }

  // 적금 가져오기
  const getSavings = function () {
    axios({
      method: 'GET',
      url: `${API_URL}/deposits_savings/savings/`
    })
    .then(response => {
      // console.log(response)
      // console.log(response.data)
      savings.value = response.data
    })
    .catch(error => {
      console.log(error)
    })
  }

  
  const articles = ref([])
  const API_URL = 'http://127.0.0.1:8000'
  // token 이 null인 것은 로그아웃
  const token = ref(null)
  const isLogin = computed(() => {
    if (token.value === null) {
      return false
    } else {
      return true
    }
  })
  const router = useRouter()

  const getArticles = function () {
    axios({
      method: 'get',
      url: `${API_URL}/api/v1/articles/`,
      headers: {
        Authorization: `Token ${token.value}`
      }
    })
      .then(response => {
        articles.value = response.data
      })
      .catch(error => {
        console.log(error)
      })
  }

  const signUp = function (payload) {
    // 1. 사용자 입력 데이터를 받아
    // const username = payload.username
    // const password1 = payload.password1
    // const password2 = payload.password2
    const { username, password1, password2 } = payload

    // 2. axios로 django에 요청을 보냄
    axios({
      method: 'post',
      url: `${API_URL}/accounts/signup/`,
      data: {
        // username: username,
        // password1: password1,
        // password2: password2
        username, password1, password2
      }
    })
     .then((response) => {
       console.log('회원가입 성공!')
       const password = password1
       logIn({ username, password })
     })
     .catch((error) => {
       console.log(error)
     })
  }

  const logIn = function (payload) {
    // 1. 사용자 입력 데이터를 받아
    const { username, password } = payload
    // 2. axios로 django에 요청을 보냄
    axios({
      method: 'post',
      url: `${API_URL}/accounts/login/`,
      data: {
        username, password
      }
    })
      .then((response) => {
        // console.log('로그인 성공!')
        // console.log(response)
        // console.log(response.data.key)
        // 3. 로그인 성공 후 응답 받은 토큰을 저장
        token.value = response.data.key
        router.push({ name : 'ArticleView' })
      })
      .catch((error) => {
        console.log(error)
      })
  }


  return { policies, deposits, savings, API_URL, getPolicies, getDeposits, getSavings, articles, getArticles, signUp, logIn, token, isLogin, selectpolicy, deposit_saving, policy }
}, {persist: true})
