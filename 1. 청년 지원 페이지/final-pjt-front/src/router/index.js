import { createRouter, createWebHistory } from 'vue-router'
import HomeView from '@/views/HomeView.vue'
import SavingsView from '@/views/SavingsView.vue'
import ExchangeView from '@/views/ExchangeView.vue'
import BankLocationView from '@/views/BankLocationView.vue'
import YoungmanNoticeView from '@/views/YoungmanNoticeView.vue'
// import NoticeBoardView from '@/views/NoticeBoardView.vue'
import RecommendAIView from '@/views/RecommendAIView.vue'
import RecommendAIResultView from '@/views/RecommendAIResultView.vue'
import YoungmanNoticeDetailView from '@/views/YoungmanNoticeDetailView.vue'
import SavingsDetailView from '@/views/SavingsDetailView.vue'

import ArticleView from '@/views/ArticleView.vue'
import DetailView from '@/views/DetailView.vue'
import CreateView from '@/views/CreateView.vue'
import SignUpView from '@/views/SignUpView.vue'
import LogInView from '@/views/LogInView.vue'


const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/',
      name: 'home',
      component : HomeView,
     },
    {
      path: '/saving',
      name: 'saving',
      component: SavingsView,
    },
    {
      path: '/saving/:name',
      name: 'savingdetail',
      component: SavingsDetailView,
    },
    {
      path: '/exchange',
      name: 'exchange',
      component: ExchangeView,
    },
    {
      path: '/bank',
      name: 'bank',
      component: BankLocationView,
    },
    {
      path: '/youngman',
      name: 'youngman',
      component: YoungmanNoticeView,
    },
    {
      path: '/youngman/:id',
      name: 'youngmandetail',
      component: YoungmanNoticeDetailView,
    },
    // {
    //   path: '/notice',
    //   name: 'notice',
    //   component: NoticeBoardView,
    // },
    {
      path: '/recommend',
      name: 'recommend',
      component: RecommendAIView,
    },
    {
      path: '/recommend/result',
      name: 'recommendresult',
      component: RecommendAIResultView
    },
    {
      path: '/articles',
      name: 'ArticleView',
      component: ArticleView
    },
    {
      path: '/articles/:id',
      name: 'DetailView',
      component: DetailView
    },
    {
      path: '/create',
      name: 'CreateView',
      component: CreateView
    },
    {
      path: '/signup',
      name: 'SignUpView',
      component: SignUpView
    },
    {
      path: '/login',
      name: 'LogInView',
      component: LogInView
    }
  ]
})

import { useCounterStore } from '@/stores/counter'


router.beforeEach((to, from) => {
  const store = useCounterStore()
  // 인증되지 않은 사용자는 메인 페이지에 접근 할 수 없음
  if (to.name === 'ArticleView' && store.isLogin === false) {
    window.alert('로그인이 필요합니다.')
    return { name: 'LogInView' }
  }

  // 인증된 사용자는 회원가입과 로그인 페이지에 접근 할 수 없음
  if ((to.name === 'SignUpView' || to.name === 'LogInView') && (store.isLogin === true)) {
    window.alert('이미 로그인되어 있습니다.')
    return { name: 'ArticleView' }
  }
})


export default router
