import piniaPluginPersistedstate from 'pinia-plugin-persistedstate'
import { createApp } from 'vue'
import { createPinia } from 'pinia'
import App from './App.vue'
import router from './router'

// 카카오Map API
import { useKakao } from 'vue3-kakao-maps'
useKakao('f63b69ad191331325056ef210a7b5d49', ['clusterer', 'services', 'drawing'])

const app = createApp(App)
const pinia = createPinia()

pinia.use(piniaPluginPersistedstate)
app.use(pinia)

app.use(router)
app.mount('#app')
