import Vue from 'vue'
import VueRouter from 'vue-router'
// import Home from '../views/Home.vue'
import Contact from '../views/Contact.vue'
import Ping from '../views/Ping.vue'
import Pong from '../views/Pong.vue'
import Parent from '../views/Parent.vue'
import TodoView from '../views/TodoView.vue'

Vue.use(VueRouter)

  const routes = [
  {
    path: '/',
    name: 'TodoView',
    component: TodoView
  },
  {
    path: '/about',
    name: 'About',
    // route level code-splitting
    // this generates a separate chunk (about.[hash].js) for this route
    // which is lazy-loaded when the route is visited.
    component: () => import(/* webpackChunkName: "about" */ '../views/About.vue')
  },
  {
    path: '/contact/:name',
    name: 'Contact',
    component: Contact
  },
  {
    path: '/ping',
    name: 'Ping',
    component: Ping
  },
  {
    path: '/pong',
    name: 'Pong',
    component: Pong
  },
  {
    path: '/parent',
    name: 'Parent',
    component: Parent
  }
]

const router = new VueRouter({
  mode: 'history',
  base: process.env.BASE_URL,
  routes
})

export default router
