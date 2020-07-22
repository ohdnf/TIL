# Vuex 시작하기

```
npm install vuex
```

## Store

```js
// main.js
import Vue from "vue";
import App from "./App.vue";

import { store } from "./store";

new Vue({
    el: '#app',
    store,
    render: h => h(App)
})
```

---

## State

> 컴포넌트 간에 공유할 data 속성

```js
// store.js
import Vue from 'vue'
import Vuex from 'vuex'

Vue.use(Vuex)

export const store = new Vuex.Store({
    state: {
        counter: 0
    }
})
```

```vue
<!-- App.vue -->
<template>
    <div id="app">
        {{ $store.state.counter }}
    </div>
</template>

<script>
export default {}
</script>

<style>
</style>
```

---

## Getters

Vuex의 공유 데이터(state) 변경을 각 컴포넌트가 아닌 Vuex에서 수행하도록 하는 로직

```js
// store.js
import Vue from 'vue'
import Vuex from 'vuex'

Vue.use(Vuex)

export const store = new Vuex.Store({
    state: {
        counter: 0
    },
    getters: {
        getCounter(state) {
            return state.counter
        }
    }
})
```

### mapGetters

```vue
<template>
    <div id="app">
        {{ counter }}
    </div>
</template>

<script>
import { mapGetters } from 'vuex'

export default {
    computed: mapGetters({
        parentCounter : 'getCounter' // getCounter 는 Vuex 의 getters 에 선언된 속성 이름
    }),

    // 이 경우엔 template에서 counter가 아닌 getCounter로 선언된 속성을 사용
    computed: {
        ...mapGetters([
            'getCounter'
        ]),
}
</script>

<style>

</style>
```

## Mutations

Vuex의 데이터(state) 값을 변경하는 로직

### Getters와 차이점

1. 인자를 받아 Vuex에 넘겨줄 수 있다.
2. `computed`가 아닌 `methods`에 등록

### Actions와 차이점

1. Mutations는 동기적 로직을 정의
2. Actions는 비동기적 로직을 정의

> Mutations의 성격상 안에 **정의한 로직들이 순차적으로 일어나야 각 컴포넌트의 반영 여부를 제대로 추적**할 수 있다.

### 등록

```js
// store.js
import Vue from 'vue'
import Vuex from 'vuex'

Vue.use(Vuex)

export const store = new Vuex.Store({
    state: {
        counter: 0
    },
    getters: {
        getCounter(state) {
            return state.counter
        }
    },
    mutations: {
        addCounter(state, payload) {
            return state.counter++
        }
    }
})
```

### 사용

```vue
<template>
    <div id="app">
        {{ counter }}
    </div>
</template>

<script>
import { mapMutations } from 'vuex'

export default {
    methods: {
        addCounter() {
            // commit을 이용하여 mutations 이벤트를 호출, 직접 접근이 불가
            this.$store.commit('addCounter')
        }
    },
    // 코드 가독성 높이기
    ...mapMutations({
        'addCounter'
        // customNameCounter: 'addCounter'      // 메서드명을 다르게 쓰고 싶을 때
    })
}
</script>

<style>

</style>
```

---

## Actions

- `Mutations`에는 순차적인 로직들만 선언하고 `Actions`에는 비순차적 또는 비동기 처리 로직들을 선언
- `Mutations`는 여러 컴포넌트가 관여하는 한 데이터를 효율적으로 관리하는 것을 목표로 함
    - 비동기 처리 로직 등이 포함되면 같은 값에 대해 여러 컴포넌트에서 변경을 요청했을 때, 변경 순서 파악이 어려움
- `setTimeout()`이나 서버와의 HTTP 통신 처리같이 결과를 받아올 타이밍이 예측되지 않는 로직은 `Actions`에 선언

### 등록

```js
// store.js
export const store = new Vuex.Store({
  // ...
  mutations: {
    addCounter: function (state, payload) {
      return state.counter++
    }
  },
  actions: {
    addCounter: function (context) {
      // commit 의 대상인 addCounter 는 mutations의 메서드를 의미한다.
      return context.commit('addCounter')
    }
  }
})
```
> 상태 변화를 추적하기 위해 결국 mutations의 메서드를 호출(commit)하는 구조. `context` 객체는 store 인스턴스에 있는 메소드나 속성들을 `context.commit()`과 같은 형태로 mutations를 실행하거나, 또는 `context.state`나 `context.getters`와 같은 형태로 `state`와 `getters`에 접근할 수 있게 해준다.

```js
// store.js
export const store = new Vuex.Store({
  actions: {
    getServerData: function (context) {
      return axios.get("sample.json").then(function() {
        // ...
      })
    },
    delayFewMinutes: function (context) {
      return setTimeout(function () {
        commit('addCounter')
      }, 1000);
    }
  }
})
```
> HTTP GET 요청이나 `setTimeout()`과 같은 비동기 처리 로직들은 actions에서 선언

### 사용

actions를 사용할 때는 `dispatch()`를 사용

```js
// App.vue
methods: {
  // Mutations 를 이용할 때
  addCounter() {
    this.$store.commit('addCounter');
  }
  // Actions 를 이용할 때
  addCounter() {
    this.$store.dispatch('addCounter');
  }
},
```

인자 넘기기

```html
<!-- by 와 duration 등의 여러 인자 값을 넘길 경우, 객체안에 key - value 형태로 여러 값을 넘길 수 있다 -->
<button @click="asyncIncrement({ by: 50, duration: 500 })">Increment</button>
```

```js
export const store = new Vuex.Store({
  actions: {
    // payload 는 일반적으로 사용하는 인자 명
    asyncIncrement: function (context, payload) {
      return setTimeout(function () {
        context.commit('increment', payload.by)
      }, payload.duration);
    }
  }
})
```
