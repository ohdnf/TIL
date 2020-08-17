# module

## `store`에 module 등록하기

### 폴더 구조

```
src/
├─App.vue
└─store/
    ├─auth.js
    ├─stepper.js
    └─store.js
```

### `store.js`

```js
import Vue from "vue";
import Vuex from "vuex";

import { auth } from "./auth";
import { stepper } from "./stepper";

Vue.use(Vuex);

export default new Vuex.Store({
  modules: {
    auth,
    stepper
  },

    ...

```

## module로 등록한 데이터 사용하기

### `mutations`에 접근하기

- 직접 접근: `this.$store.commit('모듈명/mutation명')`
- `mapMutations` 활용

```js
methods: {
    ...mapMutations({
        // this.getStyles()를 this.$store.commit('item/getSelectedVideoSytle')에 매핑
        getStyles: 'stepper/getSelectedVideoStyle'
    })
}
```

### `actions`에 접근하기

- 직접 접근: `this.$store.dispatch('모듈명/action명')`
- `mapActions` 활용

```js
methods: {
    ...mapActions({
        // this.getEditors()를 this.$store.dispatch('stepper/getSortedEditors')에 매핑
        getEditors: 'stepper/getSortedEditors'
    })
}
```

### `getters`에 접근하기

- 직접 접근: `this.$store.getters['모듈명/getter명']`
- `mapGetters` 활용: `computed` 영역에서 호출

```js
computed: {
    ...mapGetters({
        selectedCategories: 'stepper/getSelectedCategories'
    })
}
```

## 상위 모듈에 있는 `getters`나 `mutations`, `actions`를 `dispatch`나 `commit`을 사용해 실행하기

### `rootState` 사용

`dispatch("path1/actionA", payload, { root: true });`

## 같은 모듈에 있는 `getters`를 다른 `getters`에서 사용하기

```js
// store.stepper.js
export const stepper = {
  namespaced: true,
  state: {
      ...
  },
  getters: {
    // Stepper 세부사항 불러오기
    getSelectedVideoType(state) {
      ...
    },
    getSelectedVideoStyle(state) {
      ...
    },
    
    ...

    isChecked(state, getters) {
        // stepNum으로 넘겨준 인자를 step 화살표 함수로 사용
        return step => {
            switch (step) {
                case 0:
                    if (!getters.getSelectedVideoType) {
                        return true
                    }
                        ...
                case 1:
                    if (!getters.getSelectedVideoStyle) {
                        ...
                    }
            },
        }
    },
```

```vue
// Stepper.vue
<template>
...
</template>

<script>
export default {
    ...
        
    methods: {
        changeStep(stepNum) {
            // getters 뒤에 (parameter) 형태로 넘겨줄 수 있다.
            if (this.$store.getters['stepper/isChecked'](stepNum)) {
                this.currentStep = stepNum
            }
        }
    },
}
</script>

...
```