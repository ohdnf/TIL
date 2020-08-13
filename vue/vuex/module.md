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

## 상위 모듈에 있는 `dispatch`나 `commit`을 실행하기

### `rootState` 사용

`dispatch("path1/actionA", payload, { root: true });`