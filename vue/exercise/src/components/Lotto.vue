<template>
  <div>
    <h1>Lotto</h1>
    <button @click="getLotto">숫자생성</button>
    <p>{{ lotto }}</p>
  </div>
</template>

<script>
import _ from 'lodash'

export default {
  data() {
    return {
      lotto: [],
    }
  },
  methods: {
    getLotto() {
      // random으로 로또 번호 생성
      this.lotto = _.sampleSize(_.range(1, 46), 6)
    }
  }
}
</script>

<style>

</style>