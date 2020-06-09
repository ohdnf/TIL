<template>
  <div class="ping">
    <h1>Ping</h1>
    <input @keyup.enter="sentToPong" v-model="inputText" type="text">
  </div>
</template>

<script>
export default {
  name: 'Ping',
  data() {
    return {
      inputText: ''
    }
  },
  methods: {
    sentToPong() {
      // this.$router.push(`/pong?message=${this.inputText}`)
      // 아래 코드는 위 코드와 동일
      this.$router.push({
        name: 'Pong',
        query: {
          message: this.inputText
        }
      })
    }
  }
}
</script>

<style>

</style>