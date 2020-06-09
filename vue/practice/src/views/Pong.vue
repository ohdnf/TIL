<template>
  <div class="pong">
    <h1>Pong</h1>
    <p>{{ messageFromPing }}</p>
  </div>
</template>

<script>
export default {
  name: 'Pong',
  data() {
    return {
      messageFromPing: this.$route.query.message
    }
  }
}
</script>

<style>

</style>