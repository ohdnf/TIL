<template>
  <div class="child">
    <h1>Child</h1>
    <p>{{ propFromParent }}</p>
    <button @click="loveSignal">I love you too!</button>
  </div>
</template>

<script>
export default {
  name: 'Child',
  props: {
    propFromParent: String,
  },
  methods: {
    loveSignal() {
      this.$emit('loveYouToo')
    }
  }
}
</script>

<style>
  .child {
    border: 3px solid blue;
    margin: 3px;
    padding: 3px;
  }
</style>