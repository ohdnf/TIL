<template>
  <div class="parent">
    <h1>Parent</h1>
    <Child @loveYouToo="parentMsg='Love you more!'" :propFromParent="parentMsg"/>
  </div>
</template>

<script>
import Child from '@/components/Child.vue'

export default {
  name: 'Parent',
  data() {
    return {
      parentMsg: 'I love you'
    }
  },
  components: {
    Child
  }
}
</script>

<style>
  .parent {
    border: 3px solid red;
    margin: 3px;
    padding: 3px;
  }
</style>