<template>
  <div class="home">
    <img alt="Vue logo" src="../assets/logo.png">
    <MyComponent />
  </div>
</template>

<script>
// @ is an alias to /src
import MyComponent from '@/components/MyComponent.vue'

export default {
  name: 'Home',
  components: {
    MyComponent
  }
}
</script>
