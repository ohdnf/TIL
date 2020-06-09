<template>
  <div class="contact">
    <h1>Call me, {{ name }}</h1>
  </div>
</template>

<script>
export default {
  name: 'Contact',
  data() {
    return {
      name: this.$route.params.name
    }
  }
}
</script>

<style>

</style>