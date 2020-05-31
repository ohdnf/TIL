<template>
  <div id="app">
    <button>Index</button>
    <button>Lotto</button>
    <button>Lunch</button>
    <Index/>
    <Lotto/>
  </div>
</template>

<script>
import Index from './components/Index'
import Lotto from './components/Lotto'

export default {
  name: 'App',
  components: {
    Index,
    Lotto,
  }
}
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
</style>
