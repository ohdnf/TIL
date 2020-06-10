<template>
  <div class="todoview">
    <input v-model="newTodo" type="text">
    <button @click="newTodoSignal">할 일 등록</button>
  </div>
</template>

<script>
export default {
  name: 'TodoInput',
  data() {
    return {
      newTodo: ''
    }
  },
  methods: {
    newTodoSignal() {
      if (this.newTodo) {
        this.$emit('submit-todo', this.newTodo)
      }
    }
  }
}
</script>

<style>

</style>