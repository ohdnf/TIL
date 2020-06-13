<template>
  <div class="todo-view">
    <input v-model="newTodo" type="text" @keypress.enter="writeTodo">
    <button @click="writeTodo">할 일 등록</button>
  </div>
</template>

<script>
export default {
  name: 'TodoInput',
  data() {
    return {
      newTodo: null
    }
  },
  methods: {
    writeTodo() {
      if (this.newTodo.trim()) {
        this.$emit('create', {
          id: Date.now(),
          goal: this.newTodo,
          isCompleted: false
        })
      }
      this.newTodo = null
    }
  }
}
</script>

<style>

</style>