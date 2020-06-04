# Component

```js
Vue.component('component_name', {
    template: `
        <div>
            Component template should contain exactly one root element
        </div>
    `,
    data() {
        return {
            fresh_data: 'data function returns a fresh data object for each component'
        }
    }
})
```

## props

props make a component to access outside data, for instance, parent component's data.

```js
Vue.component('child_comp', {
    props: {
        message: {
            type: String,
            required: true,
            default: "Hi"
        }
    },
    template: `<div>{{message}}</div>`,
    data () { ... }
})

const app = new Vue({
    el: "#app",
    data: {
        message: "Wussup"
    }
})
```

```html
<child_comp :message="message"></child_comp>
```

## emit

emit deliever child comp's event to parent comp

```html
<product :premium="premium"
        @add-to-cart="updateCart"></product>
```


```js
Vue.component('product', {
    ...

    methods: {
        addToCart() {
            this.$emit('add-to-cart')
        }
    }
})

const app = new Vue({
    ...
    data: {
        ...
        cart: 1
    },
    methods: {
        updateCart() {
            this.cart += 1
        }
    }
})
```