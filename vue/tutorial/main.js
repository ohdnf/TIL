const app = new Vue({
  el: "#app",
  data: {
    product: "Socks",
    image: "./assets/vmSocks-green.jpg",
    alt: "blue socks",
    naverShop: "https://shopping.naver.com/",
    inventory: 100,
    inStock: true,
    details: ["80% cotton", "20% polyester", "Gender-neutral"],
    variants: [
      {
        variantId: 2234,
        variantColor: "green",
        variantImage: "./assets/vmSocks-green.jpg"
      },
      {
        variantId: 2235,
        variantColor: "blue",
        variantImage: "./assets/vmSocks-blue.jpg"
      }
    ],
    sizes: [
      {
        sizeId: 3345,
        sizeName: "small"
      },
      {
        sizeId: 3346,
        sizeName: "medium"
      },
      {
        sizeId: 3347,
        sizeName: "large"
      },
    ],
    cart: 0
  },
  methods: {
    addToCart() {
      this.cart += 1
    },
    updateProduct(variantImage) {
      this.image = variantImage
    },
    clearTheCart() {
      this.cart = 0
    }
  }
})