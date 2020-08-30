const express = require("express")
const app = express()
const router = require('./router/index')

app.listen(3000, function() {
  console.log("Express.js starts on port 3000")
})

// Static 디렉토리 설정
app.use(express.static('public'))

// Express.js의 Built-in body-parser 사용
app.use(express.json())
app.use(express.urlencoded( {extended : true } ));

// Router 모듈화
app.use(router)

// View Engine에 ejs를 사용
app.set('view engine', 'ejs')
