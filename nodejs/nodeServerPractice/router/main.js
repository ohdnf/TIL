const express = require('express')
const app = express()
const router = express.Router()

router.get('/', function(req, res) {
  res.sendFile(__dirname + '/public/main.html')
})

module.exports = router;  // 다른 파일에서 main.js를 사용할 수 있게 exports 처리