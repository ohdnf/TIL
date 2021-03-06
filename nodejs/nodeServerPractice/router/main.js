const express = require('express')
const app = express()
const router = express.Router()

router.get('/', function(req, res) {
  // session에 저장된 user 접근: req.user
  const id = req.user;
  if (!id) res.render('login.ejs')
  console.log('main js loaded', req.user)
  // res.sendFile(__dirname + '/public/main.html')
  res.render('main.ejs', {id: id})
})

module.exports = router;  // 다른 파일에서 main.js를 사용할 수 있게 exports 처리