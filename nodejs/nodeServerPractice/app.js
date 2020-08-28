var express = require("express")
var app = express()

app.listen(3000, function() {
  // console.log("Hello, Express server on port 3000")
})

// Static 디렉토리 설정
app.use(express.static('public'))

// Express.js의 Built-in body-parser 사용
app.use(express.json())
app.use(express.urlencoded( {extended : true } ));

// View Engine에 ejs를 사용
app.set('view engine', 'ejs')

// URL Routing
app.get('/', function(req, res) {
  // res.send("<h1>Hello, world!</h1>")
  res.sendFile(__dirname + "/public/main.html")
})

app.post('/email_post', function(req, res) {
  // get: req.param('email')
  // console.log(req.body)
  res.render('email.ejs', {'email': req.body.email})
})

app.post('/ajax_email', function(req, res) {
  // console.log(req.body.email)
  var responseData = {result: 'OK', email: req.body.email}
  res.json(responseData)
})