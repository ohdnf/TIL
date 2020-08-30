const express = require('express')
const app = express()

app.listen(3000, function() {})

app.use(express.static('public'))
app.use(express.json())
app.use(express.urlencoded({extended: true}))

app.set('view engine', 'ejs')

// URL Routing
app.get('/', function(req, res) {
  res.sendFile(__dirname + '/public/main.html')
})
app.post('/search', function(req, res) {
  // res.render('search.ejs', {'q': req.body.q})
  // console.log(req.body)
  const response = {result: 200, q: req.body.query}
  res.json(response)
})