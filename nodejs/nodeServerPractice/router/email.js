const express = require('express')
const app = express()
const router = express.Router()

// DB Setting
const mysql = require("mysql")

const connection = mysql.createConnection({
  host: 'localhost',
  port: 3306,
  user: 'root',
  password: 'q1w2e3r4',
  database: 'jsman'
})

connection.connect()

// URL Routing
router.post('/form', function(req, res) {
  // get: req.param('email')
  // console.log(req.body)
  res.render('email.ejs', {'email': req.body.email})
})

router.post('/ajax', function(req, res) {
  // console.log(req.body.email)
  const email = req.body.email
  const responseData = {}

  const query = connection.query('select name from user where email="' + email + '"', function(err, rows) {
    if (err) throw err
    if (rows[0]) {
      // console.log(rows[0].name)
      responseData.result = 200
      responseData.name = rows[0].name
    } else {
      // console.log('none: ' + rows[0])
      responseData.result = 400
      responseData.name = ""
    }
    // 비동기로 응답
    res.json(responseData)
  })
})

module.exports = router;
