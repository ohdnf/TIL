const express = require('express')
const app = express()
const router = express.Router()
const path = require('path')

const main = require('./main')
const email = require('./email')
const join = require('./join')

// URL Routing
router.get('/', function(req, res) {
  res.sendFile(path.join(__dirname, "../public/main.html"))
})

router.use('/main', main)
router.use('/email', email)
router.use('/join', join)

module.exports = router;