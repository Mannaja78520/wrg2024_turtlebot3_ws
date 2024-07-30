const express = require('express');
const path = require('path');
const cors = require('cors');
const app = express();
const port = process.env.PORT || 3000;

// Configure CORS
app.use(cors({
  origin: 'http://localhost:8080', // Replace with the URL of your web app if different
  methods: 'GET,POST,PUT,DELETE,OPTIONS',
  allowedHeaders: 'Content-Type,Authorization',
}));

// Serve static files from the 'build' directory
app.use(express.static(path.join(__dirname, 'build')));

// Route all other requests to the React app
app.get('*', (req, res) => {
  res.sendFile(path.join(__dirname, 'build', 'index.html'));
});

// Start the server
app.listen(port, () => {
  console.log(`Server is running on http://localhost:${port}`);
});
