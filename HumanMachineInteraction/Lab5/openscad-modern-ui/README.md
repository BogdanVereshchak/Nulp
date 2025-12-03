# OpenSCAD Modern UI

## Overview
The OpenSCAD Modern UI project provides a functional user interface for rendering 3D models using OpenSCAD-like code. It leverages Three.js for 3D rendering and offers a modern, responsive design.

## Project Structure
```
openscad-modern-ui
├── src
│   ├── index.html          # Main HTML entry point
│   ├── scripts             # JavaScript files for application logic
│   │   ├── app.js         # Initializes the application and manages logic
│   │   ├── ui.js          # UI-related functions and components
│   │   ├── engine.js      # OpenSCADEngine class for 3D rendering
│   │   └── parser.js      # Parses OpenSCAD-like code into 3D objects
│   ├── styles              # CSS styles for the application
│   │   └── main.css       # Main stylesheet
│   └── assets              # Assets such as fonts
│       └── fonts          # Font files for consistent typography
├── package.json            # npm configuration file
└── README.md               # Project documentation
```

## Setup Instructions
1. Clone the repository:
   ```
   git clone <repository-url>
   cd openscad-modern-ui
   ```

2. Install dependencies:
   ```
   npm install
   ```

3. Open `src/index.html` in a web browser to view the application.

## Usage
- Use the sidebar to navigate between different panels (Explorer, Search, Git).
- Create, open, and manage files using the file explorer.
- Write OpenSCAD-like code in the editor and render 3D models in real-time.

## Features
- Responsive design with a modern UI.
- 3D rendering using Three.js.
- File management capabilities (create, delete, open).
- Syntax highlighting and line numbering in the code editor.
- Console for output and error messages.

## Contributing
Contributions are welcome! Please submit a pull request or open an issue for any suggestions or improvements.