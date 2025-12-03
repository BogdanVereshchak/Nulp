// Virtual File System State
export const files = {
    'main.scad': `// Demo Cube\ncolor("#bd93f9");\ntranslate([0,0,0]);\ncube([20,20,20]);\n\n// Add a sphere\ntranslate([25,0,0]);\ncolor("#50fa7b");\nsphere(r=12);`,
    'parts.scad': `// Cylinder Part\ncolor("#ff79c6");\ncylinder(h=30, r=5);`,
    'config.scad': `// Configuration\n$fn = 60;`
};

export const state = {
    activeFile: 'main.scad',
    openFiles: ['main.scad', 'parts.scad']
};

export function createFile(name) {
    if (files[name]) return false;
    files[name] = "// New OpenSCAD File\n";
    return true;
}

export function deleteFile(name) {
    if (Object.keys(files).length <= 1) return false;
    delete files[name];
    
    // Manage tabs
    const idx = state.openFiles.indexOf(name);
    if(idx > -1) state.openFiles.splice(idx, 1);
    
    // Switch active file if needed
    if(state.activeFile === name) {
        state.activeFile = Object.keys(files)[0];
        if(!state.openFiles.includes(state.activeFile)) {
            state.openFiles.push(state.activeFile);
        }
    }
    return true;
}

export function saveCurrentFile(content) {
    if(state.activeFile && files[state.activeFile] !== undefined) {
        files[state.activeFile] = content;
    }
}