import { OpenSCADEngine } from './engine.js';
import { parseAndRun } from './parser.js';
import { files, state, createFile, deleteFile, saveCurrentFile } from './fileSystem.js';
import * as UI from './ui.js';

let engine;

// --- CORE FUNCTIONS ---

function openFile(filename) {
    saveCurrentFile(document.getElementById('codeEditor').value);
    state.activeFile = filename;
    if(!state.openFiles.includes(filename)) state.openFiles.push(filename);
    
    UI.loadFileIntoEditor(filename);
    UI.renderFileList(openFile);
    UI.renderTabs(openFile, closeTab);
}

function closeTab(filename) {
    const idx = state.openFiles.indexOf(filename);
    if(idx > -1) {
        state.openFiles.splice(idx, 1);
        if(state.activeFile === filename) {
            state.activeFile = state.openFiles[state.openFiles.length - 1] || Object.keys(files)[0];
            if(!state.openFiles.includes(state.activeFile) && Object.keys(files).length > 0) {
                 state.openFiles.push(state.activeFile);
            }
            if(Object.keys(files).length > 0) openFile(state.activeFile);
        }
        UI.renderTabs(openFile, closeTab);
    }
}

function compileAndRender() {
    const code = document.getElementById('codeEditor').value;
    UI.log("Compiling...", "info");
    const start = performance.now();
    try {
        const count = parseAndRun(code, engine);
        const time = ((performance.now() - start) / 1000).toFixed(3);
        UI.log(`Rendering finished. Objects: ${count}. Time: ${time}s`);
        
        const status = document.getElementById('renderStatus');
        status.innerText = `Rendered: ${count} objs in ${time}s`;
        status.style.display = 'block';
        setTimeout(() => status.style.display = 'none', 3000);
    } catch(e) {
        UI.log("Error: " + e.message, "error");
    }
}

// --- INITIALIZATION & EVENT LISTENERS ---

document.addEventListener('DOMContentLoaded', () => {
    // 1. Init UI
    UI.renderFileList(openFile);
    UI.renderTabs(openFile, closeTab);
    UI.loadFileIntoEditor(state.activeFile);
    
    // 2. Init 3D Engine
    engine = new OpenSCADEngine();
    
    // 3. Attach Global Event Listeners (Delegation or ID binding)
    
    // Sidebar Buttons
    document.querySelectorAll('[data-action="switchPanel"]').forEach(btn => {
        btn.addEventListener('click', () => UI.switchPanel(btn.dataset.target));
    });

    // Modals
    document.querySelectorAll('[data-action="openModal"]').forEach(btn => {
        btn.addEventListener('click', () => document.getElementById('modal-' + btn.dataset.target).classList.add('open'));
    });
    document.querySelectorAll('[data-action="closeModal"]').forEach(btn => {
        btn.addEventListener('click', () => document.getElementById('modal-' + btn.dataset.target).classList.remove('open'));
    });

    // Editor Input
    document.getElementById('codeEditor').addEventListener('input', UI.updateLineNumbers);
    
    // 3D Toolbar
    document.getElementById('btn-reset-view').onclick = () => engine.resetView();
    document.getElementById('btn-toggle-grid').onclick = () => engine.toggleGrid();
    document.getElementById('btn-toggle-wire').onclick = () => engine.toggleWireframe();
    document.getElementById('btn-render').onclick = compileAndRender;

    // Explorer Menu
    document.getElementById('explorerMenuBtn').onclick = (e) => {
        e.stopPropagation();
        const menu = document.getElementById('explorerMenu');
        menu.classList.toggle('show');
        document.addEventListener('click', () => menu.classList.remove('show'), {once:true});
    };

    document.getElementById('btn-new-file').onclick = () => {
        const name = prompt("Введіть назву файлу:", "new_design.scad");
        if(name) {
            if(createFile(name)) {
                openFile(name);
                UI.log(`Файл ${name} створено.`, 'info');
            } else {
                alert("Файл вже існує!");
            }
        }
    };

    document.getElementById('btn-del-file').onclick = () => {
        if(confirm(`Видалити файл ${state.activeFile}?`)) {
            if(deleteFile(state.activeFile)) {
                UI.log("Файл видалено.", 'info');
                UI.renderFileList(openFile);
                UI.renderTabs(openFile, closeTab);
                UI.loadFileIntoEditor(state.activeFile);
            } else {
                alert("Не можна видалити останній файл!");
            }
        }
    };

    // Settings
    document.getElementById('themeSelect').onchange = (e) => {
        const theme = e.target.value;
        document.body.setAttribute('data-theme', theme);
        engine.setTheme(theme);
        UI.log(`Тему змінено на ${theme}`, 'info');
    };
    
    document.getElementById('fontSizeInput').onchange = (e) => {
        document.getElementById('codeEditor').style.fontSize = e.target.value + 'px';
    };

    // Console
    document.getElementById('consoleHeader').onclick = () => document.getElementById('consolePanel').classList.toggle('collapsed');
    document.getElementById('btn-clear-log').onclick = (e) => {
        e.stopPropagation();
        document.getElementById('consoleLog').innerHTML = '';
    };

    // Search filter
    document.getElementById('searchInput').oninput = (e) => {
        const query = e.target.value.toLowerCase();
        const container = document.getElementById('searchResults');
        container.innerHTML = '';
        if(!query) return;
        Object.keys(files).forEach(f => {
            if(f.toLowerCase().includes(query)) {
                container.innerHTML += `<div class="file-item" data-file="${f}"><i class="fas fa-file"></i> ${f}</div>`;
            }
        });
        // Re-attach click events for search results
        container.querySelectorAll('.file-item').forEach(item => {
            item.onclick = () => {
                UI.switchPanel('explorer');
                openFile(item.dataset.file);
            }
        });
    };

    // Initial render
    setTimeout(compileAndRender, 500);
});