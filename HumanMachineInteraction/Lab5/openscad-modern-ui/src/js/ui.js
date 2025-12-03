import { files, state, saveCurrentFile } from './fileSystem.js';

// Helper to log to the onscreen console
export function log(msg, type='') {
    const c = document.getElementById('consoleLog');
    c.innerHTML += `<div class="log-line ${type}">> ${msg}</div>`;
    c.scrollTop = c.scrollHeight;
}

export function updateLineNumbers() {
    const editor = document.getElementById('codeEditor');
    const lines = editor.value.split('\n').length;
    document.getElementById('lineNumbers').innerHTML = Array(lines).fill(0).map((_, i) => i + 1).join('<br>');
}

export function renderFileList(onOpenFile) {
    const list = document.getElementById('fileList');
    list.innerHTML = '';
    Object.keys(files).forEach(filename => {
        const div = document.createElement('div');
        div.className = `file-item ${filename === state.activeFile ? 'active' : ''}`;
        div.innerHTML = `<span class="file-icon"><i class="fas fa-cube"></i></span> ${filename}`;
        div.onclick = () => onOpenFile(filename);
        list.appendChild(div);
    });
}

export function renderTabs(onOpenFile, onCloseTab) {
    const container = document.getElementById('tabsContainer');
    container.innerHTML = '';
    state.openFiles.forEach(filename => {
        const div = document.createElement('div');
        div.className = `tab ${filename === state.activeFile ? 'active' : ''}`;
        
        const spanName = document.createElement('span');
        spanName.innerHTML = `<i class="fas fa-cube"></i> ${filename}`;
        spanName.onclick = () => onOpenFile(filename);
        
        const spanClose = document.createElement('span');
        spanClose.className = 'close-tab';
        spanClose.innerHTML = '<i class="fas fa-times"></i>';
        spanClose.onclick = (e) => { e.stopPropagation(); onCloseTab(filename); };

        div.appendChild(spanName);
        div.appendChild(spanClose);
        container.appendChild(div);
    });
}

export function loadFileIntoEditor(filename) {
    const editor = document.getElementById('codeEditor');
    editor.value = files[filename];
    updateLineNumbers();
}

export function switchPanel(panelId) {
    const sidePanel = document.getElementById('sidePanel');
    const targetView = document.getElementById('view-' + panelId);
    const activeBtn = document.getElementById('btn-' + panelId);
    
    if (targetView.style.display === 'flex' && !sidePanel.classList.contains('collapsed')) {
        sidePanel.classList.add('collapsed');
        if(activeBtn) activeBtn.classList.remove('active');
        return;
    }

    sidePanel.classList.remove('collapsed');
    document.querySelectorAll('.panel-view').forEach(el => el.style.display = 'none');
    document.querySelectorAll('.sidebar .icon-btn').forEach(el => el.classList.remove('active'));
    
    targetView.style.display = 'flex';
    if(activeBtn) activeBtn.classList.add('active');
}