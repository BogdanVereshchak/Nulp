import { app, BrowserWindow, dialog, ipcMain, Menu } from 'electron';
import path from 'path';
import fs from 'fs/promises';

let win: BrowserWindow | null = null;

const createWindow = async () => {
  win = new BrowserWindow({
    width: 1400,
    height: 900,
    backgroundColor: '#1e1e1e',
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      preload: path.join(__dirname, 'preload.js'),
    },
    title: 'OpenSCAD Modern UI v2.0',
    show: false,
  });
  const url = app.isPackaged
    ? `file://${path.join(__dirname, '../renderer/index.html')}`
    : 'http://localhost:5173';
  await win.loadURL(url);
  win.once('ready-to-show', () => win?.show());
  setupMenu();
};

const setupMenu = () => {
  const template = [
    {
      label: 'File',
      submenu: [
        { label: 'New Project', accelerator: 'CmdOrCtrl+N', click: () => win?.webContents.send('menu:new') },
        { label: 'Open...', accelerator: 'CmdOrCtrl+O', click: async () => {
          const res = await dialog.showOpenDialog(win!, { properties: ['openDirectory'] });
          if (!res.canceled && res.filePaths[0]) win?.webContents.send('menu:open', res.filePaths[0]);
        }},
        { label: 'Save', accelerator: 'CmdOrCtrl+S', click: () => win?.webContents.send('menu:save') },
        { type: 'separator' },
        { label: 'Export STL', click: () => win?.webContents.send('menu:export-stl') },
        { label: 'Export PNG', click: () => win?.webContents.send('menu:export-png') },
        { type: 'separator' },
        { role: 'quit' },
      ]
    },
    {
      label: 'Edit',
      submenu: [
        { role: 'undo' }, { role: 'redo' },
        { type: 'separator' },
        { role: 'cut' }, { role: 'copy' }, { role: 'paste' },
        { role: 'selectAll' }
      ]
    },
    {
      label: 'View',
      submenu: [
        { label: 'Preview', accelerator: 'CmdOrCtrl+P', click: () => win?.webContents.send('menu:preview') },
        { label: 'Render', accelerator: 'CmdOrCtrl+R', click: () => win?.webContents.send('menu:render') },
        { type: 'separator' },
        { role: 'toggleDevTools' }, { role: 'reload' }
      ]
    }
  ];
  const menu = Menu.buildFromTemplate(template as any);
  Menu.setApplicationMenu(menu);
};

app.whenReady().then(createWindow);
app.on('window-all-closed', () => { if (process.platform !== 'darwin') app.quit(); });
app.on('activate', () => { if (BrowserWindow.getAllWindows().length === 0) createWindow(); });

// IPC: filesystem for projects
ipcMain.handle('fs:list', async (_e, dir: string) => {
  const entries = await fs.readdir(dir, { withFileTypes: true });
  return entries.filter(e => e.isFile() && e.name.endsWith('.scad')).map(e => ({ name: e.name }));
});
ipcMain.handle('fs:read', async (_e, filePath: string) => {
  return await fs.readFile(filePath, 'utf-8');
});
ipcMain.handle('fs:write', async (_e, filePath: string, content: string) => {
  await fs.writeFile(filePath, content, 'utf-8');
  return true;
});
ipcMain.handle('dialog:save', async (_e, defaultPath: string) => {
  const res = await dialog.showSaveDialog(win!, { defaultPath, filters: [{ name: 'STL', extensions: ['stl'] }] });
  return res.canceled ? null : res.filePath;
});
ipcMain.handle('dialog:exportPNG', async (_e, defaultPath: string) => {
  const res = await dialog.showSaveDialog(win!, { defaultPath, filters: [{ name: 'PNG', extensions: ['png'] }] });
  return res.canceled ? null : res.filePath;
});
