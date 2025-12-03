import { contextBridge, ipcRenderer } from 'electron';

contextBridge.exposeInMainWorld('api', {
  onMenu: (channel: string, cb: (payload?: any) => void) => ipcRenderer.on(channel, (_e, payload) => cb(payload)),
  fsList: (dir: string) => ipcRenderer.invoke('fs:list', dir),
  fsRead: (file: string) => ipcRenderer.invoke('fs:read', file),
  fsWrite: (file: string, content: string) => ipcRenderer.invoke('fs:write', file, content),
  saveDialog: (def: string) => ipcRenderer.invoke('dialog:save', def),
  exportPNGDialog: (def: string) => ipcRenderer.invoke('dialog:exportPNG', def),
});
