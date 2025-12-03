import React, { useEffect } from 'react';
import { useStore } from './store';
import Explorer from './Explorer';
import EditorPanel from './Editor';
import Viewport from './Viewport';
import ConsolePanel from './Console';
import Toolbar from './Toolbar';
import StatusBar from './StatusBar';

declare global { interface Window { api: any } }

export default function App() {
  const setProjectDir = useStore(s => s.setProjectDir);
  const setFiles = useStore(s => s.setFiles);
  const setActive = useStore(s => s.setActive);
  const log = useStore(s => s.log);

  useEffect(() => {
    window.api.onMenu('menu:new', () => {
      setProjectDir(null);
      const defaultFiles = [
        { name: 'main.scad', content: `// Demo Cube\ncolor("#bd93f9");\ntranslate([0,0,0]);\ncube([20,20,20]);\n\n// Add a sphere\ntranslate([25,0,0]);\ncolor("#50fa7b");\nsphere(r=12);\n`, active: true },
        { name: 'parts.scad', content: `// Put reusable parts here\n` },
        { name: 'config.scad', content: `// Variables and configuration\nsize=20;` }
      ];
      setFiles(defaultFiles);
      setActive('main.scad');
      log({ level: 'info', msg: 'New project created' });
    });

    window.api.onMenu('menu:open', async (dir: string) => {
      setProjectDir(dir);
      const entries = await window.api.fsList(dir);
      const files = await Promise.all(entries.map(async (e: any) => ({
        name: e.name,
        content: await window.api.fsRead(`${dir}/${e.name}`)
      })));
      setFiles(files);
      if (files[0]) setActive(files[0].name);
      log({ level: 'info', msg: `Opened project: ${dir}` });
    });

    window.api.onMenu('menu:save', async () => {
      const s = useStore.getState();
      if (!s.projectDir) {
        log({ level: 'warn', msg: 'No project directory. Open a folder to save.' });
        return;
      }
      for (const f of s.files) {
        await window.api.fsWrite(`${s.projectDir}/${f.name}`, f.content);
      }
      log({ level: 'info', msg: 'Project saved.' });
    });

    window.api.onMenu('menu:render', () => document.dispatchEvent(new CustomEvent('action:render')));
    window.api.onMenu('menu:preview', () => document.dispatchEvent(new CustomEvent('action:preview')));
    window.api.onMenu('menu:export-stl', () => document.dispatchEvent(new CustomEvent('action:export-stl')));
    window.api.onMenu('menu:export-png', () => document.dispatchEvent(new CustomEvent('action:export-png')));
  }, []);

  return (
    <div className="layout">
      <Toolbar />
      <div className="explorer">
        <div className="title">ПРОВІДНИК</div>
        <Explorer />
      </div>
      <div className="editor-viewport">
        <div className="editor"><EditorPanel /></div>
        <div className="viewport"><Viewport /></div>
      </div>
      <ConsolePanel />
      <StatusBar />
    </div>
  );
}
