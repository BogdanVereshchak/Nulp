import React from 'react';
import { useStore } from './store';

export default function Toolbar() {
  const log = useStore((s: { log: any; }) => s.log);

  const send = (name: string) => document.dispatchEvent(new CustomEvent(name));
  const click = (cmd: string) => {
    switch (cmd) {
      case 'new': send('menu:new'); break;
      case 'open': send('menu:open'); break;
      case 'save': send('menu:save'); break;
      case 'render': send('action:render'); break;
      case 'preview': send('action:preview'); break;
      case 'export-stl': send('action:export-stl'); break;
      case 'export-png': send('action:export-png'); break;
      default: log({ level: 'warn', msg: 'Unknown toolbar action' });
    }
  };

  return (
    <div className="toolbar">
      <div className="group">
        <button className="btn" onClick={() => click('new')}>New</button>
        <button className="btn" onClick={() => click('open')}>Open</button>
        <button className="btn" onClick={() => click('save')}>Save</button>
      </div>
      <div className="group">
        <button className="btn primary" onClick={() => click('render')}>Render</button>
        <button className="btn" onClick={() => click('preview')}>Preview</button>
      </div>
      <div className="group">
        <button className="btn" onClick={() => click('export-stl')}>Export STL</button>
        <button className="btn" onClick={() => click('export-png')}>Export PNG</button>
      </div>
      <div className="spacer"></div>
      <div className="badge">OpenSCAD Modern UI v2.0</div>
    </div>
  );
}
