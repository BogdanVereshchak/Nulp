import React from 'react';
import { useStore } from './store';
import clsx from 'clsx';

export default function Explorer() {
  const files = useStore((s: { files: any; }) => s.files);
  const setActive = useStore((s: { setActive: any; }) => s.setActive);

  return (
    <div>
      {files.map((f: { name: React.Key; active: any }) => (
        <div key={f.name} className={clsx('file', f.active && 'active')} onClick={() => setActive(f.name)}>
          <span>{String(f.name)}</span>
          <span style={{ color: '#666' }}>SCAD</span>
        </div>
      ))}
    </div>
  );
}
