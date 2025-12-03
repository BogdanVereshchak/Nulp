import React from 'react';
import { useStore } from './store';

export default function StatusBar() {
  const active = useStore((s: { activeFile: any; }) => s.activeFile);
  const renderMs = useStore((s: { renderMs: any; }) => s.renderMs);
  const count = useStore((s: { objCount: any; }) => s.objCount);

  return (
    <div className="status">
      <div><strong>File:</strong> {active || '—'}</div>
      <div><strong>Render:</strong> {renderMs.toFixed(3)} ms</div>
      <div><strong>Objects:</strong> {count}</div>
    </div>
  );
}
