import React from 'react';
import { useStore } from './store';

export default function ConsolePanel() {
  const logs = useStore((s: { logs: any; }) => s.logs);
  return (
    <div className="console">
      {logs.slice(-10).map((l: { level: string; msg: string | number | bigint | boolean | React.ReactElement<unknown, string | React.JSXElementConstructor<any>> | Iterable<React.ReactNode> | React.ReactPortal | Promise<string | number | bigint | boolean | React.ReactPortal | React.ReactElement<unknown, string | React.JSXElementConstructor<any>> | Iterable<React.ReactNode> | null | undefined> | null | undefined; }, i: React.Key | null | undefined) => (
        <div key={i}>
          <span style={{ color: color(l.level) }}>[{l.level.toUpperCase()}]</span> {l.msg}
        </div>
      ))}
    </div>
  );
}

function color(level: string): string {
  switch (level) {
    case 'info': return '#8be9fd';
    case 'warn': return '#f1fa8c';
    case 'error': return '#ff5555';
    default: return '#f8f8f2';
  }
}
