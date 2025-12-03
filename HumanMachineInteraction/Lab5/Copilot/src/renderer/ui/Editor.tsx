import React, { useEffect, useRef } from 'react';
import * as monaco from 'monaco-editor';
import { useStore } from './store';

export default function EditorPanel() {
  const active = useStore((s: { activeFile: any; }) => s.activeFile);
  const files = useStore((s: { files: any; }) => s.files);
  const update = useStore((s: { updateContent: any; }) => s.updateContent);
  const editorRef = useRef<monaco.editor.IStandaloneCodeEditor | null>(null);
  const containerRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    if (!containerRef.current) return;
    editorRef.current = monaco.editor.create(containerRef.current, {
      value: files.find((f: { name: any; }) => f.name === active)?.content || '',
      language: 'scad',
      theme: 'vs-dark',
      automaticLayout: true,
      minimap: { enabled: false }
    });

    monaco.languages.register({ id: 'scad' });
    monaco.languages.setMonarchTokensProvider('scad', {
      keywords: ['cube','sphere','translate','color','union','difference','intersection'],
      tokenizer: {
        root: [
          [/[a-zA-Z_]\w*/, { cases: { '@keywords': 'keyword', '@default': 'identifier' } }],
          [/#?[a-fA-F0-9]{6}/, 'number'],
          [/\d+/, 'number'],
          [/[{},=;]/, 'delimiter'], [/\/\/.*$/, 'comment'], ] } });

            editorRef.current.onDidChangeModelContent(() => { const content = editorRef.current?.getValue() || ''; if (active) update(active, content); });

            return () => editorRef.current?.dispose(); }, []);

            useEffect(() => { const content = files.find((f: { name: any; }) => f.name === active)?.content || ''; if (editorRef.current) editorRef.current.setValue(content); }, [active]);

            return <div style={{ height: '100%', width: '100%' }} ref={containerRef} />; }
