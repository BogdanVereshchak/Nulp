import { create } from 'zustand';

type FileEntry = { name: string; content: string; active?: boolean };
type LogEntry = { level: 'info'|'warn'|'error'; msg: string };

type State = {
  projectDir: string | null;
  files: FileEntry[];
  activeFile: string | null;
  logs: LogEntry[];
  renderMs: number;
  objCount: number;
  setProjectDir: (dir: string | null) => void;
  setFiles: (files: FileEntry[]) => void;
  setActive: (name: string) => void;
  updateContent: (name: string, content: string) => void;
  log: (entry: LogEntry) => void;
  setRenderStats: (ms: number, count: number) => void;
};

export const useStore = create<State>((set) => ({
  projectDir: null,
  files: [],
  activeFile: null,
  logs: [],
  renderMs: 0,
  objCount: 0,
  setProjectDir: (dir) => set({ projectDir: dir }),
  setFiles: (files) => set({ files }),
  setActive: (name) => set((s) => ({ activeFile: name, files: s.files.map(f => ({...f, active: f.name === name})) })),
  updateContent: (name, content) => set((s) => ({ files: s.files.map(f => f.name === name ? {...f, content} : f) })),
  log: (entry) => set((s) => ({ logs: [...s.logs, entry] })),
  setRenderStats: (ms, count) => set({ renderMs: ms, objCount: count }),
}));
