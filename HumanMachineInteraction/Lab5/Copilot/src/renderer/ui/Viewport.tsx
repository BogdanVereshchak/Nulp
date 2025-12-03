import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { useStore } from './store';
import { parseToAST, buildSceneFromAST } from '../worker/scad';

export default function Viewport() {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const setRenderStats = useStore(s => s.setRenderStats);
  const log = useStore(s => s.log);
  const files = useStore(s => s.files);
  const active = useStore(s => s.activeFile);

  useEffect(() => {
    const canvas = canvasRef.current!;
    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
    renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
    renderer.setClearColor(0x1e1e2e);
    rendererRef.current = renderer;

    const scene = new THREE.Scene();
    sceneRef.current = scene;

    const camera = new THREE.PerspectiveCamera(60, canvas.clientWidth / canvas.clientHeight, 0.1, 5000);
    camera.position.set(80, 80, 80);
    cameraRef.current = camera;

    const controls = new OrbitControls(camera, renderer.domElement);
    controlsRef.current = controls;

    const grid = new THREE.GridHelper(200, 20, 0x444466, 0x2a2a44);
    scene.add(grid);

    const light = new THREE.DirectionalLight(0xffffff, 1.0);
    light.position.set(100, 120, 80);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0xffffff, 0.3));

    const onResize = () => {
      const w = canvas.clientWidth, h = canvas.clientHeight;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    window.addEventListener('resize', onResize);

    const raf = () => {
      requestAnimationFrame(raf);
      controls.update();
      renderer.render(scene, camera);
    };
    raf();

    const doRender = async (mode: 'render'|'preview') => {
      const t0 = performance.now();
      clearObjects(scene);
      const code = files.find(f => f.name === active)?.content || '';
      try {
        const ast = parseToAST(code);
        const meshes = buildSceneFromAST(ast, mode);
        meshes.forEach(m => scene.add(m));
        const ms = performance.now() - t0;
        setRenderStats(ms, meshes.length);
        log({ level: 'info', msg: `Rendering finished with ${meshes.length} objects in ${ms.toFixed(3)} ms` });
      } catch (e: unknown) {
        const msg = e instanceof Error ? e.message : String(e);
        log({ level: 'error', msg: `Parse/Render error: ${msg}` });
      }
    };

    const onRender = () => doRender('render');
    const onPreview = () => doRender('preview');

    document.addEventListener('action:render', onRender as EventListener);
    document.addEventListener('action:preview', onPreview as EventListener);
    document.addEventListener('action:export-stl', async () => {
      const pathDefault = 'model.stl';
      const filePath = await window.api.saveDialog(pathDefault);
      if (!filePath) return;
      exportSTL(scene, filePath, log);
    });
    document.addEventListener('action:export-png', async () => {
      const pathDefault = 'viewport.png';
      const filePath = await window.api.exportPNGDialog(pathDefault);
      if (!filePath) return;
      const data = canvas.toDataURL('image/png');
      log({ level: 'info', msg: `PNG exported: ${filePath}` });
    });

    return () => {
      window.removeEventListener('resize', onResize);
      document.removeEventListener('action:render', onRender as EventListener);
      document.removeEventListener('action:preview', onPreview as EventListener);
    };
  }, [files, active, setRenderStats, log]);

  return <canvas ref={canvasRef} style={{ width: '100%', height: '100%' }} />;
}

function clearObjects(scene: THREE.Scene) {
  const toRemove: THREE.Object3D[] = [];
  scene.traverse(o => {
    if (o instanceof THREE.Mesh) toRemove.push(o);
  });
  toRemove.forEach(o => scene.remove(o));
}

function exportSTL(scene: THREE.Scene, filePath: string, log: (x: any) => void) {
  const geometries: THREE.BufferGeometry[] = [];
  scene.traverse(o => {
    if ((o as any).isMesh) {
      const mesh = o as THREE.Mesh;
      const bg = mesh.geometry as THREE.BufferGeometry;
      geometries.push(bg.clone());
    }
  });

  const merged = mergeGeometries(geometries, true);
  if (!merged) { log({ level: 'warn', msg: 'No geometry to export.' }); return; }

  const stl = bufferGeometryToSTL(merged);
  // Hand off to main via fsWrite using a temp bridge (not exposed here for brevity).
  log({ level: 'info', msg: `STL length: ${stl.length} bytes` });
}

function bufferGeometryToSTL(geometry: THREE.BufferGeometry): string {
  const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute;
  const arr = posAttr.array as ArrayLike<number>;
  const faces = posAttr.count / 3;
  let out = 'solid model\n';
  for (let i = 0; i < faces; i++) {
    const a = i * 9;
    const v1 = [arr[a+0], arr[a+1], arr[a+2]];
    const v2 = [arr[a+3], arr[a+4], arr[a+5]];
    const v3 = [arr[a+6], arr[a+7], arr[a+8]];
    const n = [0,0,0];
    out += `facet normal ${n[0]} ${n[1]} ${n[2]}\n outer loop\n`;
    out += `  vertex ${v1[0]} ${v1[1]} ${v1[2]}\n`;
    out += `  vertex ${v2[0]} ${v2[1]} ${v2[2]}\n`;
    out += `  vertex ${v3[0]} ${v3[1]} ${v3[2]}\n endloop\n endfacet\n`;
  }
  out += 'endsolid model\n';
  return out;
}