import * as THREE from 'three';

type Transform = {
  translate?: [number, number, number];
  // Future: rotate, scale
};

type CubeNode = {
  type: 'cube';
  size: [number, number, number];
  color?: string;
  transform?: Transform;
};

type SphereNode = {
  type: 'sphere';
  r: number;
  color?: string;
  transform?: Transform;
};

type GroupNode = {
  type: 'group';
  op: 'union' | 'difference' | 'intersection';
  children: ASTNode[];
  color?: string;
  transform?: Transform;
};

type ASTNode = CubeNode | SphereNode | GroupNode;

/**
 * Parse a minimal subset of OpenSCAD-like syntax into an AST.
 * Supported:
 *  - color("#hex")
 *  - translate([x,y,z])
 *  - cube([x,y,z])
 *  - sphere(r=R)
 *  - union { ... }, difference { ... }, intersection { ... }
 *
 * Notes:
 *  - color persists until changed (like a current material).
 *  - translate applies only to the next shape or group block.
 *  - Semicolons are optional for supported calls.
 */
export function parseToAST(code: string): ASTNode[] {
  const tokens = tokenize(code);
  const ctx = { i: 0, color: undefined as string | undefined, pendingTranslate: undefined as Transform | undefined };

  const nodes: ASTNode[] = [];
  while (ctx.i < tokens.length) {
    const node = parseStatement(tokens, ctx);
    if (node) nodes.push(node);
  }
  return nodes;
}

/* ---------------------- Tokenizer ---------------------- */

type Tok =
  | { t: 'ident'; v: string }
  | { t: 'number'; v: number }
  | { t: 'hex'; v: string } // "#RRGGBB"
  | { t: 'sym'; v: string }; // one-char symbols: ( ) [ ] { } , = ;

function tokenize(code: string): Tok[] {
  const src = code.replace(/\r/g, '');
  const toks: Tok[] = [];
  let i = 0;

  const isWS = (c: string) => /\s/.test(c);
  const isLetter = (c: string) => /[A-Za-z_]/.test(c);
  const isDigit = (c: string) => /[0-9]/.test(c);

  while (i < src.length) {
    const c = src[i];

    // Comments
    if (c === '/' && src[i + 1] === '/') {
      while (i < src.length && src[i] !== '\n') i++;
      continue;
    }

    if (isWS(c)) { i++; continue; }

    // Hex color "#RRGGBB"
    if (c === '#') {
      const m = src.slice(i).match(/^#[0-9a-fA-F]{6}/);
      if (!m) throw new Error(`Invalid hex color at index ${i}`);
      toks.push({ t: 'hex', v: m[0] });
      i += m[0].length;
      continue;
    }

    // Identifiers
    if (isLetter(c)) {
      let j = i + 1;
      while (j < src.length && /[\w]/.test(src[j])) j++;
      toks.push({ t: 'ident', v: src.slice(i, j) });
      i = j;
      continue;
    }

    // Numbers (support negatives and decimals)
    if (c === '-' || isDigit(c)) {
      let j = i + 1;
      while (j < src.length && /[\d.]/.test(src[j])) j++;
      const num = parseFloat(src.slice(i, j));
      if (!Number.isFinite(num)) throw new Error(`Invalid number at index ${i}`);
      toks.push({ t: 'number', v: num });
      i = j;
      continue;
    }

    // Symbols
    if ('()[]{};,='.includes(c)) {
      toks.push({ t: 'sym', v: c });
      i++;
      continue;
    }

    throw new Error(`Unexpected character '${c}' at index ${i}`);
  }

  return toks;
}

/* ---------------------- Parser ---------------------- */

function peek(tokens: Tok[], i: number, type?: Tok['t'], val?: string): Tok | undefined {
  const tok = tokens[i];
  if (!tok) return undefined;
  if (type && tok.t !== type) return undefined;
  if (val && (tok as any).v !== val) return undefined;
  return tok;
}

function expect(tokens: Tok[], ctx: { i: number }, type: Tok['t'], val?: string): Tok {
  const tok = peek(tokens, ctx.i, type, val);
  if (!tok) throw new Error(`Expected ${val ?? type}`);
  ctx.i++;
  return tok;
}

function optionalSym(tokens: Tok[], ctx: { i: number }, sym: string) {
  const tok = peek(tokens, ctx.i, 'sym', sym);
  if (tok) ctx.i++;
  return !!tok;
}

function parseStatement(tokens: Tok[], ctx: { i: number; color?: string; pendingTranslate?: Transform }): ASTNode | null {
  const id = peek(tokens, ctx.i, 'ident');
  if (!id) {
    // trailing semicolons or end
    if (optionalSym(tokens, ctx, ';')) return null;
    // no statement
    return null;
  }

  switch (id.v) {
    case 'color':
      return parseColor(tokens, ctx);
    case 'translate':
      return parseTranslate(tokens, ctx);
    case 'cube':
      return parseCube(tokens, ctx);
    case 'sphere':
      return parseSphere(tokens, ctx);
    case 'union':
    case 'difference':
    case 'intersection':
      return parseGroup(tokens, ctx, id.v as GroupNode['op']);
    default:
      throw new Error(`Unknown identifier '${id.v}'`);
  }
}

function parseColor(tokens: Tok[], ctx: { i: number; color?: string; pendingTranslate?: Transform }): null {
  expect(tokens, ctx, 'ident', 'color');
  expect(tokens, ctx, 'sym', '(');
  const hexTok = expect(tokens, ctx, 'hex') as { t: 'hex'; v: string };
  expect(tokens, ctx, 'sym', ')');
  optionalSym(tokens, ctx, ';'); // optional semicolon
  ctx.color = hexTok.v;
  return null;
}

function parseTranslate(tokens: Tok[], ctx: { i: number; color?: string; pendingTranslate?: Transform }): null {
  expect(tokens, ctx, 'ident', 'translate');
  expect(tokens, ctx, 'sym', '(');
  expect(tokens, ctx, 'sym', '[');
  const x = expect(tokens, ctx, 'number').v as number;
  expect(tokens, ctx, 'sym', ',');
  const y = expect(tokens, ctx, 'number').v as number;
  expect(tokens, ctx, 'sym', ',');
  const z = expect(tokens, ctx, 'number').v as number;
  expect(tokens, ctx, 'sym', ']');
  expect(tokens, ctx, 'sym', ')');
  optionalSym(tokens, ctx, ';'); // optional semicolon
  ctx.pendingTranslate = { translate: [x, y, z] };
  return null;
}

function parseCube(tokens: Tok[], ctx: { i: number; color?: string; pendingTranslate?: Transform }): CubeNode {
  expect(tokens, ctx, 'ident', 'cube');
  expect(tokens, ctx, 'sym', '(');
  expect(tokens, ctx, 'sym', '[');
  const sx = expect(tokens, ctx, 'number').v as number;
  expect(tokens, ctx, 'sym', ',');
  const sy = expect(tokens, ctx, 'number').v as number;
  expect(tokens, ctx, 'sym', ',');
  const sz = expect(tokens, ctx, 'number').v as number;
  expect(tokens, ctx, 'sym', ']');
  expect(tokens, ctx, 'sym', ')');
  optionalSym(tokens, ctx, ';');

  const node: CubeNode = {
    type: 'cube',
    size: [sx, sy, sz],
    color: ctx.color,
    transform: ctx.pendingTranslate,
  };
  // translate applies only once
  ctx.pendingTranslate = undefined;
  return node;
}

function parseSphere(tokens: Tok[], ctx: { i: number; color?: string; pendingTranslate?: Transform }): SphereNode {
  expect(tokens, ctx, 'ident', 'sphere');
  expect(tokens, ctx, 'sym', '(');
  expect(tokens, ctx, 'ident', 'r'); // require 'r'
  expect(tokens, ctx, 'sym', '=');
  const r = expect(tokens, ctx, 'number').v as number;
  expect(tokens, ctx, 'sym', ')');
  optionalSym(tokens, ctx, ';');

  const node: SphereNode = {
    type: 'sphere',
    r,
    color: ctx.color,
    transform: ctx.pendingTranslate,
  };
  ctx.pendingTranslate = undefined;
  return node;
}

function parseGroup(
  tokens: Tok[],
  ctx: { i: number; color?: string; pendingTranslate?: Transform },
  op: GroupNode['op']
): GroupNode {
  expect(tokens, ctx, 'ident', op);
  // translate can come before block, but in our subset, we apply the current pendingTranslate if set
  expect(tokens, ctx, 'sym', '{');

  const inner: ASTNode[] = [];
  // parse until matching '}'
  let depth = 1;
  const innerTokens: Tok[] = [];
  while (ctx.i < tokens.length && depth > 0) {
    const t = tokens[ctx.i++];
    innerTokens.push(t);
    if (t.t === 'sym' && t.v === '{') depth++;
    else if (t.t === 'sym' && t.v === '}') depth--;
  }

  // Remove trailing '}' from innerTokens
  while (innerTokens.length && innerTokens[innerTokens.length - 1].t === 'sym' && innerTokens[innerTokens.length - 1].v === '}') {
    innerTokens.pop();
  }

  // Parse children in a fresh context but inherit color
  const childCtx = { i: 0, color: ctx.color, pendingTranslate: undefined as Transform | undefined };
  while (childCtx.i < innerTokens.length) {
    const node = parseStatement(innerTokens, childCtx);
    if (node) inner.push(node);
  }

  const node: GroupNode = {
    type: 'group',
    op,
    children: inner,
    color: ctx.color,
    transform: ctx.pendingTranslate,
  };
  ctx.pendingTranslate = undefined;
  return node;
}

/* ---------------------- Scene builder ---------------------- */

export function buildSceneFromAST(ast: ASTNode[], mode: 'render' | 'preview'): THREE.Mesh[] {
  const meshes: THREE.Mesh[] = [];
  const matWire = new THREE.MeshBasicMaterial({ color: 0x8888aa, wireframe: true });
  const defaultMat = new THREE.MeshPhongMaterial({ color: 0x9aa0a6, flatShading: true });

  for (const node of ast) {
    switch (node.type) {
      case 'cube': {
        const geom = new THREE.BoxGeometry(node.size[0], node.size[1], node.size[2]);
        const mat = mode === 'preview' ? matWire : materialFrom(node.color, defaultMat);
        const mesh = new THREE.Mesh(geom, mat);
        applyTransform(mesh, node.transform);
        meshes.push(mesh);
        break;
      }
      case 'sphere': {
        const geom = new THREE.SphereGeometry(node.r, 32, 16);
        const mat = mode === 'preview' ? matWire : materialFrom(node.color, defaultMat);
        const mesh = new THREE.Mesh(geom, mat);
        applyTransform(mesh, node.transform);
        meshes.push(mesh);
        break;
      }
      case 'group': {
        // Placeholder: booleans are not applied; children are grouped visually.
        const childMeshes = buildSceneFromAST(node.children, mode);
        // Apply group transform by wrapping in a THREE.Group
        const group = new THREE.Group();
        childMeshes.forEach(m => group.add(m));
        applyTransform(group, node.transform);

        // Flatten meshes back out so viewport code that counts meshes still works
        group.traverse(o => {
          if ((o as any).isMesh) meshes.push(o as THREE.Mesh);
        });
        break;
      }
    }
  }

  return meshes;
}

function materialFrom(color: string | undefined, fallback: THREE.Material): THREE.Material {
  if (!color) return fallback;
  return new THREE.MeshPhongMaterial({ color, flatShading: true });
}

function applyTransform(obj: THREE.Object3D, t?: Transform) {
  if (!t) return;
  if (t.translate) obj.position.set(t.translate[0], t.translate[1], t.translate[2]);
}
