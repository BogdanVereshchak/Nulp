export class OpenSCADEngine {
    constructor() {
        this.container = document.getElementById('canvasContainer');
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x282a36);
        
        this.camera = new THREE.PerspectiveCamera(45, this.container.clientWidth / this.container.clientHeight, 0.1, 1000);
        this.camera.position.set(50, 50, 50);
        this.camera.lookAt(0, 0, 0);

        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.container.appendChild(this.renderer.domElement);

        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;

        const ambientLight = new THREE.AmbientLight(0x404040, 2);
        this.scene.add(ambientLight);
        const dirLight = new THREE.DirectionalLight(0xffffff, 1);
        dirLight.position.set(10, 20, 10);
        this.scene.add(dirLight);

        this.grid = new THREE.GridHelper(100, 100, 0x44475a, 0x44475a);
        this.scene.add(this.grid);
        
        this.modelGroup = new THREE.Group();
        this.scene.add(this.modelGroup);

        window.addEventListener('resize', () => this.onWindowResize(), false);
        this.animate();
    }

    onWindowResize() {
        if (!this.container) return;
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }

    resetView() {
        this.camera.position.set(50, 50, 50);
        this.camera.lookAt(0, 0, 0);
        this.controls.reset();
    }

    toggleGrid() { this.grid.visible = !this.grid.visible; }
    
    toggleWireframe() {
        this.modelGroup.children.forEach(mesh => {
            if(mesh.material) mesh.material.wireframe = !mesh.material.wireframe;
        });
    }

    clearScene() {
        while(this.modelGroup.children.length > 0){ 
            this.modelGroup.remove(this.modelGroup.children[0]); 
        }
    }

    addMesh(geometry, color, pos=[0,0,0], rot=[0,0,0]) {
        const material = new THREE.MeshPhongMaterial({ 
            color: color, 
            specular: 0x111111, 
            shininess: 30,
            transparent: true,
            opacity: 0.9
        });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.set(pos[0], pos[1], pos[2]);
        mesh.rotation.set(rot[0] * Math.PI/180, rot[1] * Math.PI/180, rot[2] * Math.PI/180);
        this.modelGroup.add(mesh);
    }

    setTheme(themeName) {
        const colors = {
            'dracula': 0x282a36,
            'light': 0xf3f4f6,
            'monokai': 0x272822,
            'oceanic': 0x1a1c25
        };
        this.scene.background = new THREE.Color(colors[themeName] || 0x282a36);
    }
}