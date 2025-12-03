export function parseAndRun(code, engine) {
    engine.clearScene();
    const lines = code.split('\n');
    
    let curColor = "#bd93f9"; 
    let curTrans = [0,0,0];
    let curRot = [0,0,0];
    let objCount = 0;

    lines.forEach(line => {
        line = line.trim();
        if(line.startsWith('//') || line.length === 0) return;

        if(line.startsWith('color')) {
            const match = line.match(/color\("(.+?)"\)/);
            if(match) curColor = match[1];
        }
        else if(line.startsWith('translate')) {
            const match = line.match(/\[([\d.-]+),\s*([\d.-]+),\s*([\d.-]+)\]/);
            if(match) curTrans = [parseFloat(match[1]), parseFloat(match[2]), parseFloat(match[3])];
        }
        else if(line.startsWith('cube')) {
            const match = line.match(/\[([\d.-]+),\s*([\d.-]+),\s*([\d.-]+)\]/);
            if(match) {
                const w=parseFloat(match[1]), h=parseFloat(match[2]), d=parseFloat(match[3]);
                const geo = new THREE.BoxGeometry(w, h, d);
                engine.addMesh(geo, curColor, curTrans, curRot);
                objCount++;
            }
        }
        else if(line.startsWith('sphere')) {
            const match = line.match(/r=([\d.-]+)/);
            if(match) {
                const geo = new THREE.SphereGeometry(parseFloat(match[1]), 32, 32);
                engine.addMesh(geo, curColor, curTrans, curRot);
                objCount++;
            }
        }
        else if(line.startsWith('cylinder')) {
            const hMatch = line.match(/h=([\d.-]+)/);
            const rMatch = line.match(/r=([\d.-]+)/);
            if(hMatch && rMatch) {
                const geo = new THREE.CylinderGeometry(parseFloat(rMatch[1]), parseFloat(rMatch[1]), parseFloat(hMatch[1]), 32);
                engine.addMesh(geo, curColor, curTrans, curRot);
                objCount++;
            }
        }
    });
    return objCount;
}