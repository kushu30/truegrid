(function() {
    'use strict';

    const POLLING_INTERVAL_MS = 200;
    const JSON_URL = 'simframe.json';
    const ROAD_WIDTH = 8; 
    const NUM_BUILDINGS = 150;
    const BUILDING_SPAWN_RADIUS = 800;
    const MIN_BUILDING_DIST_FROM_ROAD = 20;

    const container = document.getElementById('canvas-container');
    const hudEl = document.getElementById('hud');
    const loadingEl = document.getElementById('loading');
    const loadingText = document.getElementById('loading-text');

    let scene, camera, renderer, controls, carGroup, trailLine;
    let simulationPlaying = true;
    let followCar = true;
    let pollTimer = null;
    let lastJsonText = null;
    const obstaclesMap = new Map();
    let trailPoints = [];
    let isSceneInitialized = false;
    let roadPathForBuildings = [];
    
    const roadMat = new THREE.MeshStandardMaterial({ color: 0x404040, roughness: 0.8 });
    const roadSegmentsGroup = new THREE.Group();
    const barricadeMaterial = new THREE.MeshStandardMaterial();


    function init() {
      scene = new THREE.Scene();
      camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 5000);
      camera.position.set(150, 120, 150);
      
      renderer = new THREE.WebGLRenderer({ antialias: true });
      renderer.setSize(window.innerWidth, window.innerHeight);
      renderer.shadowMap.enabled = true;
      renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      renderer.setPixelRatio(window.devicePixelRatio);
      container.appendChild(renderer.domElement);
      
      controls = new THREE.OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.dampingFactor = 0.06;

      scene.add(new THREE.AmbientLight(0xffffff, 0.6));
      const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
      directionalLight.position.set(200, 300, 200);
      directionalLight.castShadow = true;
      directionalLight.shadow.mapSize.width = 2048;
      directionalLight.shadow.mapSize.height = 2048;
      scene.add(directionalLight);

      const ground = new THREE.Mesh(new THREE.PlaneGeometry(3000, 3000), new THREE.MeshStandardMaterial({ color: 0x556B2F, roughness: 0.9 }));
      ground.rotation.x = -Math.PI / 2;
      ground.receiveShadow = true;
      scene.add(ground);
      
      const skyGeo = new THREE.SphereGeometry(2000, 32, 15);
      const skyMat = new THREE.MeshBasicMaterial({ color: 0x87ceeb, side: THREE.BackSide });
      scene.add(new THREE.Mesh(skyGeo, skyMat));
      scene.fog = new THREE.Fog(0x87ceeb, 500, 1800);

      carGroup = createCar();
      scene.add(carGroup);
      
      trailLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x00ff88, transparent: true, opacity: 0.55 }));
      scene.add(trailLine);
      
      scene.add(roadSegmentsGroup);

      const loader = new THREE.TextureLoader();
      const barricadeTexture = loader.load('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAgAAAAECAYAAACzzX7wAAAABGdBTUEAALGPC/xhBQAAACNJREFUGGNj/P//PwM2AEj////9GIGwGSSTgSwySgYJmAACAgQYAOa0B5+gqWjSAAAAAElFTSuQmCC');
      barricadeTexture.wrapS = barricadeTexture.wrapT = THREE.RepeatWrapping;
      barricadeTexture.repeat.set(4, 2);
      barricadeTexture.magFilter = THREE.NearestFilter;
      barricadeMaterial.map = barricadeTexture;

      setupEventListeners();
      animate();
      startPolling();
    }

    function animate() {
      requestAnimationFrame(animate);
      if (followCar && carGroup) {
          const targetPosition = new THREE.Vector3(carGroup.position.x, 0, carGroup.position.z);
          controls.target.lerp(targetPosition, 0.12);
          const offset = new THREE.Vector3(-30, 20, 0); 
          offset.applyAxisAngle(new THREE.Vector3(0, 1, 0), carGroup.rotation.y);
          const cameraPosition = carGroup.position.clone().add(offset);
          camera.position.lerp(cameraPosition, 0.08);
      }
      controls.update();
      renderer.render(scene, camera);
    }

    async function fetchAndApplyData() {
        if (!simulationPlaying) return;
        try {
            const res = await fetch(`${JSON_URL}?_=${Date.now()}`, { cache: 'no-store' });
            if (!res.ok) {
              loadingText.textContent = `Waiting for simframe.json — HTTP ${res.status}`;
              return;
            }
            const text = await res.text();
            if (!text || text === lastJsonText) return;
            
            lastJsonText = text;
            const frame = JSON.parse(text);
            
            if (loadingEl.style.display !== 'none') {
                loadingEl.style.opacity = 0;
                setTimeout(() => loadingEl.style.display = 'none', 300);
            }

            if (!isSceneInitialized && frame.path && frame.path.length > 1) {
                roadPathForBuildings = frame.path.map(p => ({ x: p.x, y: p.y }));
                createRoadFromPath(frame.path);
                createBuildings();
                isSceneInitialized = true;
            }
            
            if (frame.cars && frame.cars.length > 0) updateCar(frame.cars[0]);
            if (frame.obstacles) updateObstacles(frame.obstacles);
            if (frame.telemetry) updateHUD(frame.telemetry, frame);
            
        } catch (err) {
            loadingText.textContent = 'Fetch/parsing error. Check console.';
            console.error('Error fetching or processing simframe.json:', err);
        }
    }

    function createRoadFromPath(pathPoints) {
      const path = pathPoints.map(p => new THREE.Vector3(p.x, 0.05, p.y));

      for (let i = 0; i < path.length - 1; i++) {
        const p1 = path[i];
        const p2 = path[i + 1];
        const distance = p1.distanceTo(p2);
        if (distance < 0.1) continue;

        const roadGeo = new THREE.BoxGeometry(ROAD_WIDTH, 0.1, distance);
        const roadSegment = new THREE.Mesh(roadGeo, roadMat);
        
        roadSegment.position.lerpVectors(p1, p2, 0.5);
        roadSegment.lookAt(p2);
        
        roadSegment.receiveShadow = true;
        roadSegmentsGroup.add(roadSegment);
      }
    }
    
    function createBuildings() {
        for (let i = 0; i < NUM_BUILDINGS; i++) {
            const pos = { x: (Math.random() - 0.5) * 2 * BUILDING_SPAWN_RADIUS, z: (Math.random() - 0.5) * 2 * BUILDING_SPAWN_RADIUS };
            let minDistToRoad = Infinity;
            for (let j = 0; j < roadPathForBuildings.length - 1; j++) {
                const p1 = roadPathForBuildings[j];
                const p2 = roadPathForBuildings[j + 1];
                const dist = Math.sqrt(distToSegmentSquared({x: pos.x, y: pos.z}, p1, p2));
                if (dist < minDistToRoad) { minDistToRoad = dist; }
            }
            if (minDistToRoad < MIN_BUILDING_DIST_FROM_ROAD) continue;
            const height = Math.random() * 80 + 20;
            const width = Math.random() * 20 + 10;
            const depth = Math.random() * 20 + 10;
            const buildingGeo = new THREE.BoxGeometry(width, height, depth);
            const buildingMat = new THREE.MeshStandardMaterial({ color: new THREE.Color().setHSL(0, 0, Math.random() * 0.4 + 0.3), metalness: 0.1, roughness: 0.7 });
            const building = new THREE.Mesh(buildingGeo, buildingMat);
            building.position.set(pos.x, height / 2, pos.z);
            building.castShadow = true; building.receiveShadow = true;
            scene.add(building);
        }
    }
    function createCar() {
        const group = new THREE.Group();
        const bodyMat = new THREE.MeshStandardMaterial({ color:0x0077ff, metalness:0.2, roughness:0.4 });
        const body = new THREE.Mesh(new THREE.BoxGeometry(4, 1.2, 2.2), bodyMat);
        body.castShadow = true; body.receiveShadow = true; group.add(body);
        const cabinMat = new THREE.MeshStandardMaterial({ color:0x3399ff, metalness:0.1, roughness:0.3 });
        const cabin = new THREE.Mesh(new THREE.BoxGeometry(2.2, 0.8, 2.0), cabinMat);
        cabin.position.set(-0.4, 1.0, 0); group.add(cabin);
        const wheelGeo = new THREE.CylinderGeometry(0.4, 0.4, 0.3, 24);
        wheelGeo.rotateX(Math.PI / 2);
        const wheelMat = new THREE.MeshStandardMaterial({ color:0x111111, metalness:0.1, roughness:0.6 });
        [{ x: 1.2, z: 1.25 }, { x: 1.2, z: -1.25 }, { x: -1.2, z: 1.25 }, { x: -1.2, z: -1.25 }].forEach(pos => {
            const wheel = new THREE.Mesh(wheelGeo, wheelMat);
            wheel.position.set(pos.x, -0.2, pos.z); group.add(wheel);
        });
        return group;
    }
    function updateCar(carData) {
      if(!carData) return;
      const x = Number(carData.x) || 0; const z = Number(carData.y) || 0; const y = (Number(carData.z) || 0) + 0.8;
      carGroup.position.set(x, y, z);
      const vx = Number(carData.vx) || 0; const vy = Number(carData.vy) || 0; const heading = Math.atan2(vy, vx);
      if(isFinite(heading)) { carGroup.rotation.y = -heading; }
      updateTrail(carData);
    }
    function updateTrail(carData) {
      const x = Number(carData.x) || 0;
      const z = Number(carData.y) || 0;
      trailPoints.push(new THREE.Vector3(x, 0.1, z));
      if (trailPoints.length > 500) trailPoints.shift();
      trailLine.geometry.setFromPoints(trailPoints);
    }
    function createObstacleMesh(o) {
        switch(o.type) {
            case 2:
                const potholeGeo = new THREE.CircleGeometry((o.width || 1.5) / 2, 24);
                const potholeMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1c, roughness: 0.8 });
                const meshPothole = new THREE.Mesh(potholeGeo, potholeMat);
                meshPothole.rotation.x = -Math.PI / 2;
                meshPothole.receiveShadow = true;
                return meshPothole;
            case 3:
                const barricadeGroup = new THREE.Group();
                const plank = new THREE.Mesh(new THREE.BoxGeometry(o.width || 3, 0.4, 0.1), barricadeMaterial);
                plank.castShadow = true;
                barricadeGroup.add(plank);
                const legMat = new THREE.MeshStandardMaterial({color: 0x505050});
                [-1.2, 1.2].forEach(xOffset => {
                    const leg1 = new THREE.Mesh(new THREE.BoxGeometry(0.1, 1, 0.1), legMat); leg1.position.set(xOffset, -0.4, 0.1); leg1.rotation.z = 0.25; barricadeGroup.add(leg1);
                    const leg2 = new THREE.Mesh(new THREE.BoxGeometry(0.1, 1, 0.1), legMat); leg2.position.set(xOffset, -0.4, -0.1); leg2.rotation.z = -0.25; barricadeGroup.add(leg2);
                });
                return barricadeGroup;
            default:
                const geom = new THREE.BoxGeometry((o.width||1.2), 0.7, (o.width||1.2));
                const mesh = new THREE.Mesh(geom, new THREE.MeshStandardMaterial({ color: 0xff4444 }));
                mesh.castShadow = true;
                return mesh;
        }
    }
    function updateObstacles(obstaclesArray) {
      const presentIds = new Set(obstaclesArray.map(o => o.id));
      obstaclesArray.forEach(o => {
        let entry = obstaclesMap.get(o.id);
        if (!entry) {
          const mesh = createObstacleMesh(o);
          scene.add(mesh);
          entry = { mesh };
          obstaclesMap.set(o.id, entry);
        }
        const x = Number(o.x) || 0; const z = Number(o.y) || 0; const y = (o.type === 2) ? 0.15 : 0.8;
        entry.mesh.position.set(x, y, z);
      });
      for (const [id, entry] of obstaclesMap.entries()) {
        if (!presentIds.has(id)) {
          scene.remove(entry.mesh);
          if (entry.mesh.isGroup) { entry.mesh.children.forEach(c => { c.geometry?.dispose(); c.material?.dispose(); });
          } else { entry.mesh.geometry?.dispose(); entry.mesh.material?.dispose(); }
          obstaclesMap.delete(id);
        }
      }
    }
    function updateHUD(telemetry = {}, frame = {}) {
        const currentSpeed = telemetry.currentSpeed_kmh ?? 0; const avgSpeed = telemetry.avgSpeed_km_h ?? 0; const reason = telemetry.lastSpeedReason || '—'; const time = telemetry.time?.toFixed(2) || '—'; const distance = Math.round(telemetry.distance) || '—'; let nearestObstacleHTML = '';
        if (frame.obstacles && frame.cars?.length) {
            const car = frame.cars[0]; let minDistance = Infinity, nearestType = null;
            frame.obstacles.forEach(o => { const d = Math.hypot(o.x-car.x, o.y-car.y); if (d < minDistance) { minDistance = d; nearestType = o.type; } });
            if (minDistance < Infinity) { const typeName = nearestType === 2 ? 'Pothole' : (nearestType === 3 ? 'Barricade' : 'Obstacle'); nearestObstacleHTML = `<div class="obstacle-info">Nearest: ${typeName} — ${minDistance.toFixed(1)} m</div>`; }
        }
        hudEl.innerHTML = `<div class="speed-indicator"><span class="current-speed">Current: ${Number(currentSpeed).toFixed(1)} km/h</span><br><span class="avg-speed">Average: ${Number(avgSpeed).toFixed(1)} km/h</span></div><div class="reason">Reason: ${reason}</div>${nearestObstacleHTML}<div class="status">Time: ${time}s | Distance: ${distance}m<br>Cars: ${frame.cars?.length || 0} | Obstacles: ${frame.obstacles?.length || 0}</div>`;
    }

    function setupEventListeners() {
        window.addEventListener('resize', () => {
          camera.aspect = window.innerWidth / window.innerHeight;
          camera.updateProjectionMatrix();
          renderer.setSize(window.innerWidth, window.innerHeight);
        }, false);
        document.getElementById('play-btn').addEventListener('click', () => { simulationPlaying = true; startPolling(); });
        document.getElementById('pause-btn').addEventListener('click', () => { simulationPlaying = false; stopPolling(); });
        document.getElementById('reset-btn').addEventListener('click', () => { location.reload(); });
        document.getElementById('follow-btn').addEventListener('click', (e) => {
          followCar = !followCar;
          e.target.textContent = `Follow Car: ${followCar ? 'ON' : 'OFF'}`;
          e.target.classList.toggle('active', followCar);
          if (!followCar) { controls.target.copy(carGroup.position); }
        });
    }
    function distToSegmentSquared(p, v, w) {
        const l2 = (v.x - w.x)**2 + (v.y - w.y)**2; if (l2 === 0) return (p.x - v.x)**2 + (p.y - v.y)**2; let t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2; t = Math.max(0, Math.min(1, t)); return (p.x - (v.x + t * (w.x - v.x)))**2 + (p.y - (v.y + t * (w.y - v.y)))**2;
    }
    function startPolling() { stopPolling(); pollTimer = setInterval(fetchAndApplyData, POLLING_INTERVAL_MS); fetchAndApplyData(); }
    function stopPolling() { if (pollTimer) { clearInterval(pollTimer); pollTimer = null; } }
    
    init();

  })();