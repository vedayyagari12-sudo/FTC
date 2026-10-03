// 3D model of the goBILDA BIOBUZZ (2026-27) StarterBot, mecanum version.
// Proportions measured from goBILDA's side/top reference renders. Built in millimetres.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

const stage = document.getElementById('robot-stage');
if (stage) {
  try { init(stage); } catch (err) { console.error(err); stage.classList.add('no-webgl'); }
}

function init(stage) {
  const canvas = stage.querySelector('canvas');
  const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;
  const coarse = matchMedia('(pointer: coarse)').matches;
  let visible = true;

  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.setClearColor(0x000000, 0);

  const scene = new THREE.Scene();
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  scene.environmentIntensity = 0.55;

  // Scene units: 1 unit = 100 mm
  const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 100);
  const target = new THREE.Vector3(0, 1.15, 0);
  const viewDir = new THREE.Vector3(0.66, 0.46, 0.6).normalize();

  scene.add(new THREE.HemisphereLight(0xcfe3ff, 0x0b0d10, 0.75));
  const key = new THREE.DirectionalLight(0xffffff, 2.2);
  key.position.set(5, 10, 6);
  key.castShadow = true;
  key.shadow.mapSize.set(1024, 1024);
  Object.assign(key.shadow.camera, { left: -4, right: 4, top: 4, bottom: -4, near: 1, far: 30 });
  key.shadow.normalBias = 0.02;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x5cb2ff, 1.2);
  rim.position.set(-7, 5, -6);
  scene.add(rim);

  function canvasTex(w, h, draw, color = true) {
    const c = document.createElement('canvas');
    c.width = w; c.height = h;
    draw(c.getContext('2d'), w, h);
    const t = new THREE.CanvasTexture(c);
    t.colorSpace = color ? THREE.SRGBColorSpace : THREE.NoColorSpace;
    t.anisotropy = 4;
    return t;
  }

  // goBILDA-style hole pattern as an alpha map
  const holeTex = canvasTex(64, 64, (g) => {
    g.fillStyle = '#fff'; g.fillRect(0, 0, 64, 64);
    g.fillStyle = '#000'; g.beginPath(); g.arc(32, 32, 17, 0, Math.PI * 2); g.fill();
  }, false);
  holeTex.wrapS = holeTex.wrapT = THREE.RepeatWrapping;

  // Box-project UVs so the hole pitch is constant on every face (geometry in mm)
  function worldUV(geo, pitch) {
    const p = geo.attributes.position, n = geo.attributes.normal, uv = geo.attributes.uv;
    for (let i = 0; i < p.count; i++) {
      const ax = Math.abs(n.getX(i)), ay = Math.abs(n.getY(i));
      const x = p.getX(i), y = p.getY(i), z = p.getZ(i);
      if (ax > 0.5) uv.setXY(i, z / pitch + 0.5, y / pitch + 0.5);
      else if (ay > 0.5) uv.setXY(i, x / pitch + 0.5, z / pitch + 0.5);
      else uv.setXY(i, x / pitch + 0.5, y / pitch + 0.5);
    }
    uv.needsUpdate = true;
  }

  const M = {
    alu: new THREE.MeshStandardMaterial({ color: 0xaeb5be, metalness: 0.9, roughness: 0.32 }),
    perf: new THREE.MeshStandardMaterial({ color: 0xa3abb5, metalness: 0.9, roughness: 0.34, alphaMap: holeTex, alphaTest: 0.5, side: THREE.DoubleSide }),
    dark: new THREE.MeshStandardMaterial({ color: 0x1b1e23, metalness: 0.2, roughness: 0.6 }),
    rubber: new THREE.MeshStandardMaterial({ color: 0x141619, roughness: 0.92 }),
    yellow: new THREE.MeshStandardMaterial({ color: 0xf5c518, metalness: 0.25, roughness: 0.42 }),
    can: new THREE.MeshStandardMaterial({ color: 0x23262b, metalness: 0.5, roughness: 0.4 }),
    battery: new THREE.MeshStandardMaterial({ color: 0x55595f, metalness: 0.1, roughness: 0.7 }),
    hubOrange: new THREE.MeshStandardMaterial({ color: 0xe8611a, roughness: 0.5 }),
    servo: new THREE.MeshStandardMaterial({ color: 0x3a3d42, roughness: 0.55 }),
  };

  function mesh(geo, mat, parent, x = 0, y = 0, z = 0) {
    const m = new THREE.Mesh(geo, mat);
    m.position.set(x, y, z);
    m.castShadow = m.receiveShadow = true;
    parent.add(m);
    return m;
  }
  function box(w, h, d, mat, parent, x, y, z) {
    const geo = new THREE.BoxGeometry(w, h, d);
    if (mat === M.perf) worldUV(geo, 12);
    return mesh(geo, mat, parent, x, y, z);
  }
  // Cylinder along an axis ('x', 'y' or 'z')
  function cyl(r, len, mat, axis, parent, x, y, z, seg = 32) {
    const geo = new THREE.CylinderGeometry(r, r, len, seg);
    if (axis === 'x') geo.rotateZ(Math.PI / 2);
    if (axis === 'z') geo.rotateX(Math.PI / 2);
    return mesh(geo, mat, parent, x, y, z);
  }
  // goBILDA Yellow Jacket: yellow gearbox + black motor can
  function yellowJacket(axis, parent, x, y, z, dir = 1) {
    const g = new THREE.Group();
    g.position.set(x, y, z);
    parent.add(g);
    const off = (d) => (axis === 'x' ? [d * dir, 0, 0] : axis === 'y' ? [0, d * dir, 0] : [0, 0, d * dir]);
    cyl(18, 50, M.yellow, axis, g, ...off(25));
    cyl(17, 55, M.can, axis, g, ...off(77));
    cyl(12, 10, M.dark, axis, g, ...off(109));
    return g;
  }

  // Robot is built in mm, then scaled to scene units
  const robot = new THREE.Group();
  robot.scale.setScalar(0.01);
  scene.add(robot);

  // ---- Chassis: two 48 mm U-channel rails + cross members ----
  for (const sx of [-1, 1]) box(48, 48, 390, M.perf, robot, sx * 157, 54, -34);
  box(266, 48, 48, M.perf, robot, 0, 54, -205);
  box(266, 48, 48, M.perf, robot, 0, 54, -118);
  box(266, 3, 210, M.perf, robot, 0, 80, -5); // deck plate

  // ---- 104 mm mecanum wheels ----
  const rollerGeo = new THREE.CapsuleGeometry(9, 30, 4, 12);
  const up = new THREE.Vector3(0, 1, 0);
  for (const sx of [-1, 1]) for (const [sz, wz] of [[1, 112], [-1, -155]]) {
    const w = new THREE.Group();
    w.position.set(sx * 209, 52, wz);
    robot.add(w);
    for (const s of [-1, 1]) cyl(32, 4, M.yellow, 'x', w, s * 19, 0, 0, 10); // side plates
    cyl(22, 40, M.dark, 'x', w, 0, 0, 0, 20);
    const hand = -sx * sz;
    for (let i = 0; i < 10; i++) {
      const a = (i / 10) * Math.PI * 2;
      const r = mesh(rollerGeo, M.rubber, w, 0, Math.cos(a) * 43, Math.sin(a) * 43);
      r.quaternion.setFromUnitVectors(up, new THREE.Vector3(hand, -Math.sin(a), Math.cos(a)).normalize());
    }
    cyl(4, 70, M.alu, 'x', robot, sx * 180, 52, wz, 8); // axle
  }

  // ---- Drive motors: rear stand vertical (bevel drive), front lie inside the rails ----
  for (const sx of [-1, 1]) {
    yellowJacket('y', robot, sx * 157, 78, -151);
    yellowJacket('z', robot, sx * 152, 54, 85, -1);
  }

  // ---- Battery + REV Control Hub on the deck ----
  box(58, 48, 135, M.battery, robot, -78, 106, 18);
  box(103, 22, 143, M.dark, robot, 72, 93, 32);
  box(103, 3, 26, M.hubOrange, robot, 72, 105, 90);

  // ---- Launcher tower: uprights, flywheel, hood ----
  for (const sx of [-1, 1]) box(48, 226, 48, M.perf, robot, sx * 110, 191, -117);
  for (const sx of [-1, 1]) box(3, 190, 150, M.perf, robot, sx * 64, 175, -62);
  box(250, 48, 48, M.perf, robot, 0, 186, 12); // cross channel
  const flywheel = new THREE.Group();
  flywheel.position.set(0, 171, -63);
  robot.add(flywheel);
  cyl(46, 30, M.rubber, 'x', flywheel, 0, 0, 0, 40);
  cyl(36, 32, M.yellow, 'x', flywheel, 0, 0, 0, 28);
  cyl(14, 34, M.dark, 'x', flywheel, 0, 0, 0, 16);
  cyl(4, 140, M.alu, 'x', robot, 0, 171, -63, 8);
  yellowJacket('x', robot, 62, 171, -63);
  // Hood: slopes from the top of the tower down toward the front
  const hood = box(170, 3, 214, M.perf, robot, 0, 240, -8);
  hood.rotation.x = Math.atan2(127, 172);
  // Rear loading ramp
  const ramp = box(92, 3, 112, M.perf, robot, 0, 76, -183);
  ramp.rotation.x = -Math.atan2(70, 85);

  // ---- Intake: Gecko wheel roller across the front, driven through a big gear ----
  for (const sx of [-1, 1]) box(24, 112, 48, M.perf, robot, sx * 145, 108, 146);
  box(300, 48, 48, M.perf, robot, 0, 140, 150);
  const intake = new THREE.Group();
  intake.position.set(0, 58, 158);
  robot.add(intake);
  cyl(4, 280, M.alu, 'x', intake, 0, 0, 0, 8);
  for (let i = 0; i < 7; i++) {
    const gw = cyl(36, 22, M.rubber, 'x', intake, -102 + i * 34, 0, 0, 20);
    gw.rotation.x = i * 0.5;
  }
  const gear = new THREE.Group();
  gear.position.set(170, 145, 118);
  robot.add(gear);
  cyl(42, 6, M.perf, 'x', gear, 0, 0, 0, 48);
  cyl(10, 8, M.alu, 'x', gear, 0, 0, 0, 12);
  cyl(8, 330, M.dark, 'x', robot, 0, 60, 95, 12); // front bumper bar

  // ---- Front corner sweepers: wheels on servos that pull POLLEN in ----
  const sweepers = [];
  for (const sx of [-1, 1]) {
    const sw = new THREE.Group();
    sw.position.set(sx * 158, 14, 196);
    robot.add(sw);
    cyl(34, 26, M.rubber, 'y', sw, 0, 0, 0, 24);
    cyl(14, 28, M.dark, 'y', sw, 0, 0, 0, 12);
    sweepers.push(sw);
    box(40, 38, 21, M.servo, robot, sx * 158, 50, 196);
    box(46, 3, 21, M.yellow, robot, sx * 158, 62, 196);
  }

  // ---- Team number plates (FTC rule: team number on the robot) ----
  const plateTex = canvasTex(512, 176, () => {});
  function drawPlate() {
    const g = plateTex.image.getContext('2d');
    g.fillStyle = '#1f6fe5'; g.fillRect(0, 0, 512, 176);
    g.fillStyle = '#fff';
    g.font = '800 132px "Archivo", "Arial Black", Arial, sans-serif';
    g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText('30819', 256, 96);
    plateTex.needsUpdate = true;
  }
  drawPlate();
  if (document.fonts) document.fonts.ready.then(drawPlate);
  const plateMat = new THREE.MeshStandardMaterial({ map: plateTex, roughness: 0.6 });
  for (const sx of [-1, 1]) {
    const p = mesh(new THREE.PlaneGeometry(140, 48), plateMat, robot, sx * 135, 250, -117);
    p.rotation.y = sx * Math.PI / 2;
  }

  // ---- Floor: a few FTC foam tiles (24 in = 610 mm) fading into the page ----
  const floorTex = canvasTex(512, 512, (g, W) => {
    g.fillStyle = '#3d4249'; g.fillRect(0, 0, W, W);
    g.strokeStyle = 'rgba(15,17,20,.9)'; g.lineWidth = 2;
    for (let s = 0; s <= 3; s++) {
      const v = (s * W) / 3;
      g.beginPath(); g.moveTo(v, 0); g.lineTo(v, W); g.stroke();
      g.beginPath(); g.moveTo(0, v); g.lineTo(W, v); g.stroke();
    }
    g.globalCompositeOperation = 'destination-in';
    const grad = g.createRadialGradient(W / 2, W / 2, W * 0.12, W / 2, W / 2, W * 0.5);
    grad.addColorStop(0, 'rgba(0,0,0,1)');
    grad.addColorStop(1, 'rgba(0,0,0,0)');
    g.fillStyle = grad; g.fillRect(0, 0, W, W);
  });
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(18.3, 18.3), new THREE.MeshStandardMaterial({ map: floorTex, roughness: 0.95, transparent: true }));
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);

  const controls = new OrbitControls(camera, canvas);
  controls.target.copy(target);
  controls.enableZoom = false;
  controls.enablePan = false;
  controls.enableDamping = true;
  controls.minPolarAngle = 0.4;
  controls.maxPolarAngle = 1.4;
  controls.autoRotate = !reduceMotion;
  controls.autoRotateSpeed = 0.5;
  if (coarse) controls.enabled = false;

  const clock = new THREE.Clock();
  function render() {
    const dt = Math.min(clock.getDelta(), 0.05);
    if (!reduceMotion) {
      flywheel.rotation.x -= dt * 14;
      intake.rotation.x -= dt * 6;
      gear.rotation.x += dt * 2.5;
      sweepers.forEach((s, i) => { s.rotation.y += dt * 3 * (i ? 1 : -1); });
    }
    controls.update();
    renderer.render(scene, camera);
  }

  function resize() {
    const r = stage.getBoundingClientRect();
    const w = Math.max(1, r.width), h = Math.max(1, r.height);
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    const dist = 15.5 / Math.min(1.15, Math.max(0.75, camera.aspect));
    camera.position.copy(viewDir).multiplyScalar(dist).add(target);
    camera.updateProjectionMatrix();
    if (!visible) render();
  }
  new ResizeObserver(resize).observe(stage);
  resize();

  const setRunning = (on) => { renderer.setAnimationLoop(on ? render : null); if (on) clock.getDelta(); };
  new IntersectionObserver(([e]) => { visible = e.isIntersecting && !document.hidden; setRunning(visible); }, { threshold: 0.05 }).observe(stage);
  document.addEventListener('visibilitychange', () => { visible = !document.hidden; setRunning(visible); });

  stage.classList.add('ready');
  render();
}
