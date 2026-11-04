import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

const byId = (id) => document.getElementById(id);
const ui = {
  viewport: byId('viewport'),
  loading: byId('loadingOverlay'),
  error: byId('errorOverlay'),
  errorMessage: byId('errorMessage'),
  scenario: byId('scenarioSelect'),
  scenarioDescription: byId('scenarioDescription'),
  methodPinv: byId('methodPinv'),
  methodDls: byId('methodDls'),
  showDesired: byId('showDesired'),
  showTrail: byId('showTrail'),
  viewIso: byId('viewIso'),
  viewTop: byId('viewTop'),
  viewFront: byId('viewFront'),
  sceneTitle: byId('sceneTitle'),
  frameBadge: byId('frameBadge'),
  play: byId('playButton'),
  reset: byId('resetButton'),
  progress: byId('progress'),
  timeCurrent: byId('timeCurrent'),
  timeTotal: byId('timeTotal'),
  speed: byId('speedSelect'),
  loop: byId('loopToggle'),
  liveError: byId('liveError'),
  trackingState: byId('trackingState'),
  actualXYZ: byId('actualXYZ'),
  desiredXYZ: byId('desiredXYZ'),
  joint1: byId('joint1'),
  joint2: byId('joint2'),
  joint3: byId('joint3'),
  sigmaMin: byId('sigmaMin'),
  satNow: byId('satNow'),
  caseRMSE: byId('caseRMSE'),
  caseSaturations: byId('caseSaturations'),
  caseMaxError: byId('caseMaxError'),
  sourceName: byId('sourceName'),
};

const scenarioNames = {
  regular: 'Trayectoria regular',
  near_singular: 'Cerca de singularidad',
  near_singular_bias: 'Singularidad + sesgo de 1 mm',
};
const scenarioDescriptions = {
  regular: 'Seguimiento sin perturbación cerca de una postura regular.',
  near_singular: 'El codo se aproxima a una configuración singular.',
  near_singular_bias: 'Se añade un sesgo suave de medición en X de hasta 1 mm.',
};

let dataset = null;
let activeCase = null;
let method = 'pinv';
let elapsed = 0;
let speed = 1;
let playing = false;
let lastTick = null;
let activeView = 'iso';

let scene;
let camera;
let renderer;
let controls;
let robot;
let desiredLine = null;
let trailLine = null;
let desiredMarker = null;
let errorConnector = null;
let saturationRing = null;

const baseTarget = new THREE.Vector3(0.18, 0, 0.01);
const yAxis = new THREE.Vector3(0, 1, 0);
const point = (xyz) => new THREE.Vector3(xyz[0], xyz[1], xyz[2]);
const lerpNumber = (a, b, alpha) => a + (b - a) * alpha;
const lerpArray = (a, b, alpha) => a.map((value, index) => lerpNumber(value, b[index], alpha));

function createLabel(text, color, position) {
  const canvas = document.createElement('canvas');
  canvas.width = 128;
  canvas.height = 128;
  const context = canvas.getContext('2d');
  context.fillStyle = 'rgba(0, 48, 73, 0.88)';
  context.beginPath();
  context.arc(64, 64, 43, 0, Math.PI * 2);
  context.fill();
  context.strokeStyle = color;
  context.lineWidth = 3;
  context.stroke();
  context.fillStyle = color;
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.font = 'bold 55px Segoe UI, sans-serif';
  context.fillText(text, 64, 63);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false }));
  sprite.position.copy(position);
  sprite.scale.set(0.07, 0.07, 1);
  sprite.renderOrder = 10;
  scene.add(sprite);
}

function makeSegment(radius, material) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 18), material);
  mesh.castShadow = true;
  scene.add(mesh);
  return mesh;
}

function makeSphere(radius, material) {
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius, 24, 16), material);
  mesh.castShadow = true;
  scene.add(mesh);
  return mesh;
}

function setSegment(mesh, a, b) {
  const direction = new THREE.Vector3().subVectors(b, a);
  mesh.position.copy(a).add(b).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(yAxis, direction.clone().normalize());
  mesh.scale.set(1, direction.length(), 1);
}

function buildScene() {
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(42, 1, 0.01, 10);
  camera.up.set(0, 0, 1);

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x003049, 0);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.25;
  ui.viewport.prepend(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.07;
  controls.minDistance = 0.34;
  controls.maxDistance = 3;
  controls.maxPolarAngle = Math.PI * 0.96;
  controls.target.copy(baseTarget);

  scene.add(new THREE.HemisphereLight(0xfdf0d5, 0x003049, 2.1));
  const key = new THREE.DirectionalLight(0xfdf0d5, 2.4);
  key.position.set(0.65, -0.55, 1.4);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x669bbc, 1.4);
  fill.position.set(-0.7, 0.8, 0.5);
  scene.add(fill);

  const grid = new THREE.GridHelper(1.8, 36, 0x669bbc, 0x29566e);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -0.55;
  grid.material.transparent = true;
  grid.material.opacity = 0.4;
  scene.add(grid);

  const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x66889a, metalness: 0.62, roughness: 0.28 });
  const ringMaterial = new THREE.MeshStandardMaterial({ color: 0xfdf0d5, metalness: 0.68, roughness: 0.25 });
  const orangeMaterial = new THREE.MeshStandardMaterial({ color: 0xc1121f, metalness: 0.47, roughness: 0.25, emissive: 0x780000, emissiveIntensity: 0.25 });
  const cyanMaterial = new THREE.MeshStandardMaterial({ color: 0x669bbc, metalness: 0.48, roughness: 0.23, emissive: 0x003049, emissiveIntensity: 0.25 });
  const jointMaterial = new THREE.MeshStandardMaterial({ color: 0xfdf0d5, metalness: 0.74, roughness: 0.22 });
  const effectorMaterial = new THREE.MeshStandardMaterial({ color: 0xfdf0d5, metalness: 0.3, roughness: 0.2, emissive: 0xc1121f, emissiveIntensity: 0.42 });

  const pedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.068, 0.07, 40), baseMaterial);
  pedestal.rotation.x = Math.PI / 2;
  pedestal.position.z = -0.034;
  scene.add(pedestal);
  const pedestalRim = new THREE.Mesh(new THREE.TorusGeometry(0.055, 0.004, 8, 48), ringMaterial);
  pedestalRim.position.z = 0.002;
  scene.add(pedestalRim);

  robot = {
    segments: [
      makeSegment(0.023, baseMaterial),
      makeSegment(0.019, orangeMaterial),
      makeSegment(0.014, cyanMaterial),
    ],
    joints: [
      makeSphere(0.027, ringMaterial),
      makeSphere(0.03, jointMaterial),
      makeSphere(0.024, jointMaterial),
      makeSphere(0.019, effectorMaterial),
    ],
  };

  const shoulderRing = new THREE.Mesh(new THREE.TorusGeometry(0.032, 0.004, 8, 42), orangeMaterial);
  shoulderRing.position.z = 0.04;
  scene.add(shoulderRing);

  const axisOrigin = new THREE.Vector3(-0.12, -0.16, -0.52);
  scene.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), axisOrigin, 0.17, 0xc1121f, 0.035, 0.018));
  scene.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), axisOrigin, 0.17, 0x669bbc, 0.035, 0.018));
  scene.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), axisOrigin, 0.17, 0xfdf0d5, 0.035, 0.018));
  createLabel('X', '#C1121F', axisOrigin.clone().add(new THREE.Vector3(0.205, 0, 0)));
  createLabel('Y', '#669BBC', axisOrigin.clone().add(new THREE.Vector3(0, 0.205, 0)));
  createLabel('Z', '#FDF0D5', axisOrigin.clone().add(new THREE.Vector3(0, 0, 0.205)));

  desiredMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.012, 22, 14),
    new THREE.MeshBasicMaterial({ color: 0x669bbc, transparent: true, opacity: 0.96 }),
  );
  scene.add(desiredMarker);
  const markerHalo = new THREE.Mesh(
    new THREE.SphereGeometry(0.019, 20, 12),
    new THREE.MeshBasicMaterial({ color: 0x669bbc, transparent: true, opacity: 0.17, depthWrite: false }),
  );
  desiredMarker.add(markerHalo);

  const connectorGeometry = new THREE.BufferGeometry();
  connectorGeometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
  errorConnector = new THREE.Line(connectorGeometry, new THREE.LineBasicMaterial({ color: 0xfdf0d5, transparent: true, opacity: 0.78 }));
  scene.add(errorConnector);

  saturationRing = new THREE.Mesh(
    new THREE.TorusGeometry(0.042, 0.005, 8, 48),
    new THREE.MeshBasicMaterial({ color: 0xc1121f, transparent: true, opacity: 0.95, depthTest: false }),
  );
  saturationRing.visible = false;
  scene.add(saturationRing);

  setCameraPreset('iso');
  resize();
  new ResizeObserver(resize).observe(ui.viewport);
}

function setCameraPreset(preset) {
  activeView = preset;
  const target = baseTarget;
  controls.target.copy(target);
  if (preset === 'top') camera.position.set(target.x + 0.15, target.y - 0.12, target.z + 1.42);
  else if (preset === 'front') camera.position.set(target.x + 0.06, target.y - 1.38, target.z + 0.25);
  else camera.position.set(target.x + 0.9, target.y - 1.06, target.z + 0.68);
  camera.lookAt(target);
  controls.update();
  for (const [name, element] of [['iso', ui.viewIso], ['top', ui.viewTop], ['front', ui.viewFront]]) {
    element.classList.toggle('active', name === activeView);
  }
}

function resize() {
  if (!renderer) return;
  const width = Math.max(1, ui.viewport.clientWidth);
  const height = Math.max(1, ui.viewport.clientHeight);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
}

function removeCasePaths() {
  for (const line of [desiredLine, trailLine]) {
    if (!line) continue;
    scene.remove(line);
    line.geometry.dispose();
    line.material.dispose();
  }
  desiredLine = null;
  trailLine = null;
}

function buildCasePaths() {
  removeCasePaths();
  const desiredPoints = activeCase.samples.map((sample) => point(sample.desired));
  const actualPoints = activeCase.samples.map((sample) => point(sample.actual));

  desiredLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(desiredPoints),
    new THREE.LineDashedMaterial({ color: 0x669bbc, dashSize: 0.016, gapSize: 0.009, transparent: true, opacity: 0.82, depthWrite: false }),
  );
  desiredLine.computeLineDistances();
  desiredLine.renderOrder = 3;
  scene.add(desiredLine);

  trailLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(actualPoints),
    new THREE.LineBasicMaterial({ color: 0xc1121f, transparent: true, opacity: 0.97, depthWrite: false }),
  );
  trailLine.renderOrder = 4;
  scene.add(trailLine);
  updateLayerVisibility();
}

function updateLayerVisibility() {
  if (desiredLine) desiredLine.visible = ui.showDesired.checked;
  if (desiredMarker) desiredMarker.visible = ui.showDesired.checked;
  if (errorConnector) errorConnector.visible = ui.showDesired.checked;
  if (trailLine) trailLine.visible = ui.showTrail.checked;
}

function sampleAt(time) {
  const samples = activeCase.samples;
  if (time <= samples[0].t) return { sample: samples[0], index: 0 };
  const last = samples.length - 1;
  if (time >= samples[last].t) return { sample: samples[last], index: last };

  let low = 0;
  let high = last;
  while (high - low > 1) {
    const middle = (low + high) >> 1;
    if (samples[middle].t <= time) low = middle;
    else high = middle;
  }
  const a = samples[low];
  const b = samples[high];
  const alpha = Math.min(1, Math.max(0, (time - a.t) / (b.t - a.t)));
  return {
    index: low,
    sample: {
      t: time,
      joints: a.joints.map((value, index) => lerpArray(value, b.joints[index], alpha)),
      desired: lerpArray(a.desired, b.desired, alpha),
      actual: lerpArray(a.actual, b.actual, alpha),
      q: lerpArray(a.q, b.q, alpha),
      error_mm: lerpNumber(a.error_mm, b.error_mm, alpha),
      sigma_min: lerpNumber(a.sigma_min, b.sigma_min, alpha),
      saturated: alpha < 0.5 ? Boolean(a.saturated) : Boolean(b.saturated),
    },
  };
}

function displayTime(time) {
  return time.toFixed(2) + ' s';
}

function displayCoord(value) {
  return value.map((part) => (part < 0 ? '' : '+') + part.toFixed(3)).join(' / ');
}

function renderPose() {
  if (!activeCase) return;
  const { sample, index } = sampleAt(elapsed);
  const joints = sample.joints.map(point);
  for (let i = 0; i < 3; i++) setSegment(robot.segments[i], joints[i], joints[i + 1]);
  for (let i = 0; i < 4; i++) robot.joints[i].position.copy(joints[i]);
  desiredMarker.position.copy(point(sample.desired));
  saturationRing.position.copy(joints[3]);
  saturationRing.visible = Boolean(sample.saturated);

  const connectorPositions = errorConnector.geometry.attributes.position;
  connectorPositions.setXYZ(0, ...sample.actual);
  connectorPositions.setXYZ(1, ...sample.desired);
  connectorPositions.needsUpdate = true;

  trailLine.geometry.setDrawRange(0, Math.max(1, index + 1));

  const duration = activeCase.samples[activeCase.samples.length - 1].t;
  ui.progress.value = String(elapsed);
  ui.progress.style.setProperty('--progress', (duration ? 100 * elapsed / duration : 0).toFixed(2) + '%');
  ui.timeCurrent.textContent = displayTime(elapsed);
  ui.frameBadge.textContent = 'FRAME ' + String(index + 1).padStart(3, '0') + ' / ' + activeCase.samples.length;
  ui.liveError.textContent = sample.error_mm.toFixed(2);
  ui.actualXYZ.textContent = displayCoord(sample.actual);
  ui.desiredXYZ.textContent = displayCoord(sample.desired);
  ui.joint1.textContent = (sample.q[0] * 180 / Math.PI).toFixed(1);
  ui.joint2.textContent = (sample.q[1] * 180 / Math.PI).toFixed(1);
  ui.joint3.textContent = (sample.q[2] * 180 / Math.PI).toFixed(1);
  ui.sigmaMin.textContent = sample.sigma_min.toExponential(2) + ' m/rad';
  ui.satNow.textContent = sample.saturated ? 'SÍ' : 'NO';
  ui.satNow.classList.toggle('safe', !sample.saturated);
  ui.satNow.classList.toggle('danger', Boolean(sample.saturated));
  ui.trackingState.classList.toggle('warning', Boolean(sample.saturated));
  ui.trackingState.innerHTML = sample.saturated
    ? '<span class="tracking-dot"></span> Límite de velocidad alcanzado'
    : '<span class="tracking-dot"></span> Seguimiento activo';
}

function setPlaying(next) {
  playing = next;
  ui.play.textContent = playing ? 'Ⅱ' : '▶';
  ui.play.setAttribute('aria-label', playing ? 'Pausar' : 'Reproducir');
  ui.play.title = playing ? 'Pausar' : 'Reproducir';
}

function setMethod(next) {
  method = next;
  ui.methodPinv.classList.toggle('active', method === 'pinv');
  ui.methodDls.classList.toggle('active', method === 'dls');
  ui.methodPinv.setAttribute('aria-pressed', String(method === 'pinv'));
  ui.methodDls.setAttribute('aria-pressed', String(method === 'dls'));
  selectCase();
}

function selectCase() {
  if (!dataset) return;
  const scenario = ui.scenario.value;
  const id = scenario + '_' + method;
  const found = dataset.cases.find((candidate) => candidate.id === id);
  if (!found || !Array.isArray(found.samples) || found.samples.length < 2) {
    throw new Error('Faltan datos de la trayectoria ' + id + '.');
  }
  activeCase = found;
  elapsed = 0;
  ui.sceneTitle.textContent = scenarioNames[scenario] || found.label || scenario;
  ui.scenarioDescription.textContent = scenarioDescriptions[scenario] || '';
  const duration = found.samples[found.samples.length - 1].t;
  ui.progress.max = String(duration);
  ui.timeTotal.textContent = displayTime(duration);
  ui.caseRMSE.textContent = Number(found.metrics?.rmse_mm ?? 0).toFixed(3) + ' mm';
  ui.caseSaturations.textContent = String(found.metrics?.saturated_samples ?? '—');
  ui.caseMaxError.textContent = Number(found.metrics?.max_error_mm ?? 0).toFixed(2) + ' mm';
  buildCasePaths();
  renderPose();
  setPlaying(true);
}

function tick(timestamp) {
  if (lastTick == null) lastTick = timestamp;
  const delta = Math.min((timestamp - lastTick) / 1000, 0.15);
  lastTick = timestamp;
  if (activeCase && playing) {
    const duration = activeCase.samples[activeCase.samples.length - 1].t;
    elapsed += delta * speed;
    if (elapsed >= duration) {
      if (ui.loop.checked) elapsed %= duration;
      else {
        elapsed = duration;
        setPlaying(false);
      }
    }
    renderPose();
  }
  if (controls) controls.update();
  if (renderer) renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

function wireControls() {
  ui.scenario.addEventListener('change', selectCase);
  ui.methodPinv.addEventListener('click', () => setMethod('pinv'));
  ui.methodDls.addEventListener('click', () => setMethod('dls'));
  ui.showDesired.addEventListener('change', updateLayerVisibility);
  ui.showTrail.addEventListener('change', updateLayerVisibility);
  ui.viewIso.addEventListener('click', () => setCameraPreset('iso'));
  ui.viewTop.addEventListener('click', () => setCameraPreset('top'));
  ui.viewFront.addEventListener('click', () => setCameraPreset('front'));
  ui.play.addEventListener('click', () => {
    const duration = activeCase?.samples.at(-1)?.t ?? 0;
    if (!playing && elapsed >= duration) elapsed = 0;
    setPlaying(!playing);
  });
  ui.reset.addEventListener('click', () => {
    elapsed = 0;
    renderPose();
  });
  ui.progress.addEventListener('input', () => {
    elapsed = Number(ui.progress.value);
    renderPose();
  });
  ui.speed.addEventListener('change', () => { speed = Number(ui.speed.value) || 1; });
  window.addEventListener('keydown', (event) => {
    if (event.target instanceof HTMLInputElement || event.target instanceof HTMLSelectElement || event.target instanceof HTMLButtonElement) return;
    if (event.code === 'Space' && activeCase) {
      event.preventDefault();
      setPlaying(!playing);
    } else if (event.code === 'ArrowRight' && activeCase) {
      elapsed = Math.min(Number(ui.progress.max), elapsed + 0.1);
      renderPose();
    } else if (event.code === 'ArrowLeft' && activeCase) {
      elapsed = Math.max(0, elapsed - 0.1);
      renderPose();
    }
  });
}

async function start() {
  try {
    buildScene();
    wireControls();
    requestAnimationFrame(tick);
    const response = await fetch('./data.json', { cache: 'no-store' });
    if (!response.ok) throw new Error('No se encontró data.json (HTTP ' + response.status + ').');
    dataset = await response.json();
    if (!Array.isArray(dataset.cases) || !dataset.cases.length) throw new Error('El archivo data.json no contiene trayectorias.');
    ui.sourceName.textContent = 'Fuente: ' + (dataset.source_folder || 'resultados locales') + ' · Datos generados en este equipo';
    selectCase();
    ui.loading.hidden = true;
  } catch (error) {
    ui.loading.hidden = true;
    ui.error.hidden = false;
    ui.errorMessage.textContent = error?.message || String(error);
    console.error(error);
  }
}

start();
