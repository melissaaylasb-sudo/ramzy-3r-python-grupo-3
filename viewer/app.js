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
  controlsTitle: byId('controlsTitle'),
  controlsSubtitle: byId('controlsSubtitle'),
  modeTrajectory: byId('modeTrajectory'),
  modeInverse: byId('modeInverse'),
  trajectoryControls: byId('trajectoryControls'),
  inverseControls: byId('inverseControls'),
  ikCase: byId('ikCaseSelect'),
  ikCaseDescription: byId('ikCaseDescription'),
  ikCustomX: byId('ikCustomX'),
  ikCustomY: byId('ikCustomY'),
  ikCustomZ: byId('ikCustomZ'),
  ikSolveTarget: byId('ikSolveTarget'),
  ikCustomStatus: byId('ikCustomStatus'),
  ikSolutionsTitle: byId('ikSolutionsTitle'),
  ikSolutions: byId('ikSolutions'),
  ikTourControls: byId('ikTourControls'),
  ikTourPlay: byId('ikTourPlay'),
  ikTourStatus: byId('ikTourStatus'),
  ikPrevious: byId('ikPrevious'),
  ikNext: byId('ikNext'),
  ikYawWrap: byId('ikYawWrap'),
  ikYaw: byId('ikYaw'),
  ikYawValue: byId('ikYawValue'),
  methodPinv: byId('methodPinv'),
  methodDls: byId('methodDls'),
  showDesired: byId('showDesired'),
  showTrail: byId('showTrail'),
  viewIso: byId('viewIso'),
  viewTop: byId('viewTop'),
  viewFront: byId('viewFront'),
  sceneTitle: byId('sceneTitle'),
  sceneLegend: byId('sceneLegend'),
  ikSceneHud: byId('ikSceneHud'),
  ikHudTitle: byId('ikHudTitle'),
  ikHudAngles: byId('ikHudAngles'),
  ikHudDetail: byId('ikHudDetail'),
  frameBadge: byId('frameBadge'),
  playback: byId('playback'),
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
  trajectoryTelemetry: byId('trajectoryTelemetry'),
  inverseTelemetry: byId('inverseTelemetry'),
  ikBranchEquation: byId('ikBranchEquation'),
  ikStatus: byId('ikStatus'),
  ikTargetXYZ: byId('ikTargetXYZ'),
  ikReachStatus: byId('ikReachStatus'),
  ikRangeMarker: byId('ikRangeMarker'),
  ikRho: byId('ikRho'),
  ikHeight: byId('ikHeight'),
  ikDistance: byId('ikDistance'),
  ikMeasureLayout: byId('ikMeasureLayout'),
  ikTriangle: byId('ikTriangle'),
  ikNumericDistance: byId('ikNumericDistance'),
  ikD: byId('ikD'),
  ikDLabel: byId('ikDLabel'),
  ikNumericD: byId('ikNumericD'),
  ikQ3FromD: byId('ikQ3FromD'),
  ikRLabel: byId('ikRLabel'),
  ikSelectedR: byId('ikSelectedR'),
  ikElbowBranch: byId('ikElbowBranch'),
  ikBranchMeaning: byId('ikBranchMeaning'),
  ikMathExplanation: byId('ikMathExplanation'),
  ikSelectedLabel: byId('ikSelectedLabel'),
  ikSelectedPosture: byId('ikSelectedPosture'),
  ikSelectedAngles: byId('ikSelectedAngles'),
  ikQ1: byId('ikQ1'),
  ikQ2: byId('ikQ2'),
  ikQ3: byId('ikQ3'),
  ikAngleChange: byId('ikAngleChange'),
  ikAngleChangeLabel: byId('ikAngleChangeLabel'),
  ikAngleChangeValues: byId('ikAngleChangeValues'),
  ikResidualLabel: byId('ikResidualLabel'),
  ikResidual: byId('ikResidual'),
  ikSelectedNote: byId('ikSelectedNote'),
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
let mode = 'trajectory';
let activeInverseCase = null;
let customInverseCase = null;
let inverseSolutionIndex = 0;
let trajectoryWasPlaying = false;
let inverseLiveQ = null;
let inverseTransition = null;
let inverseTourPlaying = false;
let inverseTourHold = 0;

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
let inverseReachOuter = null;
let inverseReachInner = null;
let inverseGhost = null;
let inverseMeasurementGuide = null;

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

function makeMeasurementSprite(text, color, position) {
  const canvas = document.createElement('canvas');
  canvas.width = 512;
  canvas.height = 128;
  const context = canvas.getContext('2d');
  context.fillStyle = 'rgba(2, 36, 74, 0.93)';
  context.fillRect(4, 10, 504, 108);
  context.strokeStyle = color;
  context.lineWidth = 5;
  context.strokeRect(4, 10, 504, 108);
  context.fillStyle = '#ffffff';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.font = 'bold 48px Segoe UI, sans-serif';
  context.fillText(text, 256, 64);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false }));
  sprite.position.copy(position);
  sprite.scale.set(0.19, 0.048, 1);
  sprite.renderOrder = 11;
  return sprite;
}

function updateInverseMeasurementGuide(current) {
  if (inverseMeasurementGuide) {
    scene.remove(inverseMeasurementGuide);
    inverseMeasurementGuide.traverse((object) => {
      object.geometry?.dispose();
      object.material?.map?.dispose();
      object.material?.dispose();
    });
  }
  inverseMeasurementGuide = new THREE.Group();
  const shoulder = new THREE.Vector3(0, 0, dataset.geometry_m[0]);
  const projection = new THREE.Vector3(current.target[0], current.target[1], dataset.geometry_m[0]);
  const target = point(current.target);
  const addMeasure = (start, end, color, caption, position) => {
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([start, end]),
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.82, depthTest: false }),
    );
    line.renderOrder = 9;
    inverseMeasurementGuide.add(line);
    inverseMeasurementGuide.add(makeMeasurementSprite(caption, '#' + color.toString(16).padStart(6, '0'), position));
  };
  if (current.rho > 1e-8) {
    addMeasure(shoulder, projection, 0x72bad6,
      `ρ ${displayNumber(current.rho, 3)} m`,
      shoulder.clone().lerp(projection, 0.48).add(new THREE.Vector3(0, 0, 0.045)));
  }
  if (Math.abs(current.h) > 1e-8) {
    addMeasure(projection, target, 0xa18bdc,
      `h ${displayNumber(current.h, 3)} m`,
      projection.clone().lerp(target, 0.55).add(new THREE.Vector3(0.055, 0.005, 0)));
  }
  const side = new THREE.Vector3(-current.target[1], current.target[0], 0);
  if (side.length() > 0) side.normalize().multiplyScalar(0.055);
  else side.set(0.055, 0, 0);
  addMeasure(shoulder, target, 0x5bd1a9,
    `d ${displayNumber(current.d, 3)} m`,
    shoulder.clone().lerp(target, 0.53).add(side));
  scene.add(inverseMeasurementGuide);
  inverseMeasurementGuide.visible = mode === 'inverse';
}

function makeSegment(radius, material) {
  const link = new THREE.Group();
  const body = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 24), material);
  body.castShadow = true;
  link.add(body);
  if (radius < 0.023) {
    const collarMaterial = new THREE.MeshStandardMaterial({ color: 0xc6dce8, metalness: 0.76, roughness: 0.22 });
    const insetMaterial = new THREE.MeshStandardMaterial({ color: 0x123b54, metalness: 0.45, roughness: 0.35 });
    for (const end of [-0.39, 0.39]) {
      const collar = new THREE.Mesh(new THREE.CylinderGeometry(radius * 1.23, radius * 1.23, 0.065, 24), collarMaterial);
      collar.position.y = end;
      link.add(collar);
    }
    const inlay = new THREE.Mesh(new THREE.BoxGeometry(radius * 0.26, 0.58, radius * 0.62), insetMaterial);
    inlay.position.x = radius * 0.96;
    link.add(inlay);
  }
  scene.add(link);
  return link;
}

function makeSphere(radius, material, axle = false) {
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius, 24, 16), material);
  mesh.castShadow = true;
  if (axle) {
    const axleMaterial = new THREE.MeshStandardMaterial({ color: 0x133c56, metalness: 0.68, roughness: 0.25 });
    const capMaterial = new THREE.MeshStandardMaterial({ color: 0xd5e8f0, metalness: 0.72, roughness: 0.2 });
    const spindle = new THREE.Mesh(new THREE.CylinderGeometry(radius * 0.77, radius * 0.77, radius * 2.1, 24), axleMaterial);
    mesh.add(spindle);
    for (const side of [-1, 1]) {
      const cap = new THREE.Mesh(new THREE.CylinderGeometry(radius * 0.55, radius * 0.55, 0.003, 24), capMaterial);
      cap.position.y = side * radius * 1.06;
      mesh.add(cap);
    }
  }
  scene.add(mesh);
  return mesh;
}

function setSegment(mesh, a, b) {
  const direction = new THREE.Vector3().subVectors(b, a);
  mesh.position.copy(a).add(b).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(yAxis, direction.clone().normalize());
  mesh.scale.set(1, direction.length(), 1);
}

function makeReachRings(radius, opacity) {
  const group = new THREE.Group();
  const material = new THREE.MeshBasicMaterial({ color: 0xa5d0f0, transparent: true, opacity, depthWrite: false });
  for (const [x, y] of [[0, 0], [Math.PI / 2, 0], [0, Math.PI / 2]]) {
    const ring = new THREE.Mesh(new THREE.TorusGeometry(radius, 0.0012, 4, 96), material);
    ring.rotation.set(x, y, 0);
    group.add(ring);
  }
  group.position.z = 0.04;
  group.visible = false;
  scene.add(group);
  return group;
}

function buildScene() {
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(42, 1, 0.01, 10);
  camera.up.set(0, 0, 1);

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x02244a, 0);
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

  scene.add(new THREE.HemisphereLight(0xffffff, 0x02244a, 2.1));
  const key = new THREE.DirectionalLight(0xffffff, 2.4);
  key.position.set(0.65, -0.55, 1.4);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xa5d0f0, 1.4);
  fill.position.set(-0.7, 0.8, 0.5);
  scene.add(fill);

  const grid = new THREE.GridHelper(1.8, 36, 0x4a91bb, 0x004b62);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -0.115;
  grid.material.transparent = true;
  grid.material.opacity = 0.22;
  scene.add(grid);

  const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x004b62, metalness: 0.62, roughness: 0.28 });
  const ringMaterial = new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.68, roughness: 0.25 });
  const oceanMaterial = new THREE.MeshStandardMaterial({ color: 0x4a91bb, metalness: 0.47, roughness: 0.25, emissive: 0x02244a, emissiveIntensity: 0.2 });
  const skyMaterial = new THREE.MeshStandardMaterial({ color: 0xa5d0f0, metalness: 0.48, roughness: 0.23, emissive: 0x004b62, emissiveIntensity: 0.2 });
  const jointMaterial = new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.74, roughness: 0.22 });
  const effectorMaterial = new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.3, roughness: 0.2, emissive: 0x4a91bb, emissiveIntensity: 0.42 });

  const deckMaterial = new THREE.MeshStandardMaterial({ color: 0x092d44, metalness: 0.42, roughness: 0.49 });
  const trimMaterial = new THREE.MeshStandardMaterial({ color: 0x72bad6, metalness: 0.61, roughness: 0.24, emissive: 0x11455e, emissiveIntensity: 0.19 });
  const floorHalo = new THREE.Mesh(
    new THREE.RingGeometry(0.18, 0.61, 72),
    new THREE.MeshBasicMaterial({ color: 0x4a91bb, transparent: true, opacity: 0.055, side: THREE.DoubleSide, depthWrite: false }),
  );
  floorHalo.position.z = -0.113;
  scene.add(floorHalo);
  const floorOutline = new THREE.Mesh(
    new THREE.TorusGeometry(0.61, 0.0013, 4, 96),
    new THREE.MeshBasicMaterial({ color: 0xa5d0f0, transparent: true, opacity: 0.28 }),
  );
  floorOutline.position.z = -0.111;
  scene.add(floorOutline);

  const foundation = new THREE.Mesh(new THREE.CylinderGeometry(0.145, 0.155, 0.022, 48), deckMaterial);
  foundation.rotation.x = Math.PI / 2;
  foundation.position.z = -0.099;
  foundation.castShadow = true;
  scene.add(foundation);
  const foundationRim = new THREE.Mesh(new THREE.TorusGeometry(0.145, 0.0022, 6, 64), trimMaterial);
  foundationRim.position.z = -0.087;
  scene.add(foundationRim);
  for (let i = 0; i < 8; i++) {
    const angle = i * Math.PI / 4;
    const bolt = new THREE.Mesh(new THREE.CylinderGeometry(0.0045, 0.0045, 0.003, 10), ringMaterial);
    bolt.rotation.x = Math.PI / 2;
    bolt.position.set(0.123 * Math.cos(angle), 0.123 * Math.sin(angle), -0.086);
    scene.add(bolt);
  }

  const housing = new THREE.Mesh(new THREE.CylinderGeometry(0.096, 0.13, 0.042, 48), baseMaterial);
  housing.rotation.x = Math.PI / 2;
  housing.position.z = -0.066;
  housing.castShadow = true;
  scene.add(housing);
  const bearing = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.022, 48), deckMaterial);
  bearing.rotation.x = Math.PI / 2;
  bearing.position.z = -0.037;
  scene.add(bearing);
  for (const z of [-0.048, -0.025]) {
    const bearingRim = new THREE.Mesh(new THREE.TorusGeometry(0.091, 0.003, 8, 64), ringMaterial);
    bearingRim.position.z = z;
    scene.add(bearingRim);
  }
  for (let i = 0; i < 24; i++) {
    const angle = i * Math.PI / 12;
    const tick = new THREE.Mesh(
      new THREE.BoxGeometry(i % 3 === 0 ? 0.012 : 0.006, 0.0016, 0.0015),
      i % 3 === 0 ? trimMaterial : ringMaterial,
    );
    tick.position.set(0.076 * Math.cos(angle), 0.076 * Math.sin(angle), -0.024);
    tick.rotation.z = angle;
    scene.add(tick);
  }

  const pedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.052, 0.061, 0.028, 40), baseMaterial);
  pedestal.rotation.x = Math.PI / 2;
  pedestal.position.z = -0.011;
  scene.add(pedestal);
  const pedestalRim = new THREE.Mesh(new THREE.TorusGeometry(0.052, 0.003, 8, 48), trimMaterial);
  pedestalRim.position.z = 0.003;
  scene.add(pedestalRim);

  const yawIndicator = new THREE.Group();
  const yawBar = new THREE.Mesh(new THREE.BoxGeometry(0.027, 0.006, 0.002), trimMaterial);
  yawBar.position.set(0.069, 0, -0.023);
  yawIndicator.add(yawBar);
  const yawTip = new THREE.Mesh(new THREE.ConeGeometry(0.005, 0.011, 8), trimMaterial);
  yawTip.rotation.z = -Math.PI / 2;
  yawTip.position.set(0.091, 0, -0.023);
  yawIndicator.add(yawTip);
  scene.add(yawIndicator);

  // En inversa, esta flecha hace visible q₁ incluso cuando dos ternas
  // colocan hombro, codo y punta en los mismos puntos XYZ.
  const ikYawArrow = new THREE.Group();
  const arrowMaterial = new THREE.MeshBasicMaterial({ color: 0xffd36b, depthTest: false });
  const arrowStem = new THREE.Mesh(new THREE.BoxGeometry(0.065, 0.007, 0.003), arrowMaterial);
  arrowStem.position.set(0.104, 0, 0.015);
  arrowStem.renderOrder = 9;
  ikYawArrow.add(arrowStem);
  const arrowTip = new THREE.Mesh(new THREE.ConeGeometry(0.009, 0.018, 10), arrowMaterial);
  arrowTip.rotation.z = -Math.PI / 2;
  arrowTip.position.set(0.145, 0, 0.015);
  arrowTip.renderOrder = 9;
  ikYawArrow.add(arrowTip);
  ikYawArrow.visible = false;
  scene.add(ikYawArrow);

  robot = {
    segments: [
      makeSegment(0.023, baseMaterial),
      makeSegment(0.019, oceanMaterial),
      makeSegment(0.014, skyMaterial),
    ],
    joints: [
      makeSphere(0.027, ringMaterial),
      makeSphere(0.03, jointMaterial, true),
      makeSphere(0.024, jointMaterial, true),
      makeSphere(0.019, effectorMaterial),
    ],
    yawIndicator,
    ikYawArrow,
  };

  const shoulderRing = new THREE.Mesh(new THREE.TorusGeometry(0.032, 0.004, 8, 42), oceanMaterial);
  shoulderRing.position.z = 0.04;
  scene.add(shoulderRing);

  const axisOrigin = new THREE.Vector3(-0.28, -0.24, -0.105);
  scene.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), axisOrigin, 0.17, 0x4a91bb, 0.035, 0.018));
  scene.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), axisOrigin, 0.17, 0xa5d0f0, 0.035, 0.018));
  scene.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), axisOrigin, 0.17, 0xffffff, 0.035, 0.018));
  createLabel('X', '#4A91BB', axisOrigin.clone().add(new THREE.Vector3(0.205, 0, 0)));
  createLabel('Y', '#A5D0F0', axisOrigin.clone().add(new THREE.Vector3(0, 0.205, 0)));
  createLabel('Z', '#FFFFFF', axisOrigin.clone().add(new THREE.Vector3(0, 0, 0.205)));

  desiredMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.012, 22, 14),
    new THREE.MeshBasicMaterial({ color: 0xa5d0f0, transparent: true, opacity: 0.96 }),
  );
  scene.add(desiredMarker);
  const markerHalo = new THREE.Mesh(
    new THREE.SphereGeometry(0.019, 20, 12),
    new THREE.MeshBasicMaterial({ color: 0xa5d0f0, transparent: true, opacity: 0.17, depthWrite: false }),
  );
  desiredMarker.add(markerHalo);

  const connectorGeometry = new THREE.BufferGeometry();
  connectorGeometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
  errorConnector = new THREE.Line(connectorGeometry, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.78 }));
  scene.add(errorConnector);

  saturationRing = new THREE.Mesh(
    new THREE.TorusGeometry(0.042, 0.005, 8, 48),
    new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.95, depthTest: false }),
  );
  saturationRing.visible = false;
  scene.add(saturationRing);

  inverseReachOuter = makeReachRings(0.53, 0.4);
  inverseReachInner = makeReachRings(0.05, 0.7);
  const ghostMaterial = new THREE.MeshBasicMaterial({ color: 0xffbc7c, transparent: true, opacity: 0.62, depthWrite: false });
  inverseGhost = {
    segments: [0, 1].map(() => {
      const mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, 1, 14), ghostMaterial);
      scene.add(mesh);
      return mesh;
    }),
    elbow: new THREE.Mesh(new THREE.SphereGeometry(0.012, 16, 10), ghostMaterial),
  };
  scene.add(inverseGhost.elbow);
  inverseGhost.segments.forEach((mesh) => { mesh.visible = false; });
  inverseGhost.elbow.visible = false;

  setCameraPreset('iso');
  resize();
  new ResizeObserver(resize).observe(ui.viewport);
}

function setCameraPreset(preset) {
  activeView = preset;
  const target = mode === 'inverse' ? new THREE.Vector3(0.12, 0, 0.22) : baseTarget;
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
    new THREE.LineDashedMaterial({ color: 0xa5d0f0, dashSize: 0.016, gapSize: 0.009, transparent: true, opacity: 0.82, depthWrite: false }),
  );
  desiredLine.computeLineDistances();
  desiredLine.renderOrder = 3;
  scene.add(desiredLine);

  trailLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(actualPoints),
    new THREE.LineBasicMaterial({ color: 0x4a91bb, transparent: true, opacity: 0.97, depthWrite: false }),
  );
  trailLine.renderOrder = 4;
  scene.add(trailLine);
  updateLayerVisibility();
}

function updateLayerVisibility() {
  if (mode === 'inverse') {
    if (desiredLine) desiredLine.visible = false;
    if (trailLine) trailLine.visible = false;
    if (desiredMarker) desiredMarker.visible = true;
    if (errorConnector) errorConnector.visible = Boolean(activeInverseCase && (inverseTransition || !activeInverseCase.solutions.length));
    return;
  }
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

function applyRobotPose(jointCoordinates, q1) {
  const joints = jointCoordinates.map(point);
  for (let i = 0; i < 3; i++) setSegment(robot.segments[i], joints[i], joints[i + 1]);
  for (let i = 0; i < 4; i++) robot.joints[i].position.copy(joints[i]);
  robot.yawIndicator.rotation.z = q1;
  robot.ikYawArrow.rotation.z = q1;
  robot.joints[1].rotation.z = q1;
  robot.joints[2].rotation.z = q1;
  return joints;
}

function renderPose() {
  if (mode !== 'trajectory' || !activeCase) return;
  const { sample, index } = sampleAt(elapsed);
  const joints = applyRobotPose(sample.joints, sample.q[0]);
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

const ikDescriptions = {
  regular: 'Punto alcanzable con el codo flexionado.',
  codo_extend: 'El brazo llega a su borde más lejano.',
  codo_plegado: 'El brazo llega a su borde más cercano.',
  eje_base: 'El punto está sobre el eje de la base.',
  interseccion: 'El punto está sobre el eje y en el borde lejano.',
  inalcanzable_exterior: 'El punto queda más lejos que el brazo.',
  inalcanzable_interior: 'El punto queda demasiado cerca del hombro.',
};

function inverseCaseDescription(current) {
  if (current.id !== 'custom') return ikDescriptions[current.id] || '';
  return current.solutions.length
    ? 'Punto elegido por ti. Modelo ideal: sin límites articulares ni obstáculos.'
    : 'Punto elegido por ti. No hay ángulos reales porque queda fuera del alcance ideal.';
}

const displayNumber = (value, decimals = 3) => Number(value).toFixed(decimals).replace('.', ',');
const displayDegrees = (value) => displayNumber(Math.abs(value) < 0.005 ? 0 : value, 2) + '°';
const displaySignedDegrees = (value) => (value > 0.005 ? '+' : '') + displayDegrees(value);

// La misma FK cerrada de ramzy3r/kinematics.py; permite mostrar ángulos
// intermedios sin interpolar puntos XYZ ni fingir que son soluciones inversas.
function jointsFromAngles([q1, q2, q3]) {
  const [L1, L2, L3] = dataset.geometry_m;
  const base = [0, 0, 0];
  const shoulder = [0, 0, L1];
  const elbowRadius = L2 * Math.cos(q2);
  const elbow = [elbowRadius * Math.cos(q1), elbowRadius * Math.sin(q1), L1 + L2 * Math.sin(q2)];
  const tipRadius = elbowRadius + L3 * Math.cos(q2 + q3);
  const tip = [tipRadius * Math.cos(q1), tipRadius * Math.sin(q1), L1 + L2 * Math.sin(q2) + L3 * Math.sin(q2 + q3)];
  return [base, shoulder, elbow, tip];
}

function wrapAngle(angle) {
  const turn = 2 * Math.PI;
  return ((angle + Math.PI) % turn + turn) % turn - Math.PI;
}

function makeInverseLimitPose(target, rho, h, d) {
  const [, L2, L3] = dataset.geometry_m;
  const q = [
    rho > 0 ? Math.atan2(target[1], target[0]) : 0,
    d > 0 ? Math.atan2(h, rho) : 0,
    d > L2 + L3 ? 0 : Math.PI,
  ];
  const joints = jointsFromAngles(q);
  return {
    q,
    joints,
    actual: joints[3],
    distance_to_target_m: Math.hypot(...joints[3].map((value, index) => value - target[index])),
  };
}

function solveIdealInverse(target, tolerance = 1e-12) {
  const [L1, L2, L3] = dataset.geometry_m;
  const [x, y, z] = target;
  const rho = Math.hypot(x, y);
  const h = z - L1;
  const d = Math.hypot(rho, h);
  const dMin = Math.abs(L2 - L3);
  const dMax = L2 + L3;
  const rawD = (d ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3);
  const current = {
    id: 'custom',
    label: 'Punto elegido · IK ideal',
    target: target.slice(),
    rho,
    h,
    d,
    D: rawD,
    solutions: [],
    posture_count: 0,
    base_free: rho <= tolerance,
  };

  if (d < dMin - tolerance || d > dMax + tolerance) {
    current.limit_pose = makeInverseLimitPose(target, rho, h, d);
    return current;
  }

  const boundedDistance = Math.min(dMax, Math.max(dMin, d));
  const D = Math.max(-1, Math.min(1, (boundedDistance ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)));
  const sinQ3Abs = Math.sqrt(Math.max(0, 1 - D ** 2));
  const onAxis = rho <= tolerance;
  const radialSigns = onAxis ? [1] : [1, -1];
  const elbowSigns = sinQ3Abs <= tolerance ? [0] : [1, -1];
  const elbowPositions = [];
  current.D = D;

  for (const radialSign of radialSigns) {
    const r = radialSign * rho;
    for (const elbowSign of elbowSigns) {
      const q3 = Math.atan2((elbowSign || 1) * sinQ3Abs, D);
      const alpha = Math.atan2(h, r);
      const beta = Math.atan2(L3 * Math.sin(q3), L2 + L3 * D);
      const q2 = alpha - beta;
      let q1 = onAxis ? 0 : Math.atan2(y, x);
      if (radialSign === -1) q1 += Math.PI;
      const q = [wrapAngle(q1), wrapAngle(q2), wrapAngle(q3)];
      const joints = jointsFromAngles(q);
      const elbow = joints[2];
      let postureIndex = elbowPositions.findIndex((other) => Math.hypot(...elbow.map((value, index) => value - other[index])) <= 1e-9);
      if (postureIndex < 0) {
        postureIndex = elbowPositions.length;
        elbowPositions.push(elbow);
      }
      current.solutions.push({
        q,
        q_deg: q.map((angle) => angle * 180 / Math.PI),
        joints,
        radial_sign: radialSign,
        elbow_sign: elbowSign,
        base_free: onAxis,
        residual_m: Math.hypot(...joints[3].map((value, index) => value - target[index])),
        posture: String.fromCharCode(65 + postureIndex),
      });
    }
  }
  current.posture_count = elbowPositions.length;
  return current;
}

function ensureCustomCaseOption() {
  let option = ui.ikCase.querySelector('option[value="custom"]');
  if (!option) {
    option = document.createElement('option');
    option.value = 'custom';
    ui.ikCase.append(option);
  }
  option.textContent = 'Punto elegido · IK ideal';
}

function setCustomTargetStatus(message, type = '') {
  ui.ikCustomStatus.textContent = message;
  ui.ikCustomStatus.className = 'ik-custom-status' + (type ? ` ${type}` : '');
}

function solveCustomTarget() {
  if (!dataset) return;
  const target = [ui.ikCustomX, ui.ikCustomY, ui.ikCustomZ]
    .map((input) => Number(input.value.trim().replace(',', '.')));
  if (!target.every(Number.isFinite)) {
    setCustomTargetStatus('Escribe valores numéricos para X, Y y Z.', 'error');
    return;
  }
  customInverseCase = solveIdealInverse(target);
  ensureCustomCaseOption();
  ui.ikCase.value = 'custom';
  activateInverseCase(customInverseCase);
  const count = customInverseCase.solutions.length;
  setCustomTargetStatus(
    count
      ? `${count} ${count === 1 ? 'terna calculada' : 'ternas calculadas'} para este punto. Selecciona una para animarla.`
      : '0 ternas: este punto queda fuera del alcance ideal.',
    count ? 'success' : 'error',
  );
}

function ikExplanation(current) {
  if (!current.solutions.length) {
    return {
      short: 'Fuera del alcance → 0 soluciones',
      parts: ['Fuera de alcance', '→', '0 soluciones'],
      status: 'Ninguna terna coloca la punta en ese punto.',
      long: 'La ley de cosenos produce |D| > 1. Entonces 1 − D² es negativo y no existe un ángulo real q₃ que lleve la punta a este objetivo.',
    };
  }
  if (current.base_free && current.solutions.length === 1) {
    return {
      short: 'Base libre · 1 codo → 1 terna mostrada',
      parts: ['Base libre', '+', '1 codo', '→', '1 mostrada'],
      status: 'q₁ puede tomar cualquier ángulo; se muestra 1 codo.',
      long: 'ρ = 0 deja libre el giro de la base. En el borde, |D| = 1 y las dos ramas de q₃ se unen. El CSV guarda q₁ = 0° como representante, aunque la base puede girar.',
    };
  }
  if (current.base_free) {
    return {
      short: 'Base libre · 2 codos → 2 ternas mostradas',
      parts: ['Base libre', '+', '2 codos', '→', '2 mostradas'],
      status: 'q₁ puede tomar cualquier ángulo; se muestran 2 codos.',
      long: 'ρ = 0 deja libre el giro de la base. Como |D| < 1, el signo ± de √(1 − D²) da dos ángulos de codo. El CSV muestra ambas ramas con q₁ = 0°; el deslizador enseña otros giros válidos.',
    };
  }
  if (current.solutions.length === 2) {
    return {
      short: '2 giros de base × 1 codo = 2 ternas',
      parts: ['2 bases', '×', '1 codo', '=', '2 ternas'],
      status: 'Las dos ternas dibujan la misma forma.',
      long: 'ρ > 0 permite r = +ρ o −ρ. En el borde, |D| = 1 y √(1 − D²) = 0: los codos positivo y negativo coinciden. Las dos ternas dibujan una sola silueta.',
    };
  }
  return {
    short: '2 giros de base × 2 codos = 4 ternas',
    parts: ['2 bases', '×', '2 codos', '=', '4 ternas'],
    status: 'Forma A: ternas 1 y 4. Forma B: ternas 2 y 3.',
    long: 'ρ > 0 permite r = +ρ con q₁ = φ o r = −ρ con q₁ = φ + 180°. Como |D| < 1, √(1 − D²) tiene signos ± y produce dos q₃. Para cada combinación se calcula q₂. Las 4 ternas se superponen por parejas: 2 siluetas.',
  };
}

function showIkCount(parts, description) {
  ui.ikBranchEquation.replaceChildren();
  ui.ikBranchEquation.setAttribute('aria-label', description);
  for (const [index, part] of parts.entries()) {
    const node = document.createElement(index === parts.length - 1 ? 'strong' : 'span');
    node.textContent = part;
    if (part === '×' || part === '=' || part === '→' || part === '+') node.className = 'ik-operator';
    ui.ikBranchEquation.append(node);
  }
}

function populateInverseCases() {
  ui.ikCase.replaceChildren();
  for (const current of dataset.inverse_cases) {
    const option = document.createElement('option');
    option.value = current.id;
    option.textContent = current.label;
    ui.ikCase.append(option);
  }
  if (customInverseCase) ensureCustomCaseOption();
  ui.ikCase.value = 'regular';
  const regular = dataset.inverse_cases.find((current) => current.id === 'regular');
  if (regular) {
    [ui.ikCustomX, ui.ikCustomY, ui.ikCustomZ].forEach((input, index) => {
      input.value = regular.target[index].toFixed(6);
    });
  }
}

function populateInverseSolutions() {
  ui.ikSolutions.replaceChildren();
  const solutions = activeInverseCase.solutions;
  ui.ikSolutionsTitle.textContent = solutions.length
    ? `${solutions.length} ${solutions.length === 1 ? 'TERNA' : 'TERNAS'} · UN MISMO OBJETIVO`
    : '0 SOLUCIONES · FUERA DEL ALCANCE';
  if (!solutions.length) {
    const empty = document.createElement('div');
    empty.className = 'ik-empty';
    empty.textContent = 'Ninguna terna llega a este punto.';
    ui.ikSolutions.append(empty);
    return;
  }
  for (const [index, solution] of solutions.entries()) {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'ik-branch';
    button.dataset.index = String(index);
    const heading = document.createElement('strong');
    heading.textContent = `Terna ${index + 1} · forma ${solution.posture}`;
    const branches = document.createElement('span');
    branches.className = 'ik-branch-description';
    const signedR = solution.radial_sign * activeInverseCase.rho;
    branches.textContent = `r = ${signedR < -1e-8 ? '−' : '+'}${displayNumber(Math.abs(signedR), 3)} m · codo ${solution.elbow_sign > 0 ? '+' : solution.elbow_sign < 0 ? '−' : 'alineado'}`;
    const angles = document.createElement('span');
    angles.className = 'ik-branch-angles';
    angles.innerHTML = `<span><small>BASE q₁</small><b>${displayDegrees(solution.q_deg[0])}</b></span><span><small>HOMBRO q₂</small><b>${displayDegrees(solution.q_deg[1])}</b></span><span><small>CODO q₃</small><b>${displayDegrees(solution.q_deg[2])}</b></span>`;
    button.append(heading, branches, angles);
    ui.ikSolutions.append(button);
  }
}

function updateInverseTourControls() {
  const count = activeInverseCase?.solutions.length || 0;
  const moving = Boolean(inverseTransition);
  ui.ikTourControls.hidden = count < 2;
  ui.ikPrevious.disabled = inverseSolutionIndex === 0 && !moving;
  ui.ikNext.disabled = inverseSolutionIndex === count - 1 && !moving;
  ui.ikTourPlay.textContent = inverseTourPlaying
    ? '■ Detener recorrido'
    : `▶ Ver las ${count} posturas`;
  ui.ikTourStatus.textContent = moving
    ? `Terna ${inverseTransition.fromIndex + 1} → ${inverseTransition.toIndex + 1}`
    : `Terna ${inverseSolutionIndex + 1} de ${count}`;
}

function stopInverseTour() {
  inverseTourPlaying = false;
  inverseTourHold = 0;
  inverseTransition = null;
  if (activeInverseCase?.solutions.length) {
    inverseLiveQ = activeInverseCase.solutions[inverseSolutionIndex].q.slice();
    if (activeInverseCase.base_free) inverseLiveQ[0] = Number(ui.ikYaw.value) * Math.PI / 180;
  }
  updateInverseTourControls();
}

function moveToInverseSolution(index) {
  const solutions = activeInverseCase.solutions;
  if (index < 0 || index >= solutions.length) return;
  const target = solutions[index];
  const toQ = target.q.slice();
  if (activeInverseCase.base_free) toQ[0] = Number(ui.ikYaw.value) * Math.PI / 180;
  if (index === inverseSolutionIndex && !inverseTransition) return;
  const fromQ = inverseLiveQ?.slice() || solutions[inverseSolutionIndex].q.slice();
  inverseTransition = {
    fromIndex: inverseSolutionIndex,
    toIndex: index,
    fromQ,
    toQ,
    elapsed: 0,
    duration: 1.7,
  };
  updateInverseTourControls();
  renderInverseScene();
}

function tickInverse(delta) {
  if (mode !== 'inverse' || !activeInverseCase?.solutions.length) return;
  if (inverseTransition) {
    const transition = inverseTransition;
    transition.elapsed = Math.min(transition.duration, transition.elapsed + delta);
    const fraction = transition.elapsed / transition.duration;
    const eased = fraction * fraction * (3 - 2 * fraction);
    inverseLiveQ = lerpArray(transition.fromQ, transition.toQ, eased);
    if (fraction >= 1) {
      inverseSolutionIndex = transition.toIndex;
      inverseLiveQ = transition.toQ.slice();
      inverseTransition = null;
      inverseTourHold = inverseTourPlaying ? 1.3 : 0;
      updateInverseTourControls();
    }
    renderInverseScene();
  } else if (inverseTourPlaying) {
    inverseTourHold -= delta;
    if (inverseTourHold <= 0) {
      if (inverseSolutionIndex + 1 < activeInverseCase.solutions.length) {
        moveToInverseSolution(inverseSolutionIndex + 1);
      } else {
        inverseTourPlaying = false;
        updateInverseTourControls();
      }
    }
  }
}

function rotateJointsAboutBase(joints, angle) {
  const cosine = Math.cos(angle);
  const sine = Math.sin(angle);
  return joints.map(([x, y, z]) => [x * cosine - y * sine, x * sine + y * cosine, z]);
}

function showInverseGhost(selected, yawOffset) {
  const alternative = activeInverseCase.solutions.find((solution) => solution.posture !== selected?.posture);
  const visible = Boolean(selected && alternative && !inverseTransition);
  inverseGhost.segments.forEach((mesh) => { mesh.visible = visible; });
  inverseGhost.elbow.visible = visible;
  if (!alternative) return;
  const coordinates = activeInverseCase.base_free
    ? rotateJointsAboutBase(alternative.joints, yawOffset)
    : alternative.joints;
  const joints = coordinates.map(point);
  setSegment(inverseGhost.segments[0], joints[1], joints[2]);
  setSegment(inverseGhost.segments[1], joints[2], joints[3]);
  inverseGhost.elbow.position.copy(joints[2]);
}

function renderInverseScene() {
  if (mode !== 'inverse' || !activeInverseCase) return;
  const current = activeInverseCase;
  const selected = current.solutions[inverseSolutionIndex] || null;
  const moving = Boolean(inverseTransition);
  const pairIndex = selected
    ? current.solutions.findIndex((solution, index) => index !== inverseSolutionIndex && solution.posture === selected.posture)
    : -1;
  const pose = selected || current.limit_pose;
  const qNow = inverseLiveQ || pose.q;
  const yawDegrees = current.base_free ? Number(ui.ikYaw.value) : qNow[0] * 180 / Math.PI;
  const yawOffset = current.base_free ? yawDegrees * Math.PI / 180 - selected.q[0] : 0;
  const jointCoordinates = selected ? jointsFromAngles(qNow) : pose.joints;
  const q1 = qNow[0];
  applyRobotPose(jointCoordinates, q1);
  desiredMarker.position.copy(point(current.target));
  desiredMarker.material.color.setHex(selected ? 0xa5d0f0 : 0xffbc7c);
  desiredMarker.material.depthTest = Boolean(selected);
  desiredMarker.renderOrder = selected ? 0 : 8;
  desiredMarker.children[0].material.depthTest = Boolean(selected);
  saturationRing.visible = false;
  inverseReachOuter.visible = true;
  inverseReachInner.visible = current.d < Math.abs(dataset.geometry_m[1] - dataset.geometry_m[2]);
  showInverseGhost(selected, yawOffset);
  const connectorPositions = errorConnector.geometry.attributes.position;
  connectorPositions.setXYZ(0, ...jointCoordinates[3]);
  connectorPositions.setXYZ(1, ...current.target);
  connectorPositions.needsUpdate = true;
  errorConnector.geometry.computeBoundingSphere();
  errorConnector.material.color.setHex(moving || !selected ? 0xffbc7c : 0xffffff);
  ui.ikHudTitle.textContent = moving
    ? `Terna ${inverseTransition.fromIndex + 1} → terna ${inverseTransition.toIndex + 1}`
    : selected
    ? `Terna ${inverseSolutionIndex + 1} de ${current.solutions.length} · forma ${selected.posture}`
    : 'Fuera del alcance · 0 soluciones';
  ui.ikHudAngles.textContent = selected
    ? `q₁ ${displayDegrees(qNow[0] * 180 / Math.PI)} · q₂ ${displayDegrees(qNow[1] * 180 / Math.PI)} · q₃ ${displayDegrees(qNow[2] * 180 / Math.PI)}`
    : `Faltan ${displayNumber(current.limit_pose.distance_to_target_m * 1000, 1)} mm`;
  ui.ikHudDetail.textContent = moving
    ? 'Movimiento ilustrativo: la punta puede separarse del objetivo entre ternas.'
    : selected
    ? current.base_free
      ? 'q₁ puede tomar cualquier ángulo.'
      : pairIndex >= 0
        ? `La terna ${pairIndex + 1} dibuja la misma forma con otros ángulos.`
        : 'La punta llega al mismo punto XYZ.'
    : 'La postura dibujada es el límite, no una solución.';
  ui.sceneLegend.textContent = moving
    ? 'Línea ámbar: separación temporal de la punta al objetivo'
    : selected
    ? 'Flecha: giro de base · ámbar: otra forma · esfera: objetivo'
    : 'Esfera ámbar: objetivo · línea: distancia que falta';
  updateLayerVisibility();
  renderInverseTelemetry(selected, pose, qNow, jointCoordinates);
}

function renderInverseTelemetry(selected, pose, qNow, jointCoordinates) {
  const current = activeInverseCase;
  const count = current.solutions.length;
  const moving = Boolean(inverseTransition);
  const explanation = ikExplanation(current);
  showIkCount(explanation.parts, explanation.short);
  ui.ikMathExplanation.textContent = explanation.long;
  ui.ikStatus.textContent = explanation.status;
  ui.ikTargetXYZ.textContent = current.target.map((part) => displayNumber(part, 3)).join(' / ');
  ui.ikRho.textContent = displayNumber(current.rho, 3) + ' m';
  ui.ikHeight.textContent = displayNumber(current.h, 6) + ' m';
  ui.ikDistance.textContent = displayNumber(current.d, 6) + ' m';
  const degenerateTriangle = current.rho < 1e-8 || Math.abs(current.h) < 1e-8;
  ui.ikTriangle.style.display = degenerateTriangle ? 'none' : '';
  ui.ikMeasureLayout.classList.toggle('degenerate', degenerateTriangle);
  ui.ikNumericDistance.textContent = `d = √(${displayNumber(current.rho, 3)}² + ${displayNumber(current.h, 6)}²) = ${displayNumber(current.d, 6)} m`;
  ui.ikRangeMarker.style.left = `${Math.min(100, Math.max(0, current.d / 0.6 * 100))}%`;
  ui.ikRangeMarker.classList.toggle('outside', !count);
  ui.ikReachStatus.textContent = count
    ? 'Dentro del alcance ideal (0,050–0,530 m).'
    : 'Fuera del alcance ideal (0,050–0,530 m).';
  ui.ikD.textContent = displayNumber(current.D, 3);
  ui.ikDLabel.textContent = count ? 'D = cos(q₃) · sin unidad' : 'D calculado · fuera de [−1, 1]';
  const [ , L2, L3 ] = dataset.geometry_m;
  ui.ikNumericD.textContent = `D = (${displayNumber(current.d, 6)}² − ${displayNumber(L2, 3)}² − ${displayNumber(L3, 3)}²) / (2·${displayNumber(L2, 3)}·${displayNumber(L3, 3)}) = ${displayNumber(current.D, 3)}`;
  ui.ikQ3FromD.textContent = count
    ? Math.abs(current.D) >= 1 - 1e-10
      ? `q₃ = arccos(${displayNumber(current.D, 3)}) = ${displayDegrees(current.solutions[0].q_deg[2])} (un codo)`
      : `q₃ = ± arccos(${displayNumber(current.D, 3)}) = ±${displayDegrees(Math.acos(Math.max(-1, Math.min(1, current.D))) * 180 / Math.PI)}`
    : 'D fuera de [−1, 1] ⇒ no existe q₃ real.';
  const branch = moving ? current.solutions[inverseTransition.toIndex] : selected;
  ui.ikRLabel.textContent = moving ? 'r al llegar a la terna' : 'r de esta terna';
  ui.ikSelectedR.textContent = branch
    ? (branch.radial_sign * current.rho < -1e-8 ? '−' : '+') + displayNumber(current.rho, 3) + ' m'
    : 'sin solución';
  ui.ikElbowBranch.textContent = branch ? displayDegrees(branch.q_deg[2]) : 'sin ángulo real';
  ui.ikBranchMeaning.textContent = !branch
    ? 'D queda fuera de [−1, 1]: no existe q₃ real para este punto.'
    : moving
      ? 'Estos son los valores de llegada; los ángulos de abajo cambian durante el paso.'
      : current.base_free
        ? 'ρ = r = 0: la base puede girar; D fija las opciones del codo.'
        : 'ρ es distancia positiva; r usa + o −. D fija |q₃|; su signo elige el codo.';
  ui.ikSelectedLabel.textContent = moving ? 'ÁNGULOS DURANTE EL MOVIMIENTO' : selected ? 'COMPRUEBA LOS ÁNGULOS' : 'LÍMITE DEL BRAZO';
  ui.ikSelectedPosture.textContent = moving
    ? `De terna ${inverseTransition.fromIndex + 1} a terna ${inverseTransition.toIndex + 1}`
    : selected
    ? `Terna ${inverseSolutionIndex + 1} · forma ${selected.posture}`
    : 'Postura ilustrativa, no solución';
  ui.ikSelectedAngles.hidden = !selected;
  const qDegrees = qNow.map((angle) => angle * 180 / Math.PI);
  ui.ikQ1.textContent = displayNumber(qDegrees[0], 2);
  ui.ikQ2.textContent = displayNumber(qDegrees[1], 2);
  ui.ikQ3.textContent = displayNumber(qDegrees[2], 2);
  const residual = selected ? Math.hypot(...jointCoordinates[3].map((value, index) => value - current.target[index])) : 0;
  ui.ikResidualLabel.textContent = moving ? 'Punta al objetivo, ahora' : selected ? 'Diferencia al volver a XYZ' : 'Distancia que falta';
  ui.ikResidual.textContent = selected
    ? (residual < 1e-9 ? '≈ 0 mm' : displayNumber(residual * 1000, moving ? 1 : 6) + ' mm')
    : displayNumber(pose.distance_to_target_m * 1000, 1) + ' mm';
  ui.ikAngleChange.hidden = !selected || inverseSolutionIndex === 0 || moving;
  if (selected && inverseSolutionIndex > 0 && !moving) {
    const prior = current.solutions[inverseSolutionIndex - 1];
    ui.ikAngleChangeLabel.textContent = `CAMBIO DESDE TERNA ${inverseSolutionIndex}`;
    ui.ikAngleChangeValues.textContent = selected.q_deg.map((value, index) => `Δq${index + 1} ${displaySignedDegrees(value - prior.q_deg[index])}`).join(' · ');
  }
  if (moving) {
    ui.ikSelectedNote.textContent = 'Entre soluciones se interpolan los giros; solo las posturas finales alcanzan el XYZ.';
  } else if (!selected) {
    ui.ikSelectedNote.textContent = 'La postura dibujada marca hasta dónde llega el brazo.';
  } else if (current.base_free) {
    ui.ikSelectedNote.textContent = 'Al girar q₁, la punta sigue en el mismo punto.';
  } else {
    const pairIndex = current.solutions.findIndex((solution, index) => index !== inverseSolutionIndex && solution.posture === selected.posture);
    ui.ikSelectedNote.textContent = pairIndex >= 0
      ? `La terna ${pairIndex + 1} conserva esta forma; cambian los ángulos y la flecha de base.`
      : 'Con estos ángulos, la punta vuelve al punto XYZ del modelo.';
  }
  for (const [index, button] of [...ui.ikSolutions.querySelectorAll('button')].entries()) {
    const active = index === inverseSolutionIndex && !moving;
    button.classList.toggle('active', active);
    button.classList.toggle('arriving', moving && index === inverseTransition.toIndex);
    button.setAttribute('aria-pressed', String(active));
  }
  ui.ikYawValue.textContent = displayDegrees(qDegrees[0]);
}

function activateInverseCase(found) {
  stopInverseTour();
  activeInverseCase = found;
  inverseSolutionIndex = 0;
  inverseLiveQ = found.solutions[0]?.q.slice() || null;
  ui.ikYaw.value = '0';
  ui.ikYawWrap.hidden = !found.base_free;
  ui.ikCaseDescription.textContent = inverseCaseDescription(found);
  ui.sceneTitle.textContent = found.label;
  populateInverseSolutions();
  updateInverseMeasurementGuide(found);
  updateInverseTourControls();
  renderInverseScene();
}

function selectInverseCase() {
  if (!dataset?.inverse_cases) return;
  const found = ui.ikCase.value === 'custom'
    ? customInverseCase
    : dataset.inverse_cases.find((item) => item.id === ui.ikCase.value);
  if (!found) throw new Error('No se encontró el caso de cinemática inversa seleccionado.');
  activateInverseCase(found);
}

function setMode(next) {
  if (!dataset || next === mode) return;
  if (next === 'inverse') trajectoryWasPlaying = playing;
  if (next !== 'inverse') stopInverseTour();
  mode = next;
  const inverse = next === 'inverse';
  ui.modeTrajectory.classList.toggle('active', !inverse);
  ui.modeInverse.classList.toggle('active', inverse);
  ui.modeTrajectory.setAttribute('aria-pressed', String(!inverse));
  ui.modeInverse.setAttribute('aria-pressed', String(inverse));
  ui.trajectoryControls.hidden = inverse;
  ui.inverseControls.hidden = !inverse;
  ui.trajectoryTelemetry.hidden = inverse;
  ui.inverseTelemetry.hidden = !inverse;
  ui.playback.hidden = inverse;
  ui.ikSceneHud.hidden = !inverse;
  if (inverseMeasurementGuide) inverseMeasurementGuide.visible = inverse;
  ui.frameBadge.hidden = inverse;
  robot.ikYawArrow.visible = inverse;
  ui.controlsTitle.textContent = inverse ? 'Cinemática inversa' : 'Simulación 3D';
  ui.controlsSubtitle.textContent = inverse
    ? '¿Qué tres ángulos llevan la punta al punto XYZ?'
    : 'Seguimiento de la punta y error de los controladores.';
  if (inverse) {
    setPlaying(false);
    selectInverseCase();
  } else {
    inverseReachOuter.visible = false;
    inverseReachInner.visible = false;
    inverseGhost.segments.forEach((mesh) => { mesh.visible = false; });
    inverseGhost.elbow.visible = false;
    desiredMarker.material.color.setHex(0xa5d0f0);
    desiredMarker.material.depthTest = true;
    desiredMarker.children[0].material.depthTest = true;
    desiredMarker.renderOrder = 0;
    errorConnector.material.color.setHex(0xffffff);
    ui.sceneLegend.innerHTML = '<span class="legend-line legend-desired"></span> Deseada <span class="legend-line legend-actual"></span> Real';
    ui.sceneTitle.textContent = scenarioNames[activeCase.scenario] || activeCase.label;
    renderPose();
    updateLayerVisibility();
    setPlaying(trajectoryWasPlaying);
  }
  setCameraPreset(activeView);
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
  if (mode === 'trajectory' && activeCase && playing) {
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
  tickInverse(delta);
  if (controls) controls.update();
  if (renderer) renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

function wireControls() {
  ui.modeTrajectory.addEventListener('click', () => setMode('trajectory'));
  ui.modeInverse.addEventListener('click', () => setMode('inverse'));
  ui.ikCase.addEventListener('change', selectInverseCase);
  ui.ikSolveTarget.addEventListener('click', solveCustomTarget);
  [ui.ikCustomX, ui.ikCustomY, ui.ikCustomZ].forEach((input) => {
    input.addEventListener('keydown', (event) => {
      if (event.key !== 'Enter') return;
      event.preventDefault();
      solveCustomTarget();
    });
  });
  ui.ikSolutions.addEventListener('click', (event) => {
    const button = event.target.closest('button[data-index]');
    if (!button || mode !== 'inverse') return;
    inverseTourPlaying = false;
    inverseTourHold = 0;
    moveToInverseSolution(Number(button.dataset.index));
  });
  ui.ikTourPlay.addEventListener('click', () => {
    if (inverseTourPlaying) {
      stopInverseTour();
    } else {
      stopInverseTour();
      inverseSolutionIndex = 0;
      inverseLiveQ = activeInverseCase.solutions[0].q.slice();
      if (activeInverseCase.base_free) inverseLiveQ[0] = Number(ui.ikYaw.value) * Math.PI / 180;
      inverseTourPlaying = true;
      inverseTourHold = 1.3;
      updateInverseTourControls();
    }
    renderInverseScene();
  });
  ui.ikPrevious.addEventListener('click', () => {
    inverseTourPlaying = false;
    moveToInverseSolution(Math.max(0, inverseSolutionIndex - 1));
  });
  ui.ikNext.addEventListener('click', () => {
    inverseTourPlaying = false;
    moveToInverseSolution(Math.min(activeInverseCase.solutions.length - 1, inverseSolutionIndex + 1));
  });
  ui.ikYaw.addEventListener('input', () => {
    stopInverseTour();
    inverseLiveQ[0] = Number(ui.ikYaw.value) * Math.PI / 180;
    renderInverseScene();
  });
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
    if (mode !== 'trajectory') return;
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
    if (!Array.isArray(dataset.inverse_cases)) {
      // Compatibilidad con el visor ya abierto detrás del túnel público: el servidor
      // anterior conserva data.json en memoria, pero sirve este archivo desde disco.
      const snapshotResponse = await fetch('./inverse_snapshot.json', { cache: 'no-store' });
      if (!snapshotResponse.ok) throw new Error('Faltan los casos de inversa para esta ejecución.');
      const snapshot = await snapshotResponse.json();
      if (snapshot.source_folder !== dataset.source_folder) {
        throw new Error('Los casos de inversa no corresponden a la corrida abierta. Reinicia launch_viewer.py.');
      }
      dataset.inverse_cases = snapshot.inverse_cases;
    }
    if (!Array.isArray(dataset.inverse_cases) || !dataset.inverse_cases.length) throw new Error('No hay casos de cinemática inversa en esta ejecución.');
    populateInverseCases();
    ui.sourceName.textContent = 'Trayectorias calculadas con el modelo nominal 3R.';
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
