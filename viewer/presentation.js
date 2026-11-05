const byId = (id) => document.getElementById(id);
const dialog = byId('expositionDialog');
const openButton = byId('openExposition');
const closeButton = byId('closeExposition');
const tabs = [...dialog.querySelectorAll('[data-exposition-tab]')];
const panels = [...dialog.querySelectorAll('[data-exposition-panel]')];
const footer = dialog.querySelector('.exposition-drawer-footer');

let summary = null;

const format = (value, digits = 2) => Number(value).toLocaleString('es-PE', {
  minimumFractionDigits: digits,
  maximumFractionDigits: digits,
});

function showTab(name, moveFocus = false) {
  for (const tab of tabs) {
    const selected = tab.dataset.expositionTab === name;
    tab.setAttribute('aria-selected', String(selected));
    tab.tabIndex = selected ? 0 : -1;
    if (selected && moveFocus) tab.focus();
  }
  for (const panel of panels) panel.hidden = panel.dataset.expositionPanel !== name;
}

function renderSummary() {
  if (!summary) return;
  const lengths = summary.geometria_m || [];
  for (const [id, value] of [['modelL1', lengths[0]], ['modelL2', lengths[1]], ['modelL3', lengths[2]]]) {
    byId(id).textContent = Number.isFinite(value) ? `${format(value * 1000, 0)} mm` : '—';
  }

  const kinematics = summary.cinematica || {};
  byId('singularityVolume').textContent = Number.isFinite(kinematics.fraccion_volumen_sigma_menor_lambda)
    ? `${format(kinematics.fraccion_volumen_sigma_menor_lambda * 100)} %` : '—';
  byId('singularityThreshold').textContent = Number.isFinite(kinematics.umbral_cercania_m_rad)
    ? format(kinematics.umbral_cercania_m_rad, 3) : '—';
  byId('singularityAccuracy').textContent = Number.isFinite(kinematics.sigma_analitica_vs_svd_max_m_rad)
    ? kinematics.sigma_analitica_vs_svd_max_m_rad.toExponential(2) : '—';
  const counts = kinematics.ik_counts || {};
  for (const [id, key] of [
    ['guideIkRegular', 'regular'],
    ['guideIkAligned', 'codo_extend'],
    ['guideIkAxis', 'eje_base'],
    ['guideIkAxisBorder', 'interseccion'],
    ['guideIkOutside', 'inalcanzable_exterior'],
  ]) {
    byId(id).textContent = Number.isInteger(counts[key]) ? String(counts[key]) : '—';
  }

  const measured = summary.articulo?.tabla3?.nominal_vs_experimental || {};
  for (const axis of ['x', 'y', 'z']) {
    const row = measured[axis] || {};
    byId(`table3${axis.toUpperCase()}`).textContent = Number.isFinite(row.mae_mm) ? `${format(row.mae_mm)} mm` : '—';
    byId(`table3${axis.toUpperCase()}N`).textContent = Number.isInteger(row.n) ? `n = ${row.n}` : '—';
  }
  renderControl();
}

function renderControl() {
  if (!summary) return;
  const scenario = byId('scenarioSelect').value;
  const labels = {
    regular: 'Trayectoria regular',
    near_singular: 'Cerca de singularidad',
    near_singular_bias: 'Singularidad + sesgo de 1 mm',
  };
  const readings = {
    regular: 'Ambos métodos siguen casi igual la referencia regular. Esta prueba sirve como línea base antes de acercarse a una singularidad.',
    near_singular: 'El codo se acerca a una singularidad sin error de medición. Compara el seguimiento y la σ mínima antes de evaluar el caso con sesgo.',
    near_singular_bias: 'Con un sesgo de medición de hasta 1 mm, la pseudoinversa solicita velocidades muy altas y alcanza el límite; DLS amortigua esa respuesta. Es una comparación de esta simulación, no una prueba física.',
  };
  byId('controlContext').textContent = labels[scenario] || 'Control del escenario seleccionado';
  byId('controlReading').textContent = readings[scenario] || 'Comparación con la misma referencia y el mismo estado inicial.';
  const pinv = summary.control?.[`${scenario}_pinv`] || {};
  const dls = summary.control?.[`${scenario}_dls`] || {};
  byId('pinvRmse').textContent = Number.isFinite(pinv.rmse_mm) ? `${format(pinv.rmse_mm, 3)} mm` : '—';
  byId('dlsRmse').textContent = Number.isFinite(dls.rmse_mm) ? `${format(dls.rmse_mm, 3)} mm` : '—';
  byId('pinvSaturations').textContent = Number.isInteger(pinv.saturated_samples) ? String(pinv.saturated_samples) : '—';
  byId('dlsSaturations').textContent = Number.isInteger(dls.saturated_samples) ? String(dls.saturated_samples) : '—';
  const dlsSelected = byId('methodDls').getAttribute('aria-pressed') === 'true';
  byId('selectedController').textContent = `Controlador seleccionado: ${dlsSelected ? 'DLS' : 'pseudoinversa'}`;

  const chart = {
    regular: 'control_regular.png',
    near_singular: 'control_near_singular.png',
    near_singular_bias: 'control_near_singular_bias.png',
  }[scenario];
  if (chart) {
    const url = `./evidence/${chart}`;
    byId('controlChartLink').href = url;
    byId('controlChart').src = url;
  }
}

openButton.addEventListener('click', () => {
  if (!dialog.open) dialog.showModal();
});
closeButton.addEventListener('click', () => dialog.close());
dialog.addEventListener('click', (event) => {
  if (event.target === dialog) dialog.close();
});
dialog.addEventListener('close', () => openButton.focus());

tabs.forEach((tab, index) => {
  tab.addEventListener('click', () => showTab(tab.dataset.expositionTab));
  tab.addEventListener('keydown', (event) => {
    const nextIndex = {
      ArrowRight: (index + 1) % tabs.length,
      ArrowLeft: (index + tabs.length - 1) % tabs.length,
      Home: 0,
      End: tabs.length - 1,
    }[event.key];
    if (nextIndex === undefined) return;
    event.preventDefault();
    showTab(tabs[nextIndex].dataset.expositionTab, true);
  });
});

byId('scenarioSelect').addEventListener('change', renderControl);
byId('methodPinv').addEventListener('click', () => queueMicrotask(renderControl));
byId('methodDls').addEventListener('click', () => queueMicrotask(renderControl));

for (const image of dialog.querySelectorAll('.exposition-figure img')) {
  image.addEventListener('error', () => { image.closest('figure').hidden = true; });
  image.addEventListener('load', () => { image.closest('figure').hidden = false; });
}

fetch('./summary.json', { cache: 'no-store' })
  .then((response) => {
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json();
  })
  .then((data) => { summary = data; renderSummary(); })
  .catch((error) => {
    footer.textContent = 'No se pudo leer el resumen de esta ejecución.';
    console.error('Guía de exposición:', error);
  });
