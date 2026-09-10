/* =========================================================================
   Omni-RC-Robot - logique de l'interface Web
   JavaScript vanilla, sans dependance : l'interface doit fonctionner
   lorsque le robot n'a aucun acces Internet.
   ========================================================================= */

'use strict';

/** Configuration telle que recue du robot, modifiee localement puis renvoyee. */
let configuration = null;

/** Derniere telemetrie recue par WebSocket. */
let telemetry = null;

/** Etat de l'assistant de calibration SBUS. */
const calibrationWizard = {
  phase: 'idle',          // idle | center | sweep | ready
  center: [0, 0, 0],
  minimum: [2047, 2047, 2047],
  maximum: [0, 0, 0]
};

const AXIS_KEYS = ['translationX', 'translationY', 'rotation'];
const AXIS_LABELS = { translationX: 'Translation X (vx)', translationY: 'Translation Y (vy)', rotation: 'Rotation (omega)' };
const MASKED_PASSWORD = '********';

// ---------------------------------------------------------------- utilitaires

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => Array.from(document.querySelectorAll(selector));

/** Lit une valeur imbriquee, par exemple readPath(config, 'drive.maxMotorOutput'). */
function readPath(object, path) {
  return path.split('.').reduce((current, key) => (current == null ? undefined : current[key]), object);
}

/** Ecrit une valeur imbriquee en creant les objets intermediaires manquants. */
function writePath(object, path, value) {
  const keys = path.split('.');
  const lastKey = keys.pop();
  const target = keys.reduce((current, key) => (current[key] = current[key] || {}), object);
  target[lastKey] = value;
}

function showToast(message, kind) {
  const toast = $('#toast');
  toast.textContent = message;
  toast.className = 'toast' + (kind ? ' toast-' + kind : '');
  clearTimeout(showToast.timer);
  showToast.timer = setTimeout(() => { toast.textContent = ''; }, 5000);
}

/** Appel d'API avec remontee explicite du message d'erreur du firmware. */
async function callApi(url, options) {
  const response = await fetch(url, options);
  let body = {};
  try { body = await response.json(); } catch (error) { /* reponse sans corps JSON */ }
  if (!response.ok) {
    throw new Error(body.error || ('Erreur HTTP ' + response.status));
  }
  return body;
}

function postJson(url, payload) {
  return callApi(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload || {})
  });
}

// ------------------------------------------------------------------- onglets

$('#tabs').addEventListener('click', (event) => {
  const tab = event.target.closest('.tab');
  if (!tab) return;
  $$('.tab').forEach((element) => element.classList.toggle('tab-active', element === tab));
  $$('.panel').forEach((panel) => {
    panel.classList.toggle('panel-active', panel.id === 'panel-' + tab.dataset.panel);
  });
  if (tab.dataset.panel === 'system') refreshSystemPanel();
});

// ------------------------------------------------------- chargement de config

async function loadConfiguration() {
  configuration = await callApi('/api/config');
  buildAxisEditors();
  buildWheelEditors();
  bindConfigurationInputs();
}

/** Recopie la configuration dans tous les champs marques data-config. */
function bindConfigurationInputs() {
  $$('[data-config]').forEach((input) => {
    const storedValue = readPath(configuration, input.dataset.config);
    if (storedValue === undefined) return;

    if (input.type === 'checkbox') {
      input.checked = Boolean(storedValue);
    } else {
      input.value = toDisplayValue(input, storedValue);
    }
  });
}

/**
 * Les voies sont stockees comme des index de tableau, donc a partir de 0, alors
 * que l'utilisateur et le tableau de bord parlent en CH1..CH16. L'attribut
 * data-offset porte cet ecart, pour que l'interface n'affiche jamais un numero
 * different de celui qu'elle montre ailleurs.
 */
function toDisplayValue(input, storedValue) {
  const offset = Number(input.dataset.offset || 0);
  return offset ? Number(storedValue) + offset : storedValue;
}

function toStoredValue(input, displayedValue) {
  const offset = Number(input.dataset.offset || 0);
  return offset ? displayedValue - offset : displayedValue;
}

/** Relit tous les champs vers l'objet de configuration local. */
function collectConfigurationFromInputs() {
  $$('[data-config]').forEach((input) => {
    const path = input.dataset.config;
    if (input.type === 'checkbox') {
      writePath(configuration, path, input.checked);
    } else if (input.type === 'number') {
      const parsed = parseFloat(input.value);
      if (!Number.isNaN(parsed)) writePath(configuration, path, toStoredValue(input, parsed));
    } else {
      writePath(configuration, path, input.value);
    }
  });
}

function buildAxisEditors() {
  const container = $('#axis-editors');
  container.innerHTML = '';

  AXIS_KEYS.forEach((axisKey) => {
    const editor = document.createElement('div');
    editor.className = 'editor';
    editor.innerHTML =
      '<h3>' + AXIS_LABELS[axisKey] + '<span class="live" id="live-' + axisKey + '">-</span></h3>' +
      '<div class="field-grid">' +
        '<label>Voie (CH1 a CH16)<input type="number" min="1" max="16" data-offset="1" data-config="radio.' + axisKey + '.channel"></label>' +
        '<label>Minimum<input type="number" min="0" max="2047" data-config="radio.' + axisKey + '.rawMin"></label>' +
        '<label>Centre<input type="number" min="0" max="2047" data-config="radio.' + axisKey + '.rawCenter"></label>' +
        '<label>Maximum<input type="number" min="0" max="2047" data-config="radio.' + axisKey + '.rawMax"></label>' +
        '<label>Deadband<input type="number" step="0.01" min="0" max="0.5" data-config="radio.' + axisKey + '.deadband"></label>' +
        '<label>Expo<input type="number" step="0.01" min="0" max="1" data-config="radio.' + axisKey + '.expo"></label>' +
        '<label>Gain<input type="number" step="0.01" min="0" max="1" data-config="radio.' + axisKey + '.gain"></label>' +
        '<label class="checkbox"><input type="checkbox" data-config="radio.' + axisKey + '.inverted"><span>Inverser</span></label>' +
      '</div>';
    container.appendChild(editor);
  });
}

function buildWheelEditors() {
  const container = $('#wheel-editors');
  container.innerHTML = '';

  for (let wheelIndex = 0; wheelIndex < 4; wheelIndex += 1) {
    const editor = document.createElement('div');
    editor.className = 'editor';
    editor.innerHTML =
      '<h3>M' + wheelIndex + '<span class="live" id="live-wheel-' + wheelIndex + '">-</span></h3>' +
      '<div class="field-grid">' +
        '<label>Angle (deg)<input type="number" step="0.5" min="-360" max="360" data-config="wheels.' + wheelIndex + '.angleDeg"></label>' +
        '<label>Gain de rotation<input type="number" step="0.05" min="-2" max="2" data-config="wheels.' + wheelIndex + '.rotationGain"></label>' +
        '<label>Gain moteur<input type="number" step="0.01" min="0" max="1" data-config="wheels.' + wheelIndex + '.outputGain"></label>' +
        '<label class="checkbox"><input type="checkbox" data-config="wheels.' + wheelIndex + '.inverted"><span>Inverser le sens</span></label>' +
        '<label class="checkbox"><input type="checkbox" data-config="wheels.' + wheelIndex + '.enabled"><span>Moteur actif</span></label>' +
      '</div>';
    container.appendChild(editor);
  }
}

// --------------------------------------------------- application / sauvegarde

async function applyConfiguration() {
  collectConfigurationFromInputs();

  // Un mot de passe laisse au marqueur signifie "ne pas changer" : il est
  // retire du document envoye pour eviter toute ambiguite cote firmware.
  const payload = JSON.parse(JSON.stringify(configuration));
  ['apPassword', 'stationPassword'].forEach((key) => {
    if (payload.wifi && payload.wifi[key] === MASKED_PASSWORD) delete payload.wifi[key];
  });

  try {
    const result = await postJson('/api/config', payload);
    showToast(result.message || 'Configuration appliquee', 'ok');
    await loadConfiguration();
  } catch (error) {
    showToast(error.message, 'bad');
  }
}

$('#apply-button').addEventListener('click', applyConfiguration);

$('#save-button').addEventListener('click', async () => {
  try {
    await applyConfiguration();
    const result = await postJson('/api/config/save');
    showToast(result.message, 'ok');
  } catch (error) {
    showToast(error.message, 'bad');
  }
});

$('#reset-button').addEventListener('click', async () => {
  if (!confirm('Restaurer toutes les valeurs par defaut ?')) return;
  try {
    const result = await postJson('/api/config/reset');
    await loadConfiguration();
    showToast(result.message, 'ok');
  } catch (error) {
    showToast(error.message, 'bad');
  }
});

$('#reboot-button').addEventListener('click', async () => {
  if (!confirm('Redemarrer le robot maintenant ?')) return;
  try {
    await postJson('/api/reboot');
    showToast('Redemarrage en cours...', 'ok');
  } catch (error) {
    showToast(error.message, 'bad');
  }
});

$('#disarm-button').addEventListener('click', async () => {
  try {
    await postJson('/api/disarm');
    showToast('Desarmement demande', 'ok');
  } catch (error) {
    showToast(error.message, 'bad');
  }
});

// ------------------------------------------------------ export / import JSON

$('#export-button').addEventListener('click', () => {
  collectConfigurationFromInputs();
  const blob = new Blob([JSON.stringify(configuration, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = 'robot-config.json';
  link.click();
  URL.revokeObjectURL(url);
  showToast('Configuration exportee (mots de passe exclus)', 'ok');
});

$('#import-button').addEventListener('click', () => $('#import-file').click());

$('#import-file').addEventListener('change', async (event) => {
  const file = event.target.files[0];
  if (!file) return;
  try {
    const imported = JSON.parse(await file.text());
    const result = await postJson('/api/config', imported);
    await loadConfiguration();
    showToast(result.message || 'Configuration importee', 'ok');
  } catch (error) {
    showToast('Import impossible : ' + error.message, 'bad');
  }
  event.target.value = '';
});

// --------------------------------------------------------- presets geometrie

$$('.preset-button').forEach((button) => {
  button.addEventListener('click', () => {
    const angle = parseFloat(button.dataset.angle);
    // Memes formules que applySymmetricWheelGeometry() cote firmware.
    const angles = [270 - angle, 90 + angle, 90 - angle, 270 + angle];
    angles.forEach((value, wheelIndex) => {
      configuration.wheels[wheelIndex].angleDeg = value;
      configuration.wheels[wheelIndex].rotationGain = 1;
    });
    bindConfigurationInputs();
    showToast('Preset ' + angle + ' degres pre-rempli. Appliquer pour l\'activer.', 'ok');
  });
});

// -------------------------------------------------------- simulateur de mixer

$$('.sim-button').forEach((button) => {
  button.addEventListener('click', async () => {
    try {
      const result = await postJson('/api/mixer-simulate', {
        vx: parseFloat(button.dataset.vx),
        vy: parseFloat(button.dataset.vy),
        omega: parseFloat(button.dataset.omega)
      });
      $('#simulation-row').innerHTML =
        result.motors.map((value) => '<td>' + value.toFixed(3) + '</td>').join('');
    } catch (error) {
      showToast(error.message, 'bad');
    }
  });
});

// ------------------------------------------------------ assistant calibration

function updateWizardStatus(text) { $('#wizard-status').textContent = text; }

$('#wizard-center').addEventListener('click', () => {
  calibrationWizard.phase = 'center';
  updateWizardStatus('Manches au centre... relachez tout, puis passez a l\'etape 2.');
});

$('#wizard-sweep').addEventListener('click', () => {
  calibrationWizard.phase = 'sweep';
  calibrationWizard.minimum = [2047, 2047, 2047];
  calibrationWizard.maximum = [0, 0, 0];
  updateWizardStatus('Balayez chaque manche jusqu\'aux butees, puis appliquez.');
  $('#wizard-apply').disabled = false;
});

$('#wizard-apply').addEventListener('click', () => {
  AXIS_KEYS.forEach((axisKey, index) => {
    configuration.radio[axisKey].rawCenter = Math.round(calibrationWizard.center[index]);
    configuration.radio[axisKey].rawMin = Math.round(calibrationWizard.minimum[index]);
    configuration.radio[axisKey].rawMax = Math.round(calibrationWizard.maximum[index]);
  });
  bindConfigurationInputs();
  calibrationWizard.phase = 'idle';
  $('#wizard-apply').disabled = true;
  updateWizardStatus('Valeurs reportees dans les champs. Verifiez puis Appliquer.');
});

$('#wizard-cancel').addEventListener('click', () => {
  calibrationWizard.phase = 'idle';
  $('#wizard-apply').disabled = true;
  updateWizardStatus('Assistant a l\'arret.');
});

/** Alimente l'assistant avec les voies brutes de la telemetrie courante. */
function feedCalibrationWizard(channels) {
  if (!configuration || calibrationWizard.phase === 'idle') return;

  AXIS_KEYS.forEach((axisKey, index) => {
    const raw = channels[configuration.radio[axisKey].channel];
    if (raw === undefined) return;
    if (calibrationWizard.phase === 'center') {
      calibrationWizard.center[index] = raw;
    } else if (calibrationWizard.phase === 'sweep') {
      calibrationWizard.minimum[index] = Math.min(calibrationWizard.minimum[index], raw);
      calibrationWizard.maximum[index] = Math.max(calibrationWizard.maximum[index], raw);
    }
  });
}

// -------------------------------------------------------------- test moteur

const motorTest = { activeIndex: -1, direction: 0, timer: null };

function buildMotorTestControls() {
  const container = $('#motor-test-controls');
  container.innerHTML = '';

  for (let motorIndex = 0; motorIndex < 4; motorIndex += 1) {
    const row = document.createElement('div');
    row.className = 'motor-test-row';
    row.innerHTML = '<strong>M' + motorIndex + '</strong>';

    [['-', -1], ['+', 1]].forEach(([label, direction]) => {
      const button = document.createElement('button');
      button.className = 'button';
      button.textContent = label;
      // Le test ne dure que tant que le bouton est tenu : la demande est
      // renouvelee periodiquement et expire d'elle-meme cote firmware.
      const start = (event) => { event.preventDefault(); startMotorTest(motorIndex, direction); };
      button.addEventListener('mousedown', start);
      button.addEventListener('touchstart', start, { passive: false });
      ['mouseup', 'mouseleave', 'touchend', 'touchcancel'].forEach((eventName) => {
        button.addEventListener(eventName, stopMotorTest);
      });
      row.appendChild(button);
    });

    container.appendChild(row);
  }
}

async function sendMotorTestRequest() {
  if (motorTest.activeIndex < 0 || !configuration) return;
  const output = motorTest.direction * configuration.safety.motorTestMaxOutput;
  try {
    await postJson('/api/motor-test', { motor: motorTest.activeIndex, output: output });
    $('#motor-test-status').textContent =
      'Test en cours : M' + motorTest.activeIndex + ' a ' + output.toFixed(2);
  } catch (error) {
    stopMotorTest();
    showToast(error.message, 'bad');
  }
}

function startMotorTest(motorIndex, direction) {
  if (!$('#motor-test-confirm').checked) {
    showToast('Cochez d\'abord la confirmation de securite', 'bad');
    return;
  }
  motorTest.activeIndex = motorIndex;
  motorTest.direction = direction;
  sendMotorTestRequest();
  clearInterval(motorTest.timer);
  motorTest.timer = setInterval(sendMotorTestRequest, 400);
}

function stopMotorTest() {
  if (motorTest.activeIndex < 0) return;
  motorTest.activeIndex = -1;
  clearInterval(motorTest.timer);
  motorTest.timer = null;
  $('#motor-test-status').textContent = 'Test moteur inactif.';
  postJson('/api/motor-test/stop').catch(() => { /* l'expiration cote robot prend le relais */ });
}

// Toute perte de focus de la page coupe le test en cours.
window.addEventListener('blur', stopMotorTest);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopMotorTest(); });

// ------------------------------------------------------------- affichage live

function setGauge(gaugeId, outputId, value) {
  const percent = Math.max(-1, Math.min(1, value)) * 50;
  const bar = $('#' + gaugeId);
  bar.style.left = (percent < 0 ? 50 + percent : 50) + '%';
  bar.style.width = Math.abs(percent) + '%';
  $('#' + outputId).textContent = value.toFixed(2);
}

function setFlag(elementId, isOn, okWhenOn, textOn, textOff) {
  const element = $('#' + elementId);
  element.textContent = isOn ? textOn : textOff;
  element.className = 'value ' + ((isOn === okWhenOn) ? 'value-ok' : 'value-bad');
}

function renderTelemetry(data) {
  telemetry = data;

  // --- Etat general
  const stateBadge = $('#robot-state');
  stateBadge.textContent = data.state;
  stateBadge.className = 'state-badge state-' + data.state.toLowerCase().replace(/_/g, '');

  // --- Radio
  $('#radio-protocol').textContent = data.radio.protocol;
  setFlag('sbus-connected', data.radio.connected, true, 'connectee', 'perdue');
  setFlag('sbus-failsafe', data.radio.failsafe, false, 'ACTIF', 'non');
  setFlag('sbus-framelost', data.radio.frameLost, false, 'oui', 'non');
  $('#sbus-rate').textContent = data.radio.frameRateHz.toFixed(1) + ' Hz';
  $('#sbus-age').textContent = data.radio.frameAgeMs < 0 ? '-' : data.radio.frameAgeMs + ' ms';
  $('#sbus-frames').textContent = data.radio.validFrames +
    ' (' + data.radio.frameLostCount + ' perdues, ' + data.radio.errorCount + ' erreurs)';
  // Trace brute : indispensable tant qu'aucune trame n'est decodee.
  $('#radio-rawbytes').textContent = data.radio.rawBytes + ' octets  ' + data.radio.rawDump;

  // --- Commandes
  setGauge('gauge-vx', 'value-vx', data.input.vx);
  setGauge('gauge-vy', 'value-vy', data.input.vy);
  setGauge('gauge-omega', 'value-omega', data.input.omega);
  $('#arm-switch').textContent = data.armSwitch ? 'arme' : 'desarme';
  renderArmingChecklist(data);
  $('#speed-mode').textContent = data.slowMode ? 'lent' : 'rapide';

  // --- Moteurs
  $('#motor-table').innerHTML = data.motors.map((applied, index) => {
    const direction = applied > 0.001 ? 'avant' : (applied < -0.001 ? 'arriere' : 'arret');
    return '<tr><td>M' + index + '</td><td>' + data.mixer[index].toFixed(3) +
           '</td><td>' + applied.toFixed(3) + '</td><td>' + direction +
           '</td><td>' + data.duty[index] + '</td></tr>';
  }).join('');

  // --- Voies brutes, avec mise en evidence des voies affectees
  if (configuration) {
    const usedChannels = new Set([
      configuration.radio.translationX.channel,
      configuration.radio.translationY.channel,
      configuration.radio.rotation.channel,
      configuration.radio.armChannel,
      configuration.radio.speedModeChannel
    ]);
    const channelMarkup = data.channels.map((raw, index) =>
      '<div class="channel' + (usedChannels.has(index) ? ' channel-used' : '') +
      '"><span>CH' + (index + 1) + '</span><span>' + raw + '</span></div>'
    ).join('');
    // Le meme tableau est affiche dans l'onglet Radio : c'est la qu'on affecte
    // les voies, et il faut voir laquelle bouge sans changer d'onglet.
    $('#channel-list').innerHTML = channelMarkup;
    $('#channel-list-radio').innerHTML = channelMarkup;

    AXIS_KEYS.forEach((axisKey) => {
      const element = $('#live-' + axisKey);
      if (element) element.textContent = data.channels[configuration.radio[axisKey].channel];
    });
    for (let wheelIndex = 0; wheelIndex < 4; wheelIndex += 1) {
      const element = $('#live-wheel-' + wheelIndex);
      if (element) element.textContent = data.motors[wheelIndex].toFixed(2);
    }
  }

  updateRobotView(data);
  setUnsavedState(data.unsavedConfig);
  feedCalibrationWizard(data.channels);
}

/** Met a jour la vue de dessus : vecteur de translation et anneau de rotation. */
function updateRobotView(data) {
  const centerX = 110, centerY = 110, scale = 60;
  // L'axe Y de l'ecran est inverse par rapport a l'axe Y du robot.
  const endX = centerX + data.input.vx * scale;
  const endY = centerY - data.input.vy * scale;
  const vector = $('#drive-vector');
  vector.setAttribute('x2', endX.toFixed(1));
  vector.setAttribute('y2', endY.toFixed(1));

  updateRotationIndicator(data.input.omega);

  // Intensite de chaque roue, proportionnelle a la commande appliquee.
  data.motors.forEach((value, index) => {
    const wheel = $('#wheel-' + index + ' rect');
    if (!wheel) return;
    const intensity = Math.min(1, Math.abs(value));
    wheel.style.fill = intensity < 0.01
      ? ''
      : (value > 0 ? 'rgba(79,155,255,' : 'rgba(240,178,60,') + (0.15 + intensity * 0.85) + ')';
  });
}

// --- Vue de dessus : geometrie de l'indicateur de rotation ------------------

const ROBOT_VIEW_CENTER = 110;
const ROTATION_ARC_RADIUS = 34;
/// En deca de cette valeur, la rotation demandee n'est pas representee.
const ROTATION_VISIBLE_THRESHOLD = 0.02;

/**
 * Convertit un angle mathematique (degres, antihoraire depuis +X) en point SVG.
 *
 * L'axe Y de l'ecran pointe vers le bas : il est donc inverse ici, ce qui fait
 * correspondre les angles croissants au sens antihoraire a l'affichage.
 */
function polarToSvgPoint(angleDeg) {
  const angleRad = angleDeg * Math.PI / 180;
  return {
    x: ROBOT_VIEW_CENTER + ROTATION_ARC_RADIUS * Math.cos(angleRad),
    y: ROBOT_VIEW_CENTER - ROTATION_ARC_RADIUS * Math.sin(angleRad)
  };
}

/**
 * Construit un arc centre sur un angle, parcouru dans le sens demande.
 *
 * La pointe de fleche etant placee en fin de trace, l'arc est toujours dessine
 * dans le sens de rotation reel : c'est ce qui rend ce sens lisible.
 *
 * @param {number} centerAngleDeg Angle autour duquel l'arc est centre.
 * @param {number} sweepDeg Ouverture totale de l'arc.
 * @param {boolean} isCounterClockwise Sens de parcours.
 * @return {string} Attribut 'd' du chemin SVG.
 */
function describeRotationArc(centerAngleDeg, sweepDeg, isCounterClockwise) {
  const halfSweep = sweepDeg / 2;
  const startAngle = centerAngleDeg + (isCounterClockwise ? -halfSweep : halfSweep);
  const endAngle   = centerAngleDeg + (isCounterClockwise ? halfSweep : -halfSweep);

  const start = polarToSvgPoint(startAngle);
  const end   = polarToSvgPoint(endAngle);

  const largeArcFlag = sweepDeg > 180 ? 1 : 0;
  // Dans le repere SVG, Y pointe vers le bas : sweep-flag = 1 dessine donc dans
  // le sens horaire a l'ecran, et l'antihoraire demande 0.
  const sweepFlag = isCounterClockwise ? 0 : 1;

  return 'M ' + start.x.toFixed(2) + ' ' + start.y.toFixed(2) +
         ' A ' + ROTATION_ARC_RADIUS + ' ' + ROTATION_ARC_RADIUS +
         ' 0 ' + largeArcFlag + ' ' + sweepFlag + ' ' +
         end.x.toFixed(2) + ' ' + end.y.toFixed(2);
}

/**
 * Affiche, condition par condition, ce qui autorise ou empeche l'armement.
 *
 * C'est le pendant visible de SafetyManager : l'utilisateur doit pouvoir voir
 * d'un coup d'oeil pourquoi le robot refuse de s'armer, sans lire les logs.
 */
function renderArmingChecklist(data) {
  const arming = data.arming;
  if (!arming) return;

  const isArmed = data.state === 'ARMED';

  const conditions = [
    { ok: arming.radioOk,  label: 'Liaison radio valide, sans failsafe' },
    { ok: arming.switchOk, label: 'Interrupteur d\'armement de la radio en position armee' },
    { ok: arming.sticksOk, label: 'Manches au neutre' },
    { ok: !arming.switchCycleNeeded,
      label: 'Interrupteur repasse au repos depuis le dernier arret',
      waiting: arming.switchCycleNeeded }
  ];

  $('#arming-checklist').innerHTML = conditions.map((condition) => {
    const cssClass = condition.ok ? 'ok' : (condition.waiting ? 'wait' : 'bad');
    const mark = condition.ok ? '✓' : (condition.waiting ? '!' : '✗');
    return '<li><span class="mark ' + cssClass + '">' + mark + '</span>' +
           '<span>' + condition.label + '</span></li>';
  }).join('');

  const summary = $('#arming-summary');
  if (isArmed) {
    summary.textContent = 'Robot ARME : les moteurs repondent a la radiocommande.';
  } else if (arming.switchCycleNeeded) {
    summary.textContent = 'Reamement bloque : remettez l\'interrupteur d\'armement au repos, ' +
                          'puis en position armee.';
  } else if (conditions.every((condition) => condition.ok)) {
    summary.textContent = 'Toutes les conditions sont reunies : armement dans ' +
                          arming.holdRemainingMs + ' ms...';
  } else {
    const missing = conditions.filter((condition) => !condition.ok).length;
    summary.textContent = missing + ' condition(s) non remplie(s) : le robot reste desarme.';
  }
}

/**
 * Affiche le sens et l'intensite de la rotation demandee.
 *
 * @param {number} omega Consigne de rotation, positive en antihoraire.
 */
function updateRotationIndicator(omega) {
  const indicator = $('#rotation-indicator');
  const magnitude = Math.abs(omega);

  if (magnitude < ROTATION_VISIBLE_THRESHOLD) {
    indicator.style.display = 'none';
    return;
  }
  indicator.style.display = '';

  const isCounterClockwise = omega > 0;

  // L'arc s'allonge avec la consigne : 40 degres au minimum pour rester
  // lisible, 140 au maximum pour laisser les deux arcs distincts.
  const sweepDeg = 40 + Math.min(1, magnitude) * 100;

  $('#rotation-arc-top').setAttribute('d',
    describeRotationArc(90, sweepDeg, isCounterClockwise));
  $('#rotation-arc-bottom').setAttribute('d',
    describeRotationArc(270, sweepDeg, isCounterClockwise));

  // L'epaisseur renforce la lecture de l'intensite, sans masquer le chassis.
  indicator.style.strokeWidth = (1.5 + Math.min(1, magnitude) * 2).toFixed(2);
}

// ------------------------------------------------- protection des reglages

/** Vrai tant que la configuration en RAM differe de celle enregistree en NVS. */
let hasUnsavedConfiguration = false;

/**
 * Signale visuellement les modifications non enregistrees.
 *
 * Une modification vit en RAM jusqu'au clic sur Enregistrer : tout redemarrage
 * de la carte la perd, y compris un reflash du firmware. L'etat doit donc etre
 * impossible a manquer.
 */
function setUnsavedState(isUnsaved) {
  hasUnsavedConfiguration = Boolean(isUnsaved);
  const banner = $('#unsaved-marker');
  banner.hidden = !hasUnsavedConfiguration;
  $('#save-button').classList.toggle('button-attention', hasUnsavedConfiguration);
}

// Le navigateur demande confirmation avant de fermer ou recharger la page si des
// reglages n'ont pas ete ecrits en memoire non volatile.
window.addEventListener('beforeunload', (event) => {
  if (!hasUnsavedConfiguration) return;
  event.preventDefault();
  event.returnValue = '';
});

// ------------------------------------------------------------------- journal

function appendLogLines(lines) {
  const view = $('#log-view');
  const isScrolledToBottom = view.scrollHeight - view.clientHeight - view.scrollTop < 40;

  lines.forEach((line) => {
    const element = document.createElement('div');
    element.className = 'log-line log-' + line.level;
    const seconds = (line.time / 1000).toFixed(3).padStart(9, ' ');
    element.textContent = '[' + seconds + '][' + line.level + '][' + line.module + '] ' + line.message;
    view.appendChild(element);
  });

  while (view.childElementCount > 400) view.removeChild(view.firstChild);
  if (isScrolledToBottom) view.scrollTop = view.scrollHeight;
}

// -------------------------------------------------------- panneau systeme

async function refreshSystemPanel() {
  try {
    const system = await callApi('/api/system');
    const rows = [
      ['Version firmware', system.firmwareVersion],
      ['Compile le', system.buildDate],
      ['Puce', system.chipModel + ' (' + system.chipCores + ' coeurs)'],
      ['Version configuration', system.configVersion],
      ['Uptime', system.uptimeSeconds + ' s'],
      ['Cause du dernier reset', system.resetReason],
      ['Heap libre', (system.freeHeap / 1024).toFixed(1) + ' Ko'],
      ['Heap minimum atteint', (system.minimumFreeHeap / 1024).toFixed(1) + ' Ko'],
      ['Mode Wi-Fi', system.wifiMode === 'station' ? 'station' : 'point d\'acces'],
      ['SSID', system.ssid],
      ['Adresse IP', system.ipAddress],
      ['RSSI', system.wifiMode === 'station' ? system.rssi + ' dBm' : '-'],
      ['Clients Web', system.webClients],
      ['Interface en flash', system.fileSystem ? 'presente' : 'ABSENTE'],
      ['GPIO SBUS', system.pins.sbusRx],
      ['GPIO LED', system.pins.statusLed],
      ['GPIO moteurs', system.pins.motors.map((pin, index) => 'M' + index + ':' + pin.rpwm + '/' + pin.lpwm).join('  ')]
    ];
    if (telemetry) {
      rows.push(['Frequence boucle controle', telemetry.loopRateHz.toFixed(0) + ' Hz']);
      rows.push(['Entrees en failsafe', telemetry.failsafeCount]);
    }
    $('#system-readout').innerHTML = rows
      .map(([label, value]) => '<dt>' + label + '</dt><dd>' + value + '</dd>').join('');
  } catch (error) {
    showToast(error.message, 'bad');
  }
}

// ----------------------------------------------------------------- WebSocket

function connectWebSocket() {
  const socket = new WebSocket('ws://' + location.host + '/ws');
  const linkState = $('#link-state');

  socket.addEventListener('open', () => {
    linkState.textContent = 'connecte';
    linkState.className = 'pill pill-ok';
  });

  socket.addEventListener('message', (event) => {
    const message = JSON.parse(event.data);
    if (message.type === 'telemetry') renderTelemetry(message);
    else if (message.type === 'logs') appendLogLines(message.lines);
  });

  socket.addEventListener('close', () => {
    linkState.textContent = 'deconnecte';
    linkState.className = 'pill pill-bad';
    stopMotorTest();
    setTimeout(connectWebSocket, 1500);   // reconnexion automatique
  });

  socket.addEventListener('error', () => socket.close());
}

// -------------------------------------------------------------- demarrage

(async function initialize() {
  buildMotorTestControls();
  try {
    await loadConfiguration();
  } catch (error) {
    showToast('Configuration illisible : ' + error.message, 'bad');
  }
  try {
    const initialLogs = await callApi('/api/logs');
    appendLogLines(initialLogs.lines);
  } catch (error) { /* le WebSocket prendra le relais */ }
  connectWebSocket();
  refreshSystemPanel();
})();
