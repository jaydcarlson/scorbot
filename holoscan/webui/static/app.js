'use strict';

const MODE = { IDLE: 0, HOLD: 1, POSITION: 2, VELOCITY: 3, PWM: 4 };
const MODE_NAME = ['idle', 'hold', 'position', 'velocity', 'pwm'];
const HOMING_NAME = ['idle', 'park', 'seek', 'backoff', 'creep', 'latch', 'hold', 'done', 'FAULT'];
const JFLAG = { HOMED: 1, AT_LIMIT: 2, SATURATED: 4, FAULT: 8 };

let ws = null;
let joints = [];
let estopped = false;
// While a slider is being dragged we stop echoing the robot's target back into
// it, otherwise the control fights the user's thumb.
let dragging = new Set();

const el = (id) => document.getElementById(id);

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

function banner(text, kind) {
  const b = el('banner');
  if (!text) {
    b.classList.add('hidden');
    return;
  }
  b.textContent = text;
  b.className = 'banner ' + (kind || 'info');
}

// Wrist joints report infinite travel; give the slider something usable.
function safeRange(j) {
  const lo = Number.isFinite(j.min) && j.min !== null ? j.min : -180;
  const hi = Number.isFinite(j.max) && j.max !== null ? j.max : 180;
  return [lo, hi];
}

function buildJoints(info) {
  joints = info.joints;
  const host = el('joints');
  host.innerHTML = '';

  const homeSel = el('home-joint-select');
  const gainSel = el('gain-joint');
  homeSel.innerHTML = '';
  gainSel.innerHTML = '';

  joints.forEach((j, i) => {
    const [lo, hi] = safeRange(j);

    const card = document.createElement('div');
    card.className = 'joint unhomed';
    card.id = `joint-${i}`;
    card.innerHTML = `
      <div class="joint-top">
        <span class="joint-name">${j.name || 'joint ' + i}</span>
        <span class="tags">
          <span class="tag" data-tag="homed">homed</span>
          <span class="tag" data-tag="limit">limit</span>
          <span class="tag" data-tag="sat">sat</span>
          <span class="tag" data-tag="fault">fault</span>
        </span>
        <span class="joint-readout">
          <b data-f="pos">--</b>&deg;
          &nbsp;v <span data-f="vel">--</span>
          &nbsp;e <span data-f="eff">--</span>
        </span>
      </div>
      <div class="slider-row">
        <input type="range" data-f="slider" min="${lo}" max="${hi}" step="0.1" value="${(lo + hi) / 2}" disabled>
        <input type="number" class="setpoint" data-f="number" min="${lo}" max="${hi}" step="0.1" value="${(lo + hi) / 2}" disabled>
      </div>
      <div class="track">
        <div class="target" data-f="track-target"></div>
        <div class="actual" data-f="track-actual"></div>
      </div>
      <div class="joint-modes">
        <button class="btn" data-mode="${MODE.IDLE}" type="button">idle</button>
        <button class="btn" data-mode="${MODE.HOLD}" type="button">hold</button>
        <button class="btn" data-mode="${MODE.POSITION}" type="button">position</button>
      </div>`;
    host.appendChild(card);

    const slider = card.querySelector('[data-f=slider]');
    const number = card.querySelector('[data-f=number]');

    const push = (v) => {
      const value = Math.min(hi, Math.max(lo, parseFloat(v)));
      if (!Number.isFinite(value)) return;
      slider.value = value;
      number.value = value.toFixed(1);
      send({ cmd: 'set_joint', joint: i, value: value });
    };

    slider.addEventListener('pointerdown', () => dragging.add(i));
    slider.addEventListener('pointerup', () => dragging.delete(i));
    slider.addEventListener('pointercancel', () => dragging.delete(i));
    slider.addEventListener('input', (e) => push(e.target.value));
    number.addEventListener('change', (e) => push(e.target.value));

    card.querySelectorAll('[data-mode]').forEach((btn) => {
      btn.addEventListener('click', () => {
        send({ cmd: 'set_mode', joint: i, value: parseInt(btn.dataset.mode, 10) });
      });
    });

    for (const sel of [homeSel, gainSel]) {
      const opt = document.createElement('option');
      opt.value = i;
      opt.textContent = `${i}: ${j.name}`;
      sel.appendChild(opt);
    }
  });
}

function applyState(s) {
  estopped = !!s.estop;
  const estopBtn = el('estop');
  estopBtn.classList.toggle('engaged', estopped);
  estopBtn.textContent = estopped ? 'CLEAR E-STOP' : 'E-STOP';

  if (s.error) {
    banner(s.error, 'error');
  } else if (s.notice) {
    banner(s.notice, 'info');
  } else if (estopped) {
    banner('E-STOP engaged. Every joint is idle.', 'error');
  } else if (s.flags & 0x0002) {
    banner('Watchdog tripped: the firmware stopped receiving setpoints.', 'warn');
  } else if (s.fault) {
    banner('Firmware reports fault code ' + s.fault, 'warn');
  } else {
    banner(null);
  }

  // Homing progress
  const hs = el('homing-status');
  const name = HOMING_NAME[s.homing_state] || '?';
  const active = s.homing_state > 0 && s.homing_state < 7;
  const jointName =
    s.homing_joint < joints.length ? joints[s.homing_joint].name : '-';
  hs.textContent = active
    ? `${name}  ->  ${jointName}`
    : name === 'done'
      ? 'all homed'
      : name;
  hs.className = 'homing-status' +
    (active ? ' active' : s.homing_state === 7 ? ' done' : s.homing_state === 8 ? ' fault' : '');

  el('stat-rtt').textContent = s.rtt_us ? s.rtt_us.toFixed(0) + ' us' : '--';
  el('stat-sent').textContent = s.sent;
  el('stat-recv').textContent = s.recv;
  el('stat-dropped').textContent = s.dropped;
  el('stat-bad').textContent = s.bad;

  if (!s.joints) return;

  s.joints.forEach((js, i) => {
    const card = el(`joint-${i}`);
    if (!card) return;
    const [lo, hi] = safeRange(joints[i]);

    const homed = (js.flags & JFLAG.HOMED) !== 0;
    card.classList.toggle('unhomed', !homed);

    const setTag = (key, on) => {
      const t = card.querySelector(`[data-tag=${key}]`);
      t.className = 'tag' + (on ? ` on-${key}` : '');
    };
    setTag('homed', homed);
    setTag('limit', (js.flags & JFLAG.AT_LIMIT) !== 0);
    setTag('sat', (js.flags & JFLAG.SATURATED) !== 0);
    setTag('fault', (js.flags & JFLAG.FAULT) !== 0);

    card.querySelector('[data-f=pos]').textContent = js.position.toFixed(2);
    card.querySelector('[data-f=vel]').textContent = js.velocity.toFixed(1);
    card.querySelector('[data-f=eff]').textContent = js.effort.toFixed(2);

    // A joint with no reference could be driven anywhere, so keep it locked
    // until the firmware says it has been homed.
    const slider = card.querySelector('[data-f=slider]');
    const number = card.querySelector('[data-f=number]');
    const locked = !homed || estopped;
    slider.disabled = locked;
    number.disabled = locked;

    if (!dragging.has(i) && Number.isFinite(js.target)) {
      slider.value = js.target;
      number.value = js.target.toFixed(1);
    }

    const pct = (v) => (100 * (Math.min(hi, Math.max(lo, v)) - lo)) / (hi - lo);
    card.querySelector('[data-f=track-actual]').style.left = pct(js.position) + '%';
    card.querySelector('[data-f=track-target]').style.left = pct(js.target) + '%';

    card.querySelectorAll('[data-mode]').forEach((btn) => {
      btn.classList.toggle('active', parseInt(btn.dataset.mode, 10) === js.mode);
    });
  });
}

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onopen = () => banner(null);

  ws.onclose = () => {
    el('subtitle').textContent = 'disconnected';
    banner('Lost the connection to scorbot_webd. Retrying...', 'warn');
    setTimeout(connect, 1500);
  };

  ws.onmessage = (ev) => {
    let msg;
    try {
      msg = JSON.parse(ev.data);
    } catch (e) {
      return;
    }
    if (msg.type === 'info') {
      buildJoints(msg);
      el('subtitle').textContent =
        `${msg.joints.length} joints  |  ${msg.transport}  |  session ${msg.session}` +
        (msg.observe ? '  |  OBSERVE ONLY' : '');
      el('stat-transport').textContent = msg.transport;
      el('stat-session').textContent = msg.observe ? 'observe' : msg.session;
    } else if (msg.type === 'state') {
      applyState(msg);
    } else if (msg.type === 'gains') {
      el('gain-kp').value = msg.kp;
      el('gain-ki').value = msg.ki;
      el('gain-kd').value = msg.kd;
    }
  };
}

el('estop').addEventListener('click', () => {
  send({ cmd: estopped ? 'clear_estop' : 'estop' });
});

el('home-all').addEventListener('click', () => {
  if (confirm('Home every joint?\n\nThe arm will move to each limit switch in turn. Keep a hand on the e-stop.')) {
    send({ cmd: 'home', mask: 0xff });
  }
});

el('home-one').addEventListener('click', () => {
  const idx = parseInt(el('home-joint-select').value, 10);
  const name = joints[idx] ? joints[idx].name : idx;
  if (confirm(`Home ${name}?\n\nIt will drive to its limit switch. Keep a hand on the e-stop.`)) {
    send({ cmd: 'home', mask: 1 << idx });
  }
});

el('gain-read').addEventListener('click', () => {
  send({ cmd: 'get_gains', joint: parseInt(el('gain-joint').value, 10) });
});

el('gain-write').addEventListener('click', () => {
  send({
    cmd: 'set_gains',
    joint: parseInt(el('gain-joint').value, 10),
    kp: parseFloat(el('gain-kp').value),
    ki: parseFloat(el('gain-ki').value),
    kd: parseFloat(el('gain-kd').value),
  });
});

document.querySelectorAll('[data-mode-all]').forEach((btn) => {
  btn.addEventListener('click', () => {
    send({ cmd: 'set_mode_all', value: parseInt(btn.dataset.modeAll, 10) });
  });
});

// Space bar is a hardware-free panic button while standing at the machine.
document.addEventListener('keydown', (e) => {
  if (e.code === 'Space' && e.target.tagName !== 'INPUT' && e.target.tagName !== 'SELECT') {
    e.preventDefault();
    if (!estopped) send({ cmd: 'estop' });
  }
});

connect();
