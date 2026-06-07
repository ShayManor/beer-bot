"""Self-contained operator console served at GET /.

Single HTML document (inline CSS + vanilla JS, no build step, no bundled
assets). Fonts are pulled from the client's own connection with a monospace
fallback, so the rover's Pi can stay offline. The `__TELEOP_V__` / `__TELEOP_W__`
tokens are substituted by the node with the configured teleop magnitudes.
"""

INDEX_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="theme-color" content="#07090b">
<title>AUTONOMOUS·ROVER — mission control</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Chakra+Petch:wght@400;500;600;700&family=IBM+Plex+Mono:wght@400;500;600&display=swap" rel="stylesheet">
<style>
:root{
  --v: __TELEOP_V__;
  --w: __TELEOP_W__;
  --bg:#07090b; --panel:#0e1318; --panel-2:#11171d;
  --line:#1c2630; --line-2:#28333f;
  --ink:#cdd8e2; --ink-dim:#74838f; --ink-faint:#475360;
  --idle:#f5a524; --idle-soft:rgba(245,165,36,.14);
  --live:#28e0a0; --live-soft:rgba(40,224,160,.14);
  --stop:#ff5a5a;
  --accent:var(--idle); --accent-soft:var(--idle-soft);
  --mono:"IBM Plex Mono", ui-monospace, "SF Mono", Menlo, Consolas, monospace;
  --disp:"Chakra Petch", var(--mono);
  --gap:14px; --radius:14px;
}
*{box-sizing:border-box}
html,body{margin:0;height:100%}
body{
  font-family:var(--mono); color:var(--ink); background:var(--bg);
  -webkit-font-smoothing:antialiased; text-rendering:optimizeLegibility;
  background-image:
    radial-gradient(1200px 600px at 78% -10%, var(--accent-soft), transparent 60%),
    radial-gradient(900px 500px at 0% 110%, rgba(40,224,160,.05), transparent 55%),
    linear-gradient(transparent 0 calc(100% - 1px), var(--line) calc(100% - 1px)),
    linear-gradient(90deg, transparent 0 calc(100% - 1px), var(--line) calc(100% - 1px));
  background-size:auto,auto,34px 34px,34px 34px;
  background-position:center,center,center,center;
  transition:--accent .4s;
}
body::before{ /* grain */
  content:""; position:fixed; inset:0; pointer-events:none; opacity:.035; z-index:0;
  background-image:url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='120' height='120'><filter id='n'><feTurbulence type='fractalNoise' baseFrequency='.9' numOctaves='2'/></filter><rect width='100%25' height='100%25' filter='url(%23n)'/></svg>");
}
.wrap{position:relative; z-index:1; max-width:1080px; margin:0 auto; padding:22px 16px 40px; padding-bottom:max(40px,env(safe-area-inset-bottom));}

/* ---------- header ---------- */
header{display:flex; align-items:center; justify-content:space-between; gap:12px; margin-bottom:20px; flex-wrap:wrap}
.brand{display:flex; align-items:baseline; gap:12px; min-width:0}
.kicker{font-size:10px; letter-spacing:.42em; color:var(--ink-dim); text-transform:uppercase}
.logo{font-family:var(--disp); font-weight:700; font-size:26px; letter-spacing:.02em; line-height:1;
  background:linear-gradient(180deg,#fff,#9fb0bd); -webkit-background-clip:text; background-clip:text; color:transparent}
.logo b{color:var(--accent); -webkit-text-fill-color:var(--accent)}
.hud{display:flex; align-items:center; gap:10px}
.link{display:flex; align-items:center; gap:7px; font-size:11px; letter-spacing:.12em; color:var(--ink-dim); text-transform:uppercase}
.dot{width:8px; height:8px; border-radius:50%; background:var(--ink-faint); box-shadow:0 0 0 0 transparent}
.link.ok .dot{background:var(--live); box-shadow:0 0 10px var(--live); animation:blink 2.4s infinite}
.link.down .dot{background:var(--stop); box-shadow:0 0 10px var(--stop)}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.35}}
.badge{font-family:var(--disp); font-weight:600; font-size:13px; letter-spacing:.22em; text-transform:uppercase;
  padding:9px 16px; border-radius:999px; color:var(--accent);
  border:1px solid color-mix(in srgb, var(--accent) 45%, transparent);
  background:var(--accent-soft); white-space:nowrap}
body.live .badge{animation:pulse 1.8s ease-in-out infinite}
@keyframes pulse{0%,100%{box-shadow:0 0 0 0 var(--accent-soft)}50%{box-shadow:0 0 0 7px transparent}}

/* ---------- layout ---------- */
.grid{display:grid; gap:var(--gap); grid-template-columns:1fr;
  grid-template-areas:"drive" "state" "tele" "log";}
@media(min-width:760px){
  .grid{grid-template-columns:minmax(0,1.15fr) minmax(0,1fr);
    grid-template-areas:"drive state" "drive tele" "log log";}
}
.card{position:relative; background:linear-gradient(180deg,var(--panel),var(--panel-2));
  border:1px solid var(--line); border-radius:var(--radius); padding:16px;
  box-shadow:0 1px 0 rgba(255,255,255,.03) inset, 0 18px 40px -28px #000;
  opacity:0; transform:translateY(10px); animation:rise .5s cubic-bezier(.2,.7,.2,1) forwards}
.card:nth-child(1){animation-delay:.04s}.card:nth-child(2){animation-delay:.1s}
.card:nth-child(3){animation-delay:.16s}.card:nth-child(4){animation-delay:.22s}
@keyframes rise{to{opacity:1; transform:none}}
.drive{grid-area:drive}.state{grid-area:state}.tele{grid-area:tele}.log{grid-area:log}
.card h2{margin:0 0 14px; font-family:var(--disp); font-weight:600; font-size:12px;
  letter-spacing:.26em; text-transform:uppercase; color:var(--ink-dim);
  display:flex; align-items:center; gap:8px}
.card h2::before{content:""; width:6px; height:6px; border-radius:1px; background:var(--accent); box-shadow:0 0 8px var(--accent)}

/* ---------- d-pad ---------- */
.pad{display:grid; gap:10px; grid-template-columns:repeat(3,1fr);
  grid-template-areas:". up ." "left stop right" ". down ."; max-width:340px; margin:6px auto 4px}
.key{appearance:none; -webkit-tap-highlight-color:transparent; touch-action:none; user-select:none;
  border:1px solid var(--line-2); background:#0c1217; color:var(--ink);
  border-radius:12px; aspect-ratio:1/1; font-size:26px; cursor:pointer;
  display:flex; align-items:center; justify-content:center;
  transition:transform .06s, background .12s, border-color .12s, box-shadow .12s}
.key:hover{border-color:var(--accent)}
.key.on{background:var(--accent-soft); border-color:var(--accent);
  box-shadow:0 0 0 1px var(--accent) inset, 0 0 22px -4px var(--accent); transform:translateY(1px) scale(.97)}
.key.up{grid-area:up}.key.down{grid-area:down}.key.left{grid-area:left}.key.right{grid-area:right}
.key.stop{grid-area:stop; font-family:var(--disp); font-weight:700; font-size:13px; letter-spacing:.16em; color:var(--stop);
  border-color:color-mix(in srgb,var(--stop) 40%, transparent)}
.key.stop:hover{border-color:var(--stop); background:rgba(255,90,90,.12)}
.hint{margin-top:12px; text-align:center; font-size:11px; color:var(--ink-faint); letter-spacing:.04em}
.hint kbd{font-family:var(--mono); color:var(--ink-dim); border:1px solid var(--line-2);
  border-radius:5px; padding:1px 5px; font-size:10px}

/* ---------- state machine ---------- */
.machine{display:flex; align-items:center; justify-content:center; gap:6px; padding:6px 0 4px}
.snode{flex:1; max-width:150px; cursor:pointer; text-align:center; border-radius:12px; padding:16px 8px;
  border:1px solid var(--line-2); background:#0c1217; transition:.18s; position:relative}
.snode .nm{font-family:var(--disp); font-weight:700; letter-spacing:.18em; font-size:15px}
.snode .sub{font-size:10px; letter-spacing:.18em; color:var(--ink-faint); text-transform:uppercase; margin-top:4px}
.snode[data-s="idle"]{--c:var(--idle)} .snode[data-s="active"]{--c:var(--live)}
.snode.cur{border-color:var(--c); background:color-mix(in srgb,var(--c) 12%, #0c1217);
  box-shadow:0 0 26px -8px var(--c)}
.snode.cur .nm{color:var(--c)} .snode:not(.cur):hover{border-color:var(--c)}
.flow{display:flex; flex-direction:column; align-items:center; color:var(--ink-faint); font-size:11px; gap:2px; padding:0 2px}
.flow .ar{font-size:18px; line-height:1}
.note{margin-top:12px; font-size:11px; color:var(--ink-faint); text-align:center; letter-spacing:.03em}

/* ---------- telemetry ---------- */
.tgrid{display:grid; grid-template-columns:repeat(2,1fr); gap:1px; background:var(--line); border:1px solid var(--line); border-radius:10px; overflow:hidden}
.cell{background:var(--panel-2); padding:12px 13px}
.cell .l{font-size:9.5px; letter-spacing:.22em; color:var(--ink-faint); text-transform:uppercase}
.cell .val{font-size:21px; margin-top:5px; font-weight:500; font-variant-numeric:tabular-nums; color:var(--ink)}
.cell .val small{font-size:11px; color:var(--ink-dim); margin-left:3px; font-weight:400}

/* ---------- logs ---------- */
.feed{height:200px; overflow-y:auto; border:1px solid var(--line); border-radius:10px; background:#080c0f;
  padding:10px 12px; font-size:12.5px; line-height:1.7; scrollbar-width:thin; scrollbar-color:var(--line-2) transparent}
.feed::-webkit-scrollbar{width:8px}.feed::-webkit-scrollbar-thumb{background:var(--line-2); border-radius:8px}
.row{display:flex; gap:10px; white-space:pre-wrap; word-break:break-word; animation:fade .3s ease}
.row .ts{color:var(--accent); opacity:.8; flex:none; font-variant-numeric:tabular-nums}
.row .mg{color:var(--ink)}
.feed .empty{color:var(--ink-faint)}
@keyframes fade{from{opacity:0}to{opacity:1}}
</style>
</head>
<body class="idle">
<div class="wrap">
  <header>
    <div class="brand">
      <div>
        <div class="kicker">Mission Control</div>
        <div class="logo">AUTONOMOUS<b>·</b>ROVER</div>
      </div>
    </div>
    <div class="hud">
      <span class="link" id="link"><span class="dot"></span><span id="linkTxt">linking…</span></span>
      <span class="badge" id="badge">—</span>
    </div>
  </header>

  <div class="grid">
    <section class="card drive">
      <h2>Manual Drive</h2>
      <div class="pad">
        <button class="key up"    data-dir="up"    aria-label="forward">▲</button>
        <button class="key left"  data-dir="left"  aria-label="turn left">◀</button>
        <button class="key stop"  data-dir="stop"  aria-label="stop">STOP</button>
        <button class="key right" data-dir="right" aria-label="turn right">▶</button>
        <button class="key down"  data-dir="down"  aria-label="reverse">▼</button>
      </div>
      <div class="hint">Hold <kbd>↑</kbd><kbd>↓</kbd><kbd>←</kbd><kbd>→</kbd> or <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> · combine to arc · release to stop</div>
    </section>

    <section class="card state">
      <h2>State Machine</h2>
      <div class="machine">
        <div class="snode" data-s="idle"><div class="nm">IDLE</div><div class="sub">no movement</div></div>
        <div class="flow"><span class="ar">⇄</span><span>set</span></div>
        <div class="snode" data-s="active"><div class="nm">ACTIVE</div><div class="sub">movement</div></div>
      </div>
      <div class="note">Tap a state to switch. Sending a goal arms <b>ACTIVE</b>.</div>
    </section>

    <section class="card tele">
      <h2>Telemetry</h2>
      <div class="tgrid">
        <div class="cell"><div class="l">Position X</div><div class="val" id="px">—</div></div>
        <div class="cell"><div class="l">Position Y</div><div class="val" id="py">—</div></div>
        <div class="cell"><div class="l">Heading</div><div class="val" id="hd">—</div></div>
        <div class="cell"><div class="l">Speed</div><div class="val" id="sp">—</div></div>
        <div class="cell"><div class="l">Turn rate</div><div class="val" id="om">—</div></div>
        <div class="cell"><div class="l">Goal</div><div class="val" id="gl">—</div></div>
      </div>
    </section>

    <section class="card log">
      <h2>Log Stream</h2>
      <div class="feed" id="feed"><div class="empty">awaiting telemetry…</div></div>
    </section>
  </div>
</div>

<script>
const V = parseFloat(getComputedStyle(document.documentElement).getPropertyValue('--v')) || 0.35;
const W = parseFloat(getComputedStyle(document.documentElement).getPropertyValue('--w')) || 1.0;
const $ = s => document.querySelector(s);
const body = document.body;

/* ---- connection helper ---- */
let online = null;
function setLink(ok){
  if(ok===online) return; online=ok;
  const el=$('#link');
  el.classList.toggle('ok',ok); el.classList.toggle('down',!ok);
  $('#linkTxt').textContent = ok ? 'link ok' : 'link lost';
}
async function api(path,opts){
  try{ const r=await fetch(path,opts); setLink(true); return r; }
  catch(e){ setLink(false); throw e; }
}
const j=(path,body)=>api(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});

/* ---- teleop: compose held directions into one (v, omega) ---- */
const held={up:false,down:false,left:false,right:false};
let last='';
function pushDrive(){
  const v=(held.up?V:0)+(held.down?-V:0);
  const w=(held.left?W:0)+(held.right?-W:0);
  const key=v+'|'+w;
  if(key===last) return; last=key;
  j('/teleop',{v,omega:w}).catch(()=>{});
}
function setDir(dir,on){
  if(dir==='stop'){ held.up=held.down=held.left=held.right=false; }
  else { held[dir]=on; }
  document.querySelectorAll('.key').forEach(k=>{
    const d=k.dataset.dir; if(d in held) k.classList.toggle('on',held[d]);
  });
  pushDrive();
}
function allStop(){ setDir('stop',false); }
// keepalive resend so a single dropped packet can't strand a held key
setInterval(()=>{ if(held.up||held.down||held.left||held.right){ last=''; pushDrive(); } },300);

/* keyboard */
const KMAP={ArrowUp:'up',KeyW:'up',ArrowDown:'down',KeyS:'down',ArrowLeft:'left',KeyA:'left',ArrowRight:'right',KeyD:'right'};
addEventListener('keydown',e=>{ const d=KMAP[e.code]; if(!d)return; e.preventDefault(); if(!held[d]) setDir(d,true); });
addEventListener('keyup',e=>{ const d=KMAP[e.code]; if(!d)return; e.preventDefault(); setDir(d,false); });
addEventListener('blur',allStop);
document.addEventListener('visibilitychange',()=>{ if(document.hidden) allStop(); });

/* on-screen pad (pointer = mouse + touch) */
document.querySelectorAll('.key').forEach(k=>{
  const d=k.dataset.dir;
  const down=e=>{e.preventDefault(); d==='stop'?allStop():setDir(d,true);};
  const up=e=>{e.preventDefault(); if(d!=='stop') setDir(d,false);};
  k.addEventListener('pointerdown',down);
  k.addEventListener('pointerup',up);
  k.addEventListener('pointerleave',up);
  k.addEventListener('pointercancel',up);
});

/* ---- state machine ---- */
function applyState(s){
  const live = s==='active';
  body.classList.toggle('live',live);
  body.classList.toggle('idle',!live);
  document.documentElement.style.setProperty('--accent', live?'var(--live)':'var(--idle)');
  document.documentElement.style.setProperty('--accent-soft', live?'var(--live-soft)':'var(--idle-soft)');
  $('#badge').textContent = s? s.toUpperCase() : '—';
  document.querySelectorAll('.snode').forEach(n=>n.classList.toggle('cur',n.dataset.s===s));
}
document.querySelectorAll('.snode').forEach(n=>{
  n.addEventListener('click',()=>{ const s=n.dataset.s; applyState(s); j('/state',{state:s}).catch(()=>{}); });
});

/* ---- telemetry poll ---- */
const fmt=(x,d=2)=> (x===null||x===undefined)?'—':(+x).toFixed(d);
async function poll(){
  try{
    const s=await(await api('/status')).json();
    applyState(s.state);
    const p=s.pose;
    $('#px').innerHTML = p? fmt(p.x)+'<small>m</small>':'—';
    $('#py').innerHTML = p? fmt(p.y)+'<small>m</small>':'—';
    $('#hd').innerHTML = (s.heading==null)?'—':fmt(s.heading*180/Math.PI,0)+'<small>°</small>';
    $('#sp').innerHTML = (s.speed&&s.speed.v!=null)? fmt(s.speed.v)+'<small>m/s</small>':'—';
    $('#om').innerHTML = (s.speed&&s.speed.omega!=null)? fmt(s.speed.omega)+'<small>rad/s</small>':'—';
    $('#gl').innerHTML = s.goal? fmt(s.goal.x,1)+', '+fmt(s.goal.y,1):'—';
  }catch(e){}
}

/* ---- log poll ---- */
const feed=$('#feed'); let logN=-1;
async function pollLogs(){
  try{
    const d=await(await api('/logs')).json();
    const logs=d.logs||[];
    if(logs.length===logN) return; logN=logs.length;
    const near = feed.scrollHeight-feed.scrollTop-feed.clientHeight < 60;
    if(!logs.length){ feed.innerHTML='<div class="empty">no events yet…</div>'; return; }
    feed.innerHTML = logs.map(e=>{
      const t=(+e.t).toFixed(1).padStart(7,' ');
      const m=String(e.msg).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
      return '<div class="row"><span class="ts">'+t+'</span><span class="mg">'+m+'</span></div>';
    }).join('');
    if(near) feed.scrollTop=feed.scrollHeight;
  }catch(e){}
}

poll(); pollLogs();
setInterval(poll,600);
setInterval(pollLogs,1500);
</script>
</body>
</html>"""
