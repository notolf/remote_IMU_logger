// =============================================================================
//  web_page.h — embedded phone dashboard (single self-contained HTML page)
// -----------------------------------------------------------------------------
//  Served from PROGMEM at "/". Designed for a phone on the device's soft-AP:
//  there is NO internet on that link, so everything (CSS, JS, chart, icons) is
//  inline — no CDNs, no web fonts, no chart library.
//
//  Styling follows the NOVA design system (Hexagon / Leica Geosystems):
//  sys.palette colour tokens for light + dark mode (prefers-color-scheme) and
//  the label/body/title type roles. Nova's typeface (Hexagon Akkurat) is
//  licensed and cannot be embedded here, so a close system-font stack with
//  tabular numerals is used for the instrument readouts.
//
//  Data flow: poll GET /api/status at ~5 Hz, backfill the trend chart once from
//  GET /api/history, send commands as form-encoded POSTs (/api/mark, /api/log,
//  /api/cal, /api/time, /api/settings). All handled in the firmware main loop.
// =============================================================================
#pragma once
#include <pgmspace.h>

static const char WEB_PAGE_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="color-scheme" content="light dark">
<title>Static Level Logger</title>
<style>
/* ---- NOVA sys.palette tokens (light), dark overrides below ---------------- */
:root{
  --surface:#F8FAFD; --surface-c:#FFFFFF; --surface-hi:#E6EAF0; --surface-lo:#F5F7FA;
  --on:#121623; --on-var:#646E78; --outline:#858C99; --outline-var:#E6EAF0;
  --primary:#005198; --on-primary:#FFFFFF;
  --success:#6DBD55; --success-c:#E2FAD5; --on-success-c:#3A6E2B;
  --warning:#F19724; --warning-c:#FEECD1; --on-warning-c:#8E5515;
  --error:#BC1C1C;  --error-c:#FCEAE7;  --on-error-c:#BC1C1C;
  --info:#33B4F2;   --brand:#83C410;    --disabled:#B4BAC4;
}
@media (prefers-color-scheme:dark){:root{
  --surface:#161A24; --surface-c:#2B2F3F; --surface-hi:#474F5F; --surface-lo:#3B3F4E;
  --on:#F5F7FA; --on-var:#B4BAC4; --outline:#646E78; --outline-var:#474F5F;
  --primary:#01ADFF; --on-primary:#121623;
  --success:#7DD166; --success-c:#255521; --on-success-c:#E2FAD5;
  --warning:#FBAF35; --warning-c:#8E5515; --on-warning-c:#FEECD1;
  --error:#FD877E;  --error-c:#8D0F0F;  --on-error-c:#FCEAE7;
  --info:#65C8FD;   --brand:#83C410;    --disabled:#646E78;
}}
*{box-sizing:border-box;margin:0;padding:0}
html{-webkit-text-size-adjust:100%}
body{
  background:var(--surface);color:var(--on);
  font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  padding:10px;max-width:560px;margin:0 auto;
}
.num{font-variant-numeric:tabular-nums}
h1{font-size:16px;font-weight:700;letter-spacing:.08em}
header{padding:6px 2px 10px}
.brandline{height:3px;background:var(--brand);border-radius:2px;margin:6px 0 8px;width:64px}
.hrow{display:flex;flex-wrap:wrap;gap:6px;align-items:center}
.chip{display:inline-flex;align-items:center;gap:5px;border:1px solid var(--outline-var);
  background:var(--surface-c);border-radius:8px;padding:2px 8px;font-size:12px;color:var(--on-var)}
.chip b{color:var(--on);font-weight:600}
.dot{width:8px;height:8px;border-radius:50%;background:var(--success)}
.live .dot{animation:pulse 1.2s infinite}
@keyframes pulse{50%{opacity:.35}}
#stale{display:none;background:var(--error-c);color:var(--on-error-c);
  border:1px solid var(--error);border-radius:10px;padding:8px 12px;margin:8px 0;font-weight:600}
body.stale #stale{display:block}
body.stale .fade{opacity:.45}
.card{background:var(--surface-c);border:1px solid var(--outline-var);border-radius:14px;
  padding:12px;margin:10px 0}
.card h2{font-size:11px;font-weight:700;letter-spacing:.12em;color:var(--on-var);
  text-transform:uppercase;margin-bottom:8px}
.row{display:flex;gap:12px;flex-wrap:wrap;align-items:flex-start}
/* level vial + readouts */
#vialwrap{flex:0 0 auto}
#readouts{flex:1 1 150px;min-width:150px}
.axlabel{font-size:11px;letter-spacing:.1em;color:var(--on-var)}
.axval{font-size:30px;font-weight:700;line-height:1.15}
.axval small{font-size:16px;font-weight:400;color:var(--on-var)}
.state-chip{display:inline-block;border-radius:7px;padding:2px 9px;font-size:12px;font-weight:700}
.st-ok{background:var(--success-c);color:var(--on-success-c)}
.st-warn{background:var(--warning-c);color:var(--on-warning-c)}
.st-err{background:var(--error-c);color:var(--on-error-c)}
.qrow{display:flex;align-items:center;gap:7px;margin-top:6px;font-size:12px;color:var(--on-var)}
.qrow .lab{width:52px;flex:0 0 auto}
.qbar{flex:1;height:8px;background:var(--surface-hi);border-radius:4px;position:relative;overflow:hidden}
.qbar i{position:absolute;left:0;top:0;bottom:0;background:var(--success);border-radius:4px;transition:width .15s}
.qbar.over i{background:var(--warning)}
.qbar u{position:absolute;left:50%;top:-1px;bottom:-1px;width:2px;background:var(--outline)}
.qrow .val{width:74px;text-align:right;flex:0 0 auto;color:var(--on)}
/* segmented range buttons */
.seg{display:inline-flex;border:1px solid var(--outline-var);border-radius:9px;overflow:hidden;margin-top:8px}
.seg button{border:0;background:transparent;color:var(--on-var);padding:4px 10px;font-size:12px;cursor:pointer}
.seg button.on{background:var(--primary);color:var(--on-primary);font-weight:700}
/* buttons */
.btn{border:0;border-radius:10px;padding:10px 14px;font-size:14px;font-weight:700;cursor:pointer}
.btn:disabled{opacity:.45}
.btn-pri{background:var(--primary);color:var(--on-primary)}
.btn-ok{background:var(--success-c);color:var(--on-success-c)}
.btn-err{background:var(--error-c);color:var(--on-error-c)}
.btn-ton{background:var(--surface-hi);color:var(--on)}
.btn-line{background:transparent;border:1px solid var(--outline);color:var(--on)}
.wide{width:100%;margin-top:8px}
input[type=text],input[type=number]{
  background:var(--surface-lo);color:var(--on);border:1px solid var(--outline-var);
  border-radius:9px;padding:8px 10px;font-size:14px;width:100%}
input:focus{outline:2px solid var(--primary);outline-offset:-1px}
.chips{display:flex;gap:6px;flex-wrap:wrap;margin-top:7px}
.chips button{border:1px solid var(--outline-var);background:var(--surface-lo);color:var(--on-var);
  border-radius:14px;padding:3px 10px;font-size:12px;cursor:pointer}
.meta{font-size:12px;color:var(--on-var);margin-top:7px}
.meta b{color:var(--on)}
canvas{width:100%;height:215px;display:block;background:var(--surface-lo);
  border:1px solid var(--outline-var);border-radius:10px}
.legend{display:flex;gap:6px;flex-wrap:wrap;margin:7px 0 0}
.legend button{display:inline-flex;align-items:center;gap:5px;border:1px solid var(--outline-var);
  background:var(--surface-lo);border-radius:14px;padding:2px 10px;font-size:12px;color:var(--on-var);cursor:pointer}
.legend button.off{opacity:.4}
.legend i{width:10px;height:3px;border-radius:2px;display:inline-block}
.stats{display:grid;grid-template-columns:auto repeat(5,1fr);gap:3px 8px;font-size:12px;margin-top:4px}
.stats div{text-align:right}.stats .h{color:var(--on-var);text-align:right;font-size:11px}
.stats .rh{text-align:left;color:var(--on-var)}
.evlist{font-size:12px;color:var(--on-var);margin-top:8px}
.evlist b{color:var(--on)}
/* settings grid */
.set{display:grid;grid-template-columns:1fr 110px;gap:8px 10px;align-items:center;margin-top:8px}
.set label{font-size:12.5px;color:var(--on-var)}
.set label b{color:var(--on);display:block;font-size:13px}
.set input,.set select{width:100%}
select{background:var(--surface-lo);color:var(--on);border:1px solid var(--outline-var);
  border-radius:9px;padding:8px 6px;font-size:14px}
.btnrow{display:flex;gap:8px;margin-top:12px}
.btnrow .btn{flex:1}
details>summary{cursor:pointer;font-size:11px;font-weight:700;letter-spacing:.12em;
  color:var(--on-var);text-transform:uppercase;list-style:none}
details>summary::after{content:" +";color:var(--outline)}
details[open]>summary::after{content:" \2212"}
.calmsg{border-radius:10px;padding:9px 12px;margin-top:8px;font-size:13px}
.calmsg.ok{background:var(--success-c);color:var(--on-success-c)}
.calmsg.err{background:var(--error-c);color:var(--on-error-c)}
.pbar{height:14px;background:var(--surface-hi);border-radius:7px;overflow:hidden;margin-top:8px}
.pbar i{display:block;height:100%;background:var(--primary);width:0;transition:width .25s}
footer{font-size:11px;color:var(--on-var);text-align:center;padding:12px 0 20px}
</style>
</head>
<body>

<header>
  <h1>STATIC LEVEL LOGGER</h1>
  <div class="brandline"></div>
  <div class="hrow">
    <span class="chip live" id="livechip"><span class="dot"></span><b id="livetxt">LIVE</b></span>
    <span class="chip">BAT <b class="num" id="bat">–</b></span>
    <span class="chip">IMU <b class="num" id="temp">–</b></span>
    <span class="chip" id="clockchip">CLOCK <b class="num" id="clock">–</b></span>
  </div>
</header>

<div id="stale">CONNECTION LOST — data is stale. Stay on the LevelLogger WiFi.</div>

<!-- ======================= LEVEL ======================= -->
<div class="card fade">
  <h2>Level</h2>
  <div class="row">
    <div id="vialwrap">
      <svg id="vial" width="225" height="225" viewBox="-112 -112 224 224">
        <line x1="-100" y1="0" x2="100" y2="0" stroke-width="1"/>
        <line x1="0" y1="-100" x2="0" y2="100" stroke-width="1"/>
        <circle id="ring3" cx="0" cy="0" r="100" fill="none" stroke-width="1.5"/>
        <circle id="ring2" cx="0" cy="0" r="66.7" fill="none" stroke-width="1"/>
        <circle id="ring1" cx="0" cy="0" r="33.3" fill="none" stroke-width="1"/>
        <circle id="tolring" cx="0" cy="0" r="10" fill="none" stroke-width="1.5"/>
        <text id="lr1" x="3" y="-35" font-size="9"></text>
        <text id="lr2" x="3" y="-68.5" font-size="9"></text>
        <text id="lr3" x="3" y="-101.5" font-size="9"></text>
        <circle id="bubble" cx="0" cy="0" r="12" stroke-width="1.5"/>
        <circle id="gloss" cx="-4" cy="-4" r="3.2" fill="#fff" opacity=".8"/>
        <circle cx="0" cy="0" r="1.6" id="centerdot"/>
      </svg>
      <div>
        <span class="seg" id="rangeseg"></span>
        <div class="meta">full scale <b class="num" id="fs">±2.0°</b></div>
      </div>
    </div>
    <div id="readouts">
      <div class="axlabel">PITCH</div>
      <div class="axval num" id="pv">+0.000<small>°</small></div>
      <div class="axlabel" style="margin-top:6px">ROLL</div>
      <div class="axval num" id="rv">+0.000<small>°</small></div>
      <div style="margin-top:8px">
        <span class="state-chip st-warn" id="settled">…</span>
        <span class="state-chip st-err" id="calchip" style="margin-left:5px">…</span>
      </div>
      <div class="qrow"><span class="lab">avg n</span><span class="qbar"><i id="avgbar"></i></span><span class="val num" id="avgtxt">0/500</span></div>
      <div class="qrow"><span class="lab">σ|a|</span><span class="qbar" id="sigbarw"><i id="sigbar"></i><u></u></span><span class="val num" id="sigtxt">–</span></div>
      <div class="qrow"><span class="lab">gyro</span><span class="qbar" id="gyrobarw"><i id="gyrobar"></i><u></u></span><span class="val num" id="gyrotxt">–</span></div>
    </div>
  </div>
</div>

<!-- ======================= TREND ======================= -->
<div class="card fade">
  <h2>Trend</h2>
  <canvas id="chart"></canvas>
  <div class="legend">
    <button id="lgP" class=""><i style="background:var(--primary)"></i>pitch</button>
    <button id="lgR" class=""><i style="background:var(--info)"></i>roll</button>
    <button id="lgT" class="off"><i style="background:var(--warning)"></i>IMU temp</button>
    <span style="flex:1"></span>
    <button id="win5">5 min</button>
    <button id="win15" class="off">15 min</button>
  </div>
  <div class="stats num" id="stats"></div>
  <div class="evlist" id="evlist"></div>
</div>

<!-- ======================= CONTROLS ======================= -->
<div class="card fade">
  <h2>Controls</h2>
  <input type="text" id="marklabel" maxlength="31" placeholder="event label (optional)">
  <div class="chips" id="presetchips"></div>
  <button class="btn btn-pri wide" id="markbtn">⚑ MARK EVENT</button>
  <button class="btn btn-ok wide" id="logbtn">▶ START LOGGING</button>
  <div class="meta">file <b id="file">–</b> · rows <b class="num" id="rows">0</b> · SD <b id="sds">–</b> · events <b class="num" id="evn">0</b></div>
  <div class="meta">device clock <b class="num" id="devclock">–</b> <span id="drift"></span></div>
  <button class="btn btn-ton wide" id="syncbtn">⏱ Sync clock from phone</button>
</div>

<!-- ======================= CALIBRATION ======================= -->
<div class="card fade">
  <h2>Calibration</h2>
  <div id="calidle">
    <div class="meta" id="calinfo">–</div>
    <button class="btn btn-ton wide" id="calstart">Start flip calibration</button>
  </div>
  <div id="calwiz" style="display:none">
    <div class="meta" id="calprompt" style="font-size:13.5px;color:var(--on)"></div>
    <div class="pbar"><i id="calprog"></i></div>
    <div class="btnrow">
      <button class="btn btn-pri" id="calnext">NEXT</button>
      <button class="btn btn-err" id="calcancel">CANCEL</button>
    </div>
  </div>
  <div class="calmsg" id="calmsg" style="display:none"></div>
</div>

<!-- ======================= SETTINGS ======================= -->
<div class="card fade">
  <details id="setdet">
    <summary>Settings — stationary detection &amp; logging</summary>
    <div class="set">
      <label><b>Gyro quiet threshold</b>deg/s — below = still</label>
      <input type="number" id="s_gyroTh" step="0.05" min="0.05" max="10">
      <label><b>Accel σ threshold</b>g — std(|a|) below = quiet</label>
      <input type="number" id="s_sigTh" step="0.0005" min="0.0005" max="0.05">
      <label><b>|a| tolerance</b>g — allowed offset from 1 g</label>
      <input type="number" id="s_magTol" step="0.01" min="0.01" max="0.5">
      <label><b>Dwell</b>ms still before SETTLED</label>
      <input type="number" id="s_dwellMs" step="100" min="200" max="10000">
      <label><b>Averaging window</b>samples @100 Hz (50–1000)</label>
      <input type="number" id="s_avgN" step="50" min="50" max="1000">
      <label><b>Settled log interval</b>ms per CSV row</label>
      <input type="number" id="s_logMs" step="100" min="100" max="60000">
      <label><b>Log while moving</b>also write settled=0 rows</label>
      <select id="s_logMove"><option value="0">off</option><option value="1">on</option></select>
      <label><b>Moving log interval</b>ms</label>
      <input type="number" id="s_moveMs" step="100" min="100" max="60000">
      <label><b>Level tolerance</b>deg — bubble turns green</label>
      <input type="number" id="s_tol" step="0.01" min="0.02" max="5">
      <label><b>Pitch sign</b>flip if polarity is wrong</label>
      <select id="s_psign"><option value="1">+1</option><option value="-1">−1</option></select>
      <label><b>Roll sign</b></label>
      <select id="s_rsign"><option value="1">+1</option><option value="-1">−1</option></select>
      <label><b>Swap pitch/roll</b>axes exchanged on the mount</label>
      <select id="s_swap"><option value="0">no</option><option value="1">yes</option></select>
    </div>
    <div class="btnrow">
      <button class="btn btn-pri" id="setapply">Apply</button>
      <button class="btn btn-line" id="setreload">Reload</button>
      <button class="btn btn-line" id="setdef">Factory defaults</button>
    </div>
    <div class="meta">Stored in device NVS. Changing the averaging window restarts the running average.</div>
  </details>
</div>

<footer id="foot">–</footer>

<script>
'use strict';
/* ---------- helpers ---------- */
const $=id=>document.getElementById(id);
const css=n=>getComputedStyle(document.body).getPropertyValue(n).trim();
const fmt=(v,d)=>(v>=0?'+':'')+v.toFixed(d);
const post=(url,obj)=>fetch(url,{method:'POST',
  headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:new URLSearchParams(obj).toString()});
/* ---------- state ---------- */
let st=null, stale=true, lastOk=0;
let buf=[];                 // {ms,p,r,t,settled,ev}
let winMs=5*60*1000;
let showP=true,showR=true,showT=false;
let rangeMode='auto';       // 'auto' | number
const STEPS=[0.5,1,2,5,10,20,45,90];
let autoIdx=2;
let lastEvCount=0;
/* ---------- range segmented control ---------- */
const segDefs=[['auto','AUTO'],[0.5,'0.5°'],[2,'2°'],[10,'10°'],[45,'45°']];
segDefs.forEach(([v,txt])=>{
  const b=document.createElement('button');b.textContent=txt;
  b.onclick=()=>{rangeMode=v;renderSeg();};
  b.dataset.v=v;$('rangeseg').appendChild(b);
});
function renderSeg(){[...$('rangeseg').children].forEach(b=>
  b.classList.toggle('on',String(rangeMode)===b.dataset.v));}
renderSeg();
function currentScale(){
  if(rangeMode!=='auto')return rangeMode;
  if(st){const m=Math.max(Math.abs(st.dp),Math.abs(st.dr));
    while(autoIdx<STEPS.length-1&&m>STEPS[autoIdx]*0.90)autoIdx++;
    while(autoIdx>0&&m<STEPS[autoIdx-1]*0.55)autoIdx--;}
  return STEPS[autoIdx];
}
/* ---------- preset chips ---------- */
['mark','before-adjust','after-adjust','disturbed'].forEach(p=>{
  const b=document.createElement('button');b.textContent=p;
  b.onclick=()=>{$('marklabel').value=p;};
  $('presetchips').appendChild(b);
});
/* ---------- bullseye ---------- */
function drawVial(){
  if(!st)return;
  const s=currentScale();
  $('fs').textContent='±'+(s<1?s.toFixed(1):s.toFixed(s<10?1:0))+'°';
  const fmtR=v=>v<1?v.toFixed(2).replace(/0$/,''):(v<10?v.toFixed(1):v.toFixed(0));
  $('lr1').textContent=fmtR(s/3);$('lr2').textContent=fmtR(2*s/3);$('lr3').textContent=fmtR(s);
  const ol=css('--outline'),ov=css('--outline-var'),sv=css('--on-var');
  ['ring3','ring2','ring1'].forEach((id,i)=>$(id).setAttribute('stroke',i?ov:ol));
  document.querySelectorAll('#vial line').forEach(l=>l.setAttribute('stroke',ov));
  ['lr1','lr2','lr3'].forEach(id=>$(id).setAttribute('fill',sv));
  $('centerdot').setAttribute('fill',sv);
  const tolR=Math.max(6,st.tol/s*100);
  $('tolring').setAttribute('r',Math.min(tolR,100));
  $('tolring').setAttribute('stroke',css('--success'));
  let nx=st.dr/s,ny=st.dp/s;const rr=Math.hypot(nx,ny);
  if(rr>1){nx/=rr;ny/=rr;}
  const R=100-12,bx=nx*R,by=-ny*R;
  const level=Math.abs(st.dp)<=st.tol&&Math.abs(st.dr)<=st.tol;
  const bub=$('bubble');
  bub.setAttribute('cx',bx);bub.setAttribute('cy',by);
  bub.setAttribute('fill',level?css('--success'):css('--info'));
  bub.setAttribute('stroke',css('--on'));
  $('gloss').setAttribute('cx',bx-4);$('gloss').setAttribute('cy',by-4);
}
/* ---------- status poll ---------- */
async function poll(){
  try{
    const r=await fetch('/api/status',{cache:'no-store'});
    if(!r.ok)throw 0;
    st=await r.json();lastOk=Date.now();
    if(stale){stale=false;document.body.classList.remove('stale');
      $('livetxt').textContent='LIVE';$('livechip').classList.add('live');}
    render();
  }catch(e){/* watchdog flags staleness */}
  setTimeout(poll,200);
}
setInterval(()=>{if(Date.now()-lastOk>1600&&!stale){stale=true;
  document.body.classList.add('stale');$('livetxt').textContent='STALE';
  $('livechip').classList.remove('live');}},500);
/* ---------- main render ---------- */
function render(){
  const showPitch=st.settled?st.pitch:st.dp, showRoll=st.settled?st.roll:st.dr;
  $('pv').innerHTML=fmt(showPitch,3)+'<small>°</small>';
  $('rv').innerHTML=fmt(showRoll,3)+'<small>°</small>';
  const sc=$('settled');
  sc.textContent=st.settled?'SETTLED':'MOVING';
  sc.className='state-chip '+(st.settled?'st-ok':'st-warn');
  const cc=$('calchip');
  cc.textContent=st.cal?'CALIBRATED':'UNCALIBRATED';
  cc.className='state-chip '+(st.cal?'st-ok':'st-err');
  $('pv').style.color=$('rv').style.color=st.settled?css('--success'):css('--warning');
  /* quality */
  $('avgbar').style.width=Math.min(100,100*st.n/st.nT)+'%';
  $('avgtxt').textContent=st.n+'/'+st.nT;
  if(st.sig>=0){
    $('sigbar').style.width=Math.min(100,50*st.sig/st.sigTh)+'%';
    $('sigbarw').classList.toggle('over',st.sig>=st.sigTh);
    $('sigtxt').textContent=(st.sig*1000).toFixed(2)+' mg';
  }else{$('sigbar').style.width='0%';$('sigtxt').textContent='–';}
  $('gyrobar').style.width=Math.min(100,50*st.gyro/st.gyroTh)+'%';
  $('gyrobarw').classList.toggle('over',st.gyro>=st.gyroTh);
  $('gyrotxt').textContent=st.gyro.toFixed(2)+' °/s';
  /* header */
  $('bat').textContent=st.bat>=0?st.bat+'%'+(st.chg?'⚡':''):'–';
  $('temp').textContent=st.temp!=null?st.temp.toFixed(1)+'°C':'–';
  $('clock').textContent=st.clock?new Date(st.epoch*1000)
    .toLocaleTimeString([],{hour12:false}):'NOT SET';
  /* controls */
  $('file').textContent=st.file||'–';
  $('rows').textContent=st.rows;
  $('sds').textContent=st.sds;
  $('evn').textContent=st.ev;
  const lb=$('logbtn');
  lb.textContent=st.log?'■ STOP LOGGING':'▶ START LOGGING';
  lb.className='btn wide '+(st.log?'btn-err':'btn-ok');
  lb.disabled=!st.sd;
  $('devclock').textContent=st.clock?new Date(st.epoch*1000).toLocaleString():'not set';
  const drift=st.clock?Math.round(Date.now()/1000-st.epoch):null;
  $('drift').textContent=drift==null?'':(Math.abs(drift)<=2?'(in sync)':'(off by '+drift+' s)');
  $('syncbtn').style.display=(st.clock&&Math.abs(drift)<=2)?'none':'';
  /* calibration */
  const wiz=st.cs!=='idle';
  $('calidle').style.display=wiz?'none':'';
  $('calwiz').style.display=wiz?'':'none';
  if(!wiz){
    let info='Status: <b>'+(st.cal?'calibrated':'UNCALIBRATED — zero offset')+'</b>';
    if(st.cal){
      info+=' · offsets <b>'+(st.ox*1000).toFixed(2)+' / '+(st.oy*1000).toFixed(2)+' mg</b>';
      if(st.calT!=null&&st.temp!=null){
        const dT=st.temp-st.calT;
        info+=' · ΔT since cal <b>'+fmt(dT,1)+'°C</b>';
        if(Math.abs(dT)>5)info+=' <span class="state-chip st-warn">consider recalibrating</span>';
      }
    }
    $('calinfo').innerHTML=info;
    $('calmsg').style.display='none';
  }else{
    const prompts={wait_a:'Step 1/2 — place the device FLAT on the granite plate (orientation A). Hold still, then press NEXT.',
      cap_a:'Capturing orientation A — keep absolutely still…',
      wait_b:'Step 2/2 — rotate 180° about the VERTICAL axis, same spot. Hold still, then press NEXT.',
      cap_b:'Capturing orientation B — keep absolutely still…',
      done:st.cm};
    let ptxt=prompts[st.cs]||'';
    if(st.cr&&(st.cs==='cap_a'||st.cs==='cap_b'))
      ptxt+=' (restarted '+st.cr+'× — vibration/drift detected)';
    $('calprompt').textContent=ptxt;
    $('calprog').style.width=st.cp+'%';
    const capturing=st.cs==='cap_a'||st.cs==='cap_b';
    $('calnext').disabled=capturing;
    $('calnext').textContent=st.cs==='done'?'FINISH':'NEXT';
    const m=$('calmsg');
    if(st.cs==='done'){m.style.display='';m.textContent=st.cm;
      m.className='calmsg '+(st.ce?'err':'ok');}
    else m.style.display='none';
  }
  /* footer */
  $('foot').textContent='WiFi '+st.wifi+(st.wifi==='AP'?' · '+st.clients+' client(s)':' · RSSI '+st.rssi+' dBm')
    +' · uptime '+fmtUp(st.up)+' · dwell '+st.dwellMs+' ms';
  /* chart buffer append at ~1 Hz */
  if(!buf.length||st.up-buf[buf.length-1].ms>=995){
    buf.push({ms:st.up,p:showPitch,r:showRoll,t:st.temp,settled:st.settled,
      ev:st.ev>lastEvCount});
    lastEvCount=st.ev;
    if(buf.length>1900)buf.splice(0,buf.length-1900);
    drawChart();drawStats();drawEvents();
  }
  drawVial();
}
const fmtUp=ms=>{const s=Math.floor(ms/1000);
  return Math.floor(s/3600)+':'+String(Math.floor(s/60)%60).padStart(2,'0')
  +':'+String(s%60).padStart(2,'0');};
/* ---------- history backfill ---------- */
fetch('/api/history').then(r=>r.json()).then(h=>{
  const seeded=h.e.map(e=>({ms:e[0],p:e[1],r:e[2],t:e[3],settled:!!(e[4]&1),ev:!!(e[4]&2)}));
  buf=seeded.concat(buf.filter(b=>!seeded.length||b.ms>seeded[seeded.length-1].ms));
  drawChart();drawStats();
}).catch(()=>{});
/* ---------- trend chart (hand-rolled canvas, no libs) ---------- */
function drawChart(){
  const cv=$('chart'),dpr=window.devicePixelRatio||1;
  const W=cv.clientWidth,H=cv.clientHeight;
  if(cv.width!==W*dpr){cv.width=W*dpr;cv.height=H*dpr;}
  const g=cv.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,W,H);
  const now=st?st.up:(buf.length?buf[buf.length-1].ms:0);
  const data=buf.filter(b=>b.ms>=now-winMs);
  const ml=42,mr=showT?34:8,mt=6,mb=18,iw=W-ml-mr,ih=H-mt-mb;
  g.font='10px system-ui';
  if(data.length<2){g.fillStyle=css('--on-var');
    g.fillText('collecting…',ml,H/2);return;}
  let lo=1e9,hi=-1e9;
  for(const d of data){
    if(showP){lo=Math.min(lo,d.p);hi=Math.max(hi,d.p);}
    if(showR){lo=Math.min(lo,d.r);hi=Math.max(hi,d.r);}
  }
  if(lo>hi){lo=-1;hi=1;}
  let pad=(hi-lo)*0.15;if(hi-lo<0.05){pad=(0.05-(hi-lo))/2+0.005;}
  lo-=pad;hi+=pad;
  const X=ms=>ml+iw*(1-(now-ms)/winMs);
  const Y=v=>mt+ih*(1-(v-lo)/(hi-lo));
  /* settled shading */
  g.fillStyle=css('--success');g.globalAlpha=0.10;
  let runStart=null;
  for(let i=0;i<=data.length;i++){
    const s=i<data.length&&data[i].settled;
    if(s&&runStart===null)runStart=data[i].ms;
    if(!s&&runStart!==null){g.fillRect(X(runStart),mt,X(data[i-1].ms)-X(runStart),ih);runStart=null;}
  }
  g.globalAlpha=1;
  /* grid + y ticks */
  g.strokeStyle=css('--outline-var');g.fillStyle=css('--on-var');g.lineWidth=1;
  const ticks=5;
  for(let i=0;i<=ticks;i++){
    const v=lo+(hi-lo)*i/ticks,y=Y(v)+.5;
    g.beginPath();g.moveTo(ml,y);g.lineTo(W-mr,y);g.stroke();
    g.fillText(v.toFixed(2)+'°',2,y+3);
  }
  /* x ticks (minutes ago) */
  for(let i=0;i<=5;i++){
    const x=ml+iw*i/5;
    g.fillText('-'+((1-i/5)*winMs/60000).toFixed(0)+'m',x-8,H-5);
  }
  /* temp series (own right-hand scale) */
  if(showT){
    const td=data.filter(d=>d.t!=null);
    if(td.length>1){
      let tlo=Math.min(...td.map(d=>d.t)),thi=Math.max(...td.map(d=>d.t));
      if(thi-tlo<0.5){const c=(thi+tlo)/2;tlo=c-0.25;thi=c+0.25;}
      const TY=v=>mt+ih*(1-(v-tlo)/(thi-tlo));
      g.strokeStyle=css('--warning');g.beginPath();
      td.forEach((d,i)=>i?g.lineTo(X(d.ms),TY(d.t)):g.moveTo(X(d.ms),TY(d.t)));
      g.stroke();
      g.fillStyle=css('--warning');
      g.fillText(thi.toFixed(1),W-mr+2,mt+8);g.fillText(tlo.toFixed(1),W-mr+2,mt+ih);
    }
  }
  /* pitch / roll series */
  const series=[[showP,'p','--primary'],[showR,'r','--info']];
  for(const[en,k,col]of series){
    if(!en)continue;
    g.strokeStyle=css(col);g.lineWidth=1.6;g.beginPath();
    data.forEach((d,i)=>i?g.lineTo(X(d.ms),Y(d[k])):g.moveTo(X(d.ms),Y(d[k])));
    g.stroke();
  }
  /* event markers */
  g.strokeStyle=css('--brand');g.fillStyle=css('--brand');g.lineWidth=1.4;
  for(const d of data)if(d.ev){
    const x=X(d.ms);
    g.beginPath();g.moveTo(x,mt);g.lineTo(x,mt+ih);g.stroke();
    g.beginPath();g.moveTo(x,mt);g.lineTo(x+6,mt+4);g.lineTo(x,mt+8);g.fill();
  }
}
/* ---------- statistics over the visible window (settled only) ---------- */
function drawStats(){
  const now=st?st.up:0;
  const data=buf.filter(b=>b.ms>=now-winMs);
  const sd=data.filter(d=>d.settled);
  const calc=k=>{
    const v=sd.map(d=>d[k]);if(v.length<5)return null;
    const m=v.reduce((a,b)=>a+b)/v.length;
    const sig=Math.sqrt(v.reduce((a,b)=>a+(b-m)*(b-m),0)/v.length);
    return{m,sig,min:Math.min(...v),max:Math.max(...v)};
  };
  const P=calc('p'),R=calc('r');
  const row=(n,s)=>s?'<div class="rh">'+n+'</div><div>'+fmt(s.m,3)+'</div><div>'
    +(s.sig*1000).toFixed(1)+'</div><div>'+((s.max-s.min)*1000).toFixed(1)+'</div><div>'
    +fmt(s.min,3)+'</div><div>'+fmt(s.max,3)+'</div>'
    :'<div class="rh">'+n+'</div><div colspan=5>–</div><div></div><div></div><div></div><div></div>';
  const td=data.filter(d=>d.t!=null);
  const dT=td.length>1?(td[td.length-1].t-td[0].t):null;
  const setPct=data.length?Math.round(100*sd.length/data.length):0;
  $('stats').innerHTML=
    '<div class="h"></div><div class="h">mean°</div><div class="h">σ m°</div><div class="h">p-p m°</div><div class="h">min°</div><div class="h">max°</div>'
    +row('pitch',P)+row('roll',R)
    +'<div class="rh">window</div><div>'+(winMs/60000)+'m</div><div>'+setPct+'%set</div><div>'
    +(dT==null?'–':fmt(dT,2)+'°C')+'</div><div></div><div></div>';
}
function drawEvents(){
  if(!st||!st.evs.length){$('evlist').textContent='';return;}
  $('evlist').innerHTML='Recent events: '+st.evs.slice(0,3).map(e=>{
    const ago=Math.max(0,Math.round((st.up-e[0])/1000));
    const t=ago<120?ago+' s':Math.round(ago/60)+' min';
    return'<b>'+e[1]+'</b> ('+t+' ago)';
  }).join(' · ');
}
/* ---------- legend / window buttons ---------- */
const tgl=(id,get,set)=>{$(id).onclick=()=>{set(!get());
  $(id).classList.toggle('off',!get());drawChart();drawStats();};};
tgl('lgP',()=>showP,v=>showP=v);
tgl('lgR',()=>showR,v=>showR=v);
tgl('lgT',()=>showT,v=>showT=v);
$('win5').onclick=()=>{winMs=5*60000;$('win5').classList.remove('off');$('win15').classList.add('off');drawChart();drawStats();};
$('win15').onclick=()=>{winMs=15*60000;$('win15').classList.remove('off');$('win5').classList.add('off');drawChart();drawStats();};
/* ---------- actions ---------- */
$('markbtn').onclick=async()=>{
  await post('/api/mark',{label:$('marklabel').value});
  $('markbtn').textContent='⚑ MARKED ✓';
  setTimeout(()=>$('markbtn').textContent='⚑ MARK EVENT',900);
};
$('logbtn').onclick=()=>post('/api/log',{action:st&&st.log?'stop':'start'});
$('syncbtn').onclick=()=>post('/api/time',{epoch:Math.floor(Date.now()/1000)});
$('calstart').onclick=()=>post('/api/cal',{action:'start'});
$('calnext').onclick=()=>post('/api/cal',{action:'next'});
$('calcancel').onclick=()=>post('/api/cal',{action:'cancel'});
/* ---------- settings ---------- */
const SKEYS=['gyroTh','sigTh','magTol','dwellMs','avgN','logMs','logMove','moveMs','tol','psign','rsign','swap'];
function fillSettings(s){
  SKEYS.forEach(k=>{const el=$('s_'+k);if(el)el.value=s[k];});
}
function loadSettings(){fetch('/api/settings').then(r=>r.json()).then(fillSettings).catch(()=>{});}
$('setreload').onclick=loadSettings;
$('setapply').onclick=async()=>{
  const o={};SKEYS.forEach(k=>{const el=$('s_'+k);if(el)o[k]=el.value;});
  const r=await post('/api/settings',o);
  fillSettings(await r.json());
  $('setapply').textContent='Applied ✓';
  setTimeout(()=>$('setapply').textContent='Apply',900);
};
$('setdef').onclick=async()=>{
  const r=await post('/api/settings',{reset:1});
  fillSettings(await r.json());
};
loadSettings();
poll();
window.addEventListener('resize',drawChart);
</script>
</body>
</html>
)rawliteral";
