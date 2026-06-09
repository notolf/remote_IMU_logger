// =============================================================================
//  web_page.h  —  Single self-contained live web view (no external CDN).
//  Served at "/" by the firmware. Talks to the device over a WebSocket at "/ws".
//  Inline CSS + vanilla JS only, so it works even with no internet (soft-AP mode).
// =============================================================================
#pragma once
#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CoreS3 Static Level Logger</title>
<style>
  :root{--bg:#0d1117;--card:#161b22;--line:#30363d;--fg:#e6edf3;--mut:#8b949e;
        --ok:#3fb950;--warn:#d29922;--bad:#f85149;--accent:#58a6ff;}
  *{box-sizing:border-box}
  body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;
       background:var(--bg);color:var(--fg);}
  header{display:flex;align-items:center;justify-content:space-between;
         padding:10px 16px;border-bottom:1px solid var(--line);background:var(--card);
         position:sticky;top:0;z-index:5;}
  header h1{font-size:16px;margin:0;font-weight:600;}
  #conn{display:flex;align-items:center;gap:8px;font-size:13px;color:var(--mut);}
  #dot{width:11px;height:11px;border-radius:50%;background:var(--bad);
       box-shadow:0 0 6px currentColor;color:var(--bad);}
  #dot.up{background:var(--ok);color:var(--ok);}
  main{max-width:760px;margin:0 auto;padding:16px;display:grid;gap:16px;}
  .card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px;}
  .angles{display:grid;grid-template-columns:1fr 1fr;gap:16px;text-align:center;}
  .angle .lbl{font-size:13px;color:var(--mut);letter-spacing:.08em;}
  .angle .val{font-size:54px;font-weight:700;font-variant-numeric:tabular-nums;line-height:1.1;}
  .angle .unit{font-size:22px;color:var(--mut);}
  .state-row{display:flex;flex-wrap:wrap;gap:8px;margin-top:14px;justify-content:center;}
  .pill{padding:5px 12px;border-radius:999px;font-size:13px;font-weight:600;
        border:1px solid var(--line);background:#0d1117;color:var(--mut);}
  .pill.ok{color:var(--ok);border-color:var(--ok);}
  .pill.warn{color:var(--warn);border-color:var(--warn);}
  .pill.bad{color:var(--bad);border-color:var(--bad);}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;}
  .kv{background:#0d1117;border:1px solid var(--line);border-radius:8px;padding:10px 12px;}
  .kv .k{font-size:11px;color:var(--mut);text-transform:uppercase;letter-spacing:.05em;}
  .kv .v{font-size:16px;font-weight:600;margin-top:2px;word-break:break-all;}
  h2{font-size:13px;color:var(--mut);text-transform:uppercase;letter-spacing:.06em;margin:0 0 10px;}
  button{font:inherit;font-weight:600;border:1px solid var(--line);background:#21262d;color:var(--fg);
         padding:11px 14px;border-radius:8px;cursor:pointer;}
  button:hover{border-color:var(--accent);}
  button:active{transform:translateY(1px);}
  button.primary{background:var(--accent);border-color:var(--accent);color:#0d1117;}
  button.danger{background:#3d1418;border-color:var(--bad);color:#ffb4ad;}
  button:disabled{opacity:.45;cursor:not-allowed;}
  .controls{display:flex;flex-wrap:wrap;gap:10px;}
  .event-row{display:flex;gap:10px;margin-top:10px;}
  input[type=text]{flex:1;font:inherit;background:#0d1117;border:1px solid var(--line);
                   color:var(--fg);border-radius:8px;padding:11px 12px;}
  #calCard{display:none;border-color:var(--accent);}
  #calCard.show{display:block;}
  #calPrompt{font-size:15px;line-height:1.5;margin-bottom:12px;}
  .prog{height:10px;background:#0d1117;border-radius:6px;overflow:hidden;border:1px solid var(--line);}
  .prog>div{height:100%;width:0;background:var(--accent);transition:width .15s;}
  footer{color:var(--mut);font-size:12px;text-align:center;padding:6px 0 22px;}
  .muted{color:var(--mut);font-size:12px;margin-top:8px;}
</style>
</head>
<body>
<header>
  <h1>CoreS3 Static Level Logger</h1>
  <div id="conn"><span id="dot"></span><span id="connTxt">connecting...</span></div>
</header>

<main>
  <section class="card">
    <div class="angles">
      <div class="angle"><div class="lbl">PITCH</div>
        <div><span class="val" id="pitch">--.---</span><span class="unit">&deg;</span></div></div>
      <div class="angle"><div class="lbl">ROLL</div>
        <div><span class="val" id="roll">--.---</span><span class="unit">&deg;</span></div></div>
    </div>
    <div class="state-row">
      <span class="pill" id="pState">STATE</span>
      <span class="pill" id="pCal">CALIBRATION</span>
      <span class="pill" id="pLog">LOGGING</span>
    </div>
  </section>

  <!-- Calibration wizard (only visible while a calibration is in progress) -->
  <section class="card" id="calCard">
    <h2>Granite-plate calibration</h2>
    <div id="calPrompt"></div>
    <div class="prog"><div id="calBar"></div></div>
    <div class="controls" style="margin-top:14px;">
      <button class="primary" id="btnCalNext">Next &#9654;</button>
      <button class="danger" id="btnCalCancel">Cancel</button>
    </div>
  </section>

  <section class="card">
    <h2>Live status</h2>
    <div class="grid">
      <div class="kv"><div class="k">Battery</div><div class="v" id="bat">--</div></div>
      <div class="kv"><div class="k">IMU temp</div><div class="v" id="temp">--</div></div>
      <div class="kv"><div class="k">Events</div><div class="v" id="ev">0</div></div>
      <div class="kv"><div class="k">WiFi</div><div class="v" id="wifi">--</div></div>
      <div class="kv"><div class="k">SD card</div><div class="v" id="sd">--</div></div>
      <div class="kv"><div class="k">Log file</div><div class="v" id="file">--</div></div>
      <div class="kv"><div class="k">Clock</div><div class="v" id="clock">--</div></div>
      <div class="kv"><div class="k">accel (g)</div><div class="v" id="acc">--</div></div>
    </div>
  </section>

  <section class="card">
    <h2>Mark event</h2>
    <div class="controls"><button class="primary" id="btnMark">&#9873; Mark Event</button></div>
    <div class="event-row">
      <input type="text" id="evLabel" placeholder="optional label (e.g. waypoint 3)" maxlength="60">
    </div>
    <div class="muted">Writes an immediate event row to the CSV, even while moving.</div>
  </section>

  <section class="card">
    <h2>Controls</h2>
    <div class="controls">
      <button id="btnLog">Start / Stop Logging</button>
      <button id="btnCal">Calibrate</button>
      <button class="danger" id="btnClear">Clear Calibration</button>
    </div>
    <div class="muted">The physical 180&deg; flip happens at the device; this page shows the
      current step and lets you advance it.</div>
  </section>

  <footer>Local microSD is the source of truth &mdash; this view is a live mirror.</footer>
</main>

<script>
(function(){
  "use strict";
  var ws=null, retry=0;
  var $=function(id){return document.getElementById(id);};

  function setPill(el,txt,cls){el.textContent=txt;el.className="pill "+(cls||"");}

  function setConn(up){
    $("dot").className=up?"up":"";
    $("connTxt").textContent=up?"connected":"reconnecting...";
    var dis=!up;
    ["btnMark","btnLog","btnCal","btnClear","btnCalNext","btnCalCancel"]
      .forEach(function(i){$(i).disabled=dis;});
  }

  function send(obj){ if(ws&&ws.readyState===1){ ws.send(JSON.stringify(obj)); } }

  function fmt(v,d){ return (v===null||v===undefined||isNaN(v))?"--":Number(v).toFixed(d); }

  function onMsg(m){
    var s; try{ s=JSON.parse(m.data); }catch(e){ return; }
    if(s.type!=="status") return;

    $("pitch").textContent=fmt(s.pitch,3);
    $("roll").textContent =fmt(s.roll,3);

    if(s.settled){ setPill($("pState"),"SETTLED","ok"); }
    else         { setPill($("pState"),"MOVING","warn"); }

    if(s.calibrated){ setPill($("pCal"),"CALIBRATED","ok"); }
    else            { setPill($("pCal"),"UNCALIBRATED","bad"); }

    if(s.logging){ setPill($("pLog"),"LOGGING","ok"); $("btnLog").textContent="Stop Logging"; }
    else         { setPill($("pLog"),"NOT LOGGING",""); $("btnLog").textContent="Start Logging"; }

    $("bat").textContent  = (s.battery<0?"--":s.battery+" %") + (s.charging?" ⚡":"");
    $("temp").textContent = fmt(s.imu_temp,1)+" °C";
    $("ev").textContent   = s.events;
    $("wifi").textContent = s.wifi;
    $("sd").textContent   = s.sd;
    $("file").textContent = s.file||"--";
    $("clock").textContent= s.clock||"--";
    $("acc").textContent  = fmt(s.ax,3)+", "+fmt(s.ay,3)+", "+fmt(s.az,3);

    // Calibration wizard visibility + prompt
    var card=$("calCard");
    if(s.calStep && s.calStep!=="idle"){
      card.classList.add("show");
      var pr=$("calPrompt");
      pr.textContent=s.calPrompt||"";
      pr.style.color=(s.calStep==="done")?(s.calError?"var(--bad)":"var(--ok)"):"var(--fg)";
      $("calBar").style.width=(s.calProgress||0)+"%";
      var capturing=(s.calStep==="capture_a"||s.calStep==="capture_b");
      $("btnCalNext").disabled=capturing;
      $("btnCalNext").textContent=(s.calStep==="done")?"Finish":"Next ▶";
    } else {
      card.classList.remove("show");
    }
  }

  function connect(){
    ws=new WebSocket("ws://"+location.host+"/ws");
    ws.onopen   =function(){ retry=0; setConn(true); };
    ws.onmessage=onMsg;
    ws.onclose  =function(){ setConn(false); retry=Math.min(retry+1,10);
                             setTimeout(connect, 400*retry); };
    ws.onerror  =function(){ try{ws.close();}catch(e){} };
  }

  // --- button wiring ---
  $("btnMark").onclick    =function(){ send({cmd:"mark_event",label:$("evLabel").value||""});
                                       $("evLabel").value=""; };
  $("btnLog").onclick      =function(){ send({cmd:"toggle_log"}); };
  $("btnCal").onclick      =function(){ send({cmd:"calibrate"}); };
  $("btnClear").onclick    =function(){ if(confirm("Clear stored calibration offsets?"))
                                          send({cmd:"clear_calib"}); };
  $("btnCalNext").onclick  =function(){ send({cmd:"calib_next"}); };
  $("btnCalCancel").onclick=function(){ send({cmd:"calib_cancel"}); };

  setConn(false);
  connect();
})();
</script>
</body>
</html>
)HTMLPAGE";
