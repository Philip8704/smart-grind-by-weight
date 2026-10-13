#pragma once

// The page served at / while WiFi is on. Self-contained: no external scripts, fonts or
// styles, since the grinder may be on a network without internet access.
static const char WIFI_WEB_PAGE[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Grind by Weight</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--text:#1d2330;--muted:#677084;--line:#dde1e8;--accent:#c62828;--ok:#2e7d32;--warn:#b26a00;--mono:#0f141c;--monotext:#cfd6e3}
@media (prefers-color-scheme:dark){:root{--bg:#0e1116;--card:#171b22;--text:#e6e9ef;--muted:#949cad;--line:#2a303b;--accent:#ef5350;--ok:#66bb6a;--warn:#ffb74d}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
main{max-width:860px;margin:0 auto;padding:16px}
header{display:flex;flex-wrap:wrap;align-items:baseline;gap:8px 16px;margin:8px 0 16px}
h1{font-size:22px;margin:0}#info{color:var(--muted);font-size:13px}
section{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:16px;margin-bottom:14px}
h2{font-size:16px;margin:0 0 10px}p{margin:0 0 10px;color:var(--muted);font-size:13px}
button,.btn{background:var(--accent);color:#fff;border:0;border-radius:7px;padding:8px 14px;font:inherit;cursor:pointer;text-decoration:none;display:inline-block}
button.alt,.btn.alt{background:transparent;color:var(--text);border:1px solid var(--line)}
button:disabled{opacity:.5;cursor:default}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
input[type=file]{max-width:100%;color:var(--muted)}
.bar{height:8px;background:var(--line);border-radius:4px;overflow:hidden;margin-top:10px;display:none}.bar div{height:100%;width:0;background:var(--accent)}
.msg{margin-top:8px;font-size:13px;min-height:1em}.msg.ok{color:var(--ok)}.msg.err{color:var(--accent)}.msg.warn{color:var(--warn)}
pre{background:var(--mono);color:var(--monotext);border-radius:8px;padding:10px;margin:10px 0 0;height:320px;overflow:auto;font:12px/1.4 ui-monospace,Consolas,monospace;white-space:pre-wrap;word-break:break-all}
ul{list-style:none;padding:0;margin:0}li{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid var(--line);font-size:13px}li a{color:var(--accent)}
</style></head><body><main>
<header><h1>Grind by Weight</h1><span id="info">Connecting...</span></header>

<section><h2>Firmware update</h2>
<p>Choose the <b>-web-ota.bin</b> file from the web flasher's firmware folder - the same file Bluetooth updates use. Not possible while grinding.</p>
<div class="row"><input type="file" id="fw" accept=".bin"><button id="up" onclick="upload()">Upload</button></div>
<div class="bar" id="bar"><div id="barfill"></div></div><div class="msg" id="upmsg"></div></section>

<section><h2>Diagnostic report</h2>
<div class="row"><a class="btn" href="/report" target="_blank">Open</a><a class="btn alt" href="/report?download=1">Download</a></div></section>

<section><h2>Live log</h2>
<p>Every log line as it is written. Also on TCP port 23 (telnet, nc, PuTTY raw).</p>
<div class="row"><button id="lv" onclick="toggleLive()">Start</button><button class="alt" onclick="logbox.textContent=''">Clear</button>
<label style="font-size:13px;color:var(--muted)"><input type="checkbox" id="follow" checked> Follow</label></div>
<pre id="log"></pre></section>

<section><h2>Saved logs</h2><p>Kept across restarts, oldest first.</p><ul id="logs"><li>Loading...</li></ul></section>
</main>
<script>
const $=id=>document.getElementById(id),logbox=$('log');let live=false,pos=null,timer=null,build=null;
function fmt(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h '+m+'m'}
async function status(){try{const r=await fetch('/status',{cache:'no-store'});const j=await r.json();build=j.build;
$('info').textContent='Build #'+j.build+' ('+j.git+') | up '+fmt(j.uptime_s)+' | '+j.ip+' '+j.rssi+' dBm'+(j.grinding?' | GRINDING':'');return j}catch(e){$('info').textContent='Not reachable';return null}}
function msg(t,c){const m=$('upmsg');m.textContent=t;m.className='msg '+(c||'')}
function upload(){const f=$('fw').files[0];if(!f){msg('Choose a file first','err');return}
if(!/web-ota/.test(f.name)&&!confirm(f.name+' is not a -web-ota.bin file. Upload anyway?'))return;
const x=new XMLHttpRequest(),before=build;$('up').disabled=true;$('bar').style.display='block';msg('Uploading...');
x.upload.onprogress=e=>{if(e.lengthComputable){const p=Math.round(e.loaded/e.total*100);$('barfill').style.width=p+'%';msg('Uploading '+p+'%')}};
x.onload=()=>{if(x.status==200){msg('Received - applying the update, the grinder restarts...','warn');waitReboot(before)}else{msg(x.responseText||('Failed ('+x.status+')'),'err');$('up').disabled=false}};
x.onerror=()=>{msg('Upload interrupted','err');$('up').disabled=false};
x.open('POST','/update');x.setRequestHeader('Content-Type','application/octet-stream');x.send(f)}
function waitReboot(before){const t0=Date.now();let gone=false;const iv=setInterval(async()=>{const j=await status();
if(!j){gone=true}else if(gone||j.build!=before){clearInterval(iv);$('up').disabled=false;
msg(j.build!=before?'Updated to build #'+j.build:'Back on build #'+j.build+' - the update did not apply',j.build!=before?'ok':'err');loadLogs()}
if(Date.now()-t0>180000){clearInterval(iv);$('up').disabled=false;msg('No answer after 3 minutes - check the grinder screen','err')}},2500)}
function toggleLive(){live=!live;$('lv').textContent=live?'Stop':'Start';if(live)poll();else clearTimeout(timer)}
async function poll(){if(!live)return;try{const r=await fetch('/live'+(pos===null?'':'?from='+pos),{cache:'no-store'});
pos=r.headers.get('X-Next');const t=await r.text();if(t){logbox.textContent+=t;if(logbox.textContent.length>200000)logbox.textContent=logbox.textContent.slice(-150000);
if($('follow').checked)logbox.scrollTop=logbox.scrollHeight}}catch(e){}timer=setTimeout(poll,700)}
async function loadLogs(){try{const r=await fetch('/logs',{cache:'no-store'});const l=await r.json();
$('logs').innerHTML=l.length?l.map(f=>'<li><a href="/logs/get?name='+encodeURIComponent(f.name)+'" download>'+f.name+'</a><span>'+(f.size/1024).toFixed(1)+' KB</span></li>').join(''):'<li>No saved logs</li>'}catch(e){$('logs').innerHTML='<li>Could not list logs</li>'}}
status();loadLogs();setInterval(()=>{if(!$('up').disabled)status()},10000);
</script></body></html>)HTML";
