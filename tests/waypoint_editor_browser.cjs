// Uses installed Chrome and Node's built-in WebSocket; no npm downloads.
const {spawn,spawnSync}=require('node:child_process');
const fs=require('node:fs');
const assert=require('node:assert/strict');
const path=require('node:path');
const os=require('node:os');
const artifacts=process.env.B2W_EDITOR_TEST_ARTIFACTS||fs.mkdtempSync(path.join(os.tmpdir(),'b2w-editor-browser-'));
fs.mkdirSync(artifacts,{recursive:true});
process.chdir(path.resolve(__dirname,'..'));
const profile=fs.mkdtempSync(path.join(artifacts,'chrome-'));
const delay=ms=>new Promise(r=>setTimeout(r,ms));
(async()=>{
 const backup=process.env.B2W_EDITOR_V1_BACKUP,canonicalMode=process.env.B2W_EDITOR_CANONICAL==='1';
 let generated;
 if(!backup&&!canonicalMode){
  // Portable imported V1 fixture, with a rotated origin and existing annotations.
  const fixture=spawnSync('/usr/bin/python3',['-c',`
import sys, pathlib, copy, yaml, hashlib
sys.path.insert(0,'scripts')
from PIL import Image
from waypoint_editor.core import Editor,ROOT
p=pathlib.Path(sys.argv[1]);p.mkdir(parents=True,exist_ok=True)
Image.new('L',(800,800),254).save(p/'map.pgm')
(p/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-20,-20,.3],negate=0,occupied_thresh=.65,free_thresh=.25)))
e=Editor(p/'map.yaml',ROOT/'course_navigation/config/nav2.yaml',ROOT/'course_navigation/config/warehouse_route.yaml')
s=copy.deepcopy(e.initial);s['goals'][0]['position'][0]+=.3;s['goals'][0]['yaw_rad']=-1.234567891234
s['markers']={'entrance':[1.,2.,.5],'exit':[3.,4.,-.5]}
s['polygons']=[dict(kind='corridor',points=[[-10,-10],[-8,-10],[-8,-8],[-10,-8]])]
f=e.export(s,p/'exports')
for name in ['candidate.yaml','manifest.yaml']:
 d=yaml.safe_load((f/name).read_text())
 for k in ['xy_tolerance','configuration_identity','import_history']:d.pop(k,None)
 if name=='candidate.yaml':d['candidate_schema_version']=1
 else:d.pop('candidate_schema_version',None)
 (f/name).write_text(yaml.safe_dump(d,sort_keys=False))
m=yaml.safe_load((f/'manifest.yaml').read_text());m['outputs']={n:hashlib.sha256((f/n).read_bytes()).hexdigest() for n in m['outputs']};(f/'manifest.yaml').write_text(yaml.safe_dump(m))
print(f/'candidate.yaml')
`,path.join(artifacts,'fixture')],{encoding:'utf8'});
  assert.equal(fixture.status,0,fixture.stderr);generated=fixture.stdout.trim();
 }
 const extra=backup?['--map',path.join(backup,'warehouse.yaml'),'--candidate',path.join(backup,'candidate-16nlistv/candidate.yaml')]:canonicalMode?[]:['--map',path.join(artifacts,'fixture/map.yaml'),'--candidate',generated];
 const editor=spawn('/usr/bin/python3',['scripts/edit_warehouse_waypoints.py','--port','0','--output',path.join(artifacts,'exports'),...extra]);
 const chrome=spawn('/usr/bin/google-chrome',['--headless','--no-sandbox','--disable-gpu','--disable-background-networking','--no-proxy-server','--disable-extensions','--no-first-run','--remote-debugging-port=0',`--user-data-dir=${profile}`,'about:blank'],{stdio:'ignore'});
 let editorPort;editor.stdout.on('data',d=>{const match=String(d).match(/127\.0\.0\.1:(\d+)/);if(match)editorPort=Number(match[1]);});editor.stderr.on('data',d=>process.stderr.write(d));
 let ws;
 try{
  let tabs;for(let i=0;i<100;i++){try{const browserPort=Number(fs.readFileSync(path.join(profile,'DevToolsActivePort'),'utf8').split('\n')[0]);if(!editorPort)throw Error('Editor not ready');const created=await(await fetch(`http://127.0.0.1:${browserPort}/json/new?http://127.0.0.1:${editorPort}/`,{method:'PUT'})).json();const candidate=[created];await fetch(`http://127.0.0.1:${editorPort}/bootstrap`);tabs=candidate;break;}catch{await delay(100);}}
  assert.ok(tabs);ws=new WebSocket(tabs[0].webSocketDebuggerUrl);await new Promise(r=>ws.addEventListener('open',r,{once:true}));
  let id=0;const pending=new Map(),errors=[];
  ws.addEventListener('message',e=>{const m=JSON.parse(e.data);if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails);if(m.id){const [ok,no]=pending.get(m.id);pending.delete(m.id);m.error?no(m.error):ok(m.result);}});
  const send=(method,params={})=>new Promise((ok,no)=>{pending.set(++id,[ok,no]);ws.send(JSON.stringify({id,method,params}));});
  const evaluate=async(expression)=>{const r=await send('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;};
  await send('Runtime.enable');await send('Page.enable');await send('Emulation.setDeviceMetricsOverride',{width:1400,height:1000,deviceScaleFactor:1,mobile:false});

  for(let i=0;i<100;i++){if(await evaluate('document.getElementById("status")?.textContent.includes("Ready.")'))break;await delay(100);}
  assert.ok(await evaluate('document.getElementById("status")?.textContent.includes("Ready.")'),JSON.stringify(await evaluate('({url:location.href,body:document.body?.innerText})')));
  const click=async(x,y)=>{await send('Input.dispatchMouseEvent',{type:'mousePressed',x,y,button:'left',clickCount:1});await send('Input.dispatchMouseEvent',{type:'mouseReleased',x,y,button:'left',clickCount:1});};
  const drag=async(a,b)=>{await send('Input.dispatchMouseEvent',{type:'mousePressed',x:a[0],y:a[1],button:'left',clickCount:1});await send('Input.dispatchMouseEvent',{type:'mouseMoved',x:b[0],y:b[1],button:'left',buttons:1});await send('Input.dispatchMouseEvent',{type:'mouseReleased',x:b[0],y:b[1],button:'left',clickCount:1});await delay(150);};
  const bootstrap=await(await fetch(`http://127.0.0.1:${editorPort}/bootstrap`)).json();
  const start=bootstrap.initial.goals[0].position;
  if(backup){
   assert.deepEqual(start,[0.039514056696976496,6.779728935566027]);
   assert.equal(await evaluate('Number(document.getElementById("yaw").value)'),-3.1012105780252917);
   assert.notDeepEqual(bootstrap.canonical.goals,bootstrap.initial.goals);
   assert.equal(await evaluate('document.getElementById("reset").textContent'),'Reset imported candidate');
  }
  const rect=await evaluate(`(()=>{const r=document.getElementById('map').getBoundingClientRect();return {left:r.left,top:r.top,width:r.width,height:r.height}})()`);
  let expectedZoom=Math.min(rect.width/800,rect.height/800)*.94;
  let expectedOffset=[(rect.width-800*expectedZoom)/2,(rect.height-800*expectedZoom)/2];
  const projected=([x,y])=>{
   const [ox,oy,a]=bootstrap.map.origin,dx=x-ox,dy=y-oy;
   return [(Math.cos(a)*dx+Math.sin(a)*dy)/.05*expectedZoom+expectedOffset[0],(800-(-Math.sin(a)*dx+Math.cos(a)*dy)/.05)*expectedZoom+expectedOffset[1]];
  };
  const xy=projected(start).map((v,i)=>v+[rect.left,rect.top][i]);
  // Observe calls while forwarding to the real Canvas renderer.
  await evaluate(`window.discs=[];{const c=document.getElementById('map').getContext('2d'),arc=c.arc.bind(c),dash=c.setLineDash.bind(c);let last;c.arc=(...a)=>{last=a;return arc(...a)};c.setLineDash=a=>{if(a.length)window.discs.push(last.slice(0,3));return dash(a)}}`);
  const recordDiscs=()=>evaluate(`window.discs=[];document.getElementById('xy-circles').dispatchEvent(new Event('change'));window.discs`);
  let discs=await recordDiscs();assert.equal(discs.length,5);
  const checkDiscs=async()=>{
   const current=await evaluate('JSON.parse(document.getElementById("values").textContent).candidate');
   const observed=await recordDiscs();assert.equal(observed.length,5);
   observed.forEach((d,i)=>{
    const want=[...projected(current[i].position),5*expectedZoom];
    d.forEach((v,j)=>assert.ok(Math.abs(v-want[j])<1e-8,`${v} != ${want[j]}`));
   });
  };
  await checkDiscs();
  await evaluate(`document.getElementById('xy-circles').checked=false`);assert.deepEqual(await recordDiscs(),[]);
  await evaluate(`document.getElementById('xy-circles').checked=true`);
  for(const id of ['original-footprints','candidate-footprints']){
   const before=await evaluate(`document.getElementById('map').toDataURL()`);
   await evaluate(`document.getElementById('${id}').checked=false;document.getElementById('${id}').dispatchEvent(new Event('change'))`);
   assert.notEqual(await evaluate(`document.getElementById('map').toDataURL()`),before);
   await evaluate(`document.getElementById('${id}').checked=true;document.getElementById('${id}').dispatchEvent(new Event('change'))`);
  }
  const original=await evaluate('Number(document.getElementById("x").value)');
  await drag(xy,[xy[0]+25,xy[1]+10]);
  assert.notEqual(await evaluate('Number(document.getElementById("x").value)'),original);await checkDiscs();
  await evaluate('document.getElementById("undo").click()');assert.equal(await evaluate('Number(document.getElementById("x").value)'),original);
  await evaluate('document.getElementById("redo").click()');assert.notEqual(await evaluate('Number(document.getElementById("x").value)'),original);
  await evaluate(`document.getElementById('x').value='0.123456789012345';document.getElementById('apply').click()`);
  assert.equal(await evaluate('Number(document.getElementById("x").value)'),.123456789012345);await checkDiscs();
  // Drag heading arrow to a different angle using the current displayed pose.
  const currentPose=await evaluate(`['x','y','yaw'].map(id=>Number(document.getElementById(id).value))`);
  const arrow=projected([currentPose[0]+.95*Math.cos(currentPose[2]),currentPose[1]+.95*Math.sin(currentPose[2])]).map((v,i)=>v+[rect.left,rect.top][i]);
  const oldYaw=await evaluate('Number(document.getElementById("yaw").value)');await drag(arrow,[arrow[0]+20,arrow[1]-30]);assert.notEqual(await evaluate('Number(document.getElementById("yaw").value)'),oldYaw);
  const edited=await evaluate('JSON.parse(document.getElementById("values").textContent).candidate[0]');
  if(edited.quaternion)assert.ok(Math.abs(edited.quaternion.z-Math.sin(edited.yaw_rad/2))<1e-14);
  await evaluate(`document.getElementById('mode').value='corridor';document.getElementById('mode').dispatchEvent(new Event('change'))`);
  for(const p of [[200,300],[350,300],[350,420],[200,420]])await click(...p);
  await evaluate('document.getElementById("finish").click()');await delay(200);
  await evaluate(`document.getElementById('mode').value='edit';document.getElementById('mode').dispatchEvent(new Event('change'))`);
  await drag([200,300],[185,285]);
  await evaluate(`document.getElementById('mode').value='entrance';document.getElementById('mode').dispatchEvent(new Event('change'))`);await click(300,400);
  assert.ok(await evaluate('document.getElementById("selected").textContent.includes("entrance")'));
  await evaluate(`document.getElementById('mode').value='exit';document.getElementById('mode').dispatchEvent(new Event('change'))`);await click(350,450);
  await evaluate(`document.getElementById('sweep').checked=true;document.getElementById('sweep').dispatchEvent(new Event('change'));document.getElementById('check').click()`);
  for(let i=0;i<100;i++){if(await evaluate('document.getElementById("checks").textContent.includes("sample_count")'))break;await delay(100);}
  assert.ok(await evaluate('document.getElementById("checks").textContent.includes("sample_count")'));
  await evaluate('document.getElementById("export").click()');
  for(let i=0;i<100;i++){if(await evaluate('document.getElementById("status").textContent.startsWith("Exported to")'))break;await delay(100);}
  const status=await evaluate('document.getElementById("status").textContent');assert.ok(status.startsWith('Exported to'),status);
  await evaluate('document.getElementById("export").click()');await delay(300);
  const secondStatus=await evaluate('document.getElementById("status").textContent');assert.ok(secondStatus.startsWith('Exported to'));assert.notEqual(secondStatus,status);
  const screenshot=await send('Page.captureScreenshot',{format:'png'});fs.writeFileSync(path.join(artifacts,'browser.png'),Buffer.from(screenshot.data,'base64'));
  const cursorBefore=await evaluate('document.getElementById("cursor").textContent');
  await send('Input.dispatchMouseEvent',{type:'mouseWheel',x:500,y:500,deltaX:0,deltaY:-250});
  await delay(100);
  const cursor=[500-rect.left,500-rect.top],next=expectedZoom*Math.exp(.25);
  expectedOffset=cursor.map((v,i)=>v-(v-expectedOffset[i])*next/expectedZoom);expectedZoom=next;
  await checkDiscs();
  await evaluate(`document.getElementById('mode').value='pan'`);
  await drag([100,800],[130,820]);expectedOffset=expectedOffset.map((v,i)=>v+[30,20][i]);await checkDiscs();
  await send('Emulation.setDeviceMetricsOverride',{width:1250,height:900,deviceScaleFactor:2,mobile:false});await delay(100);await checkDiscs();
  await evaluate(`document.getElementById('mode').value='restricted';document.getElementById('mode').dispatchEvent(new Event('change'))`);
  for(const p of [[100,600],[200,600],[150,700]])await click(...p);
  await evaluate('document.getElementById("finish").click()');await delay(200);
  await evaluate('document.getElementById("reset").click();document.getElementById("selected").value="G1";document.getElementById("selected").dispatchEvent(new Event("change"))');assert.equal(await evaluate('Number(document.getElementById("x").value)'),original);
  assert.deepEqual(await evaluate('JSON.parse(document.getElementById("values").textContent).candidate'),bootstrap.initial.goals);
  assert.equal(await evaluate('document.getElementById("selected").textContent.includes("entrance")'),Object.hasOwn(bootstrap.initial.markers,'entrance'));
  assert.deepEqual(await evaluate('JSON.parse(document.getElementById("values").textContent).markers'),bootstrap.initial.markers);
  assert.deepEqual(await evaluate('JSON.parse(document.getElementById("values").textContent).polygons'),bootstrap.initial.polygons);
  await evaluate('document.getElementById("undo").click()');assert.equal(await evaluate('document.getElementById("selected").textContent.includes("entrance")'),true);
  assert.notEqual(await evaluate('document.getElementById("cursor").textContent'),cursorBefore);
  assert.deepEqual(errors,[]);console.log('Real Chrome PASS: V1/canonical load, metric discs/toggles/numeric edit/zoom/pan/resize, unique exports, waypoint drag, arrow yaw drag, undo/redo, polygon drawing/vertex edit, entrance/exit, sweep/check and full export.');console.log(status);console.log(`Browser artifacts: ${artifacts}`);
 }finally{if(ws)ws.close();editor.kill('SIGINT');chrome.kill('SIGTERM');}
})().catch(e=>{console.error(e);process.exitCode=1;});
