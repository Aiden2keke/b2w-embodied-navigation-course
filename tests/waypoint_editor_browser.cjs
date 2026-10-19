// Uses installed Chrome and Node's built-in WebSocket; no npm downloads.
const {spawn}=require('node:child_process');
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
 const editor=spawn('/usr/bin/python3',['scripts/edit_warehouse_waypoints.py','--port','0','--output',path.join(artifacts,'exports')]);
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
  const xy=await evaluate(`(()=>{const c=document.getElementById('map'),r=c.getBoundingClientRect(),s=Math.min(r.width/800,r.height/800)*.94,ox=(r.width-800*s)/2,oy=(r.height-800*s)/2;return [r.left+ox+((-0.18571805953979492+20)/.05)*s,r.top+oy+((20-7.189985752105713)/.05)*s]})()`);
  const original=await evaluate('Number(document.getElementById("x").value)');
  await drag(xy,[xy[0]+25,xy[1]+10]);
  assert.notEqual(await evaluate('Number(document.getElementById("x").value)'),original);
  await evaluate('document.getElementById("undo").click()');assert.equal(await evaluate('Number(document.getElementById("x").value)'),original);
  await evaluate('document.getElementById("redo").click()');assert.notEqual(await evaluate('Number(document.getElementById("x").value)'),original);
  // Drag heading arrow to a different angle using the current displayed pose.
  const arrow=await evaluate(`(()=>{const c=document.getElementById('map'),r=c.getBoundingClientRect(),s=Math.min(r.width/800,r.height/800)*.94,ox=(r.width-800*s)/2,oy=(r.height-800*s)/2;const x=Number(document.getElementById('x').value),y=Number(document.getElementById('y').value),a=Number(document.getElementById('yaw').value);return [r.left+ox+((x+.95*Math.cos(a)+20)/.05)*s,r.top+oy+((20-y-.95*Math.sin(a))/.05)*s]})()`);
  const oldYaw=await evaluate('Number(document.getElementById("yaw").value)');await drag(arrow,[arrow[0]+20,arrow[1]-30]);assert.notEqual(await evaluate('Number(document.getElementById("yaw").value)'),oldYaw);
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
  const screenshot=await send('Page.captureScreenshot',{format:'png'});fs.writeFileSync(path.join(artifacts,'browser.png'),Buffer.from(screenshot.data,'base64'));
  const cursorBefore=await evaluate('document.getElementById("cursor").textContent');
  await send('Input.dispatchMouseEvent',{type:'mouseWheel',x:500,y:500,deltaX:0,deltaY:-250});
  await drag([100,800],[130,820]);
  await evaluate(`document.getElementById('mode').value='restricted';document.getElementById('mode').dispatchEvent(new Event('change'))`);
  for(const p of [[100,600],[200,600],[150,700]])await click(...p);
  await evaluate('document.getElementById("finish").click()');await delay(200);
  await evaluate('document.getElementById("reset").click()');assert.equal(await evaluate('Number(document.getElementById("x").value)'),original);
  assert.equal(await evaluate('document.getElementById("selected").textContent.includes("entrance")'),false);
  await evaluate('document.getElementById("undo").click()');assert.equal(await evaluate('document.getElementById("selected").textContent.includes("entrance")'),true);
  assert.notEqual(await evaluate('document.getElementById("cursor").textContent'),cursorBefore);
  assert.deepEqual(errors,[]);console.log('Real Chrome PASS: map load, waypoint drag, arrow yaw drag, undo/redo, polygon drawing/vertex edit, entrance/exit, sweep/check and full export.');console.log(status);console.log(`Browser artifacts: ${artifacts}`);
 }finally{if(ws)ws.close();editor.kill('SIGINT');chrome.kill('SIGTERM');}
})().catch(e=>{console.error(e);process.exitCode=1;});
