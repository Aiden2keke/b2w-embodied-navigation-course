import {pixelToMap,mapToPixel,transform,clone,History,moveVertex} from './model.mjs';
const $=id=>document.getElementById(id), canvas=$('map'),ctx=canvas.getContext('2d');
let data,state,history,image,zoom=1,offset=[0,0],selection='G1',polygonSelection=null,draft=[],drag=null,sweepResults=null;
const costmapImages={};
const status=message=>$('status').textContent=message;
function pose(name){const g=state.goals.find(g=>g.name===name);return g?[...g.position,g.yaw_rad]:state.markers[name];}
function setPose(name,p){const g=state.goals.find(g=>g.name===name);if(g){g.position=p.slice(0,2);g.yaw_rad=p[2];}else state.markers[name]=p;}
function screen(xy){const p=mapToPixel(data.map,...xy);return [(p[0]+.5)*zoom+offset[0],(p[1]+.5)*zoom+offset[1]];}
function world(p){return pixelToMap(data.map,(p[0]-offset[0])/zoom-.5,(p[1]-offset[1])/zoom-.5);}
function local(e){const r=canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];}
function distance(a,b){return Math.hypot(a[0]-b[0],a[1]-b[1]);}
function line(points,color,width=1,closed=false){if(!points.length)return;ctx.beginPath();points.forEach((p,i)=>{const q=screen(p);i?ctx.lineTo(...q):ctx.moveTo(...q);});if(closed)ctx.closePath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();}
function dot(xy,color,r=5){ctx.beginPath();ctx.arc(...screen(xy),r,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();}
function arrowTip(p){return [p[0]+Math.cos(p[2])*.95,p[1]+Math.sin(p[2])*.95];}
function drawPose(name,p,color,original=false){
 line(transform(data.footprint,p),color,original?1:2,true);line([p.slice(0,2),arrowTip(p)],color,2);
 dot(p.slice(0,2),color,original?3:6);if(!original)dot(arrowTip(p),color,4);
 ctx.fillStyle=color;const q=screen(p);ctx.font='bold 13px sans-serif';ctx.fillText(name+(original?' original':''),q[0]+8,q[1]-9);
 if(!original&&name===selection){ctx.strokeStyle='white';ctx.beginPath();ctx.arc(...q,10,0,Math.PI*2);ctx.stroke();}
}
function draw(){
 if(!data)return;const w=canvas.clientWidth,h=canvas.clientHeight,dpr=window.devicePixelRatio||1;
 if(canvas.width!==w*dpr||canvas.height!==h*dpr){canvas.width=w*dpr;canvas.height=h*dpr;}
 ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);ctx.imageSmoothingEnabled=false;
 ctx.drawImage(image,...offset,data.map.width*zoom,data.map.height*zoom);
 if($('costmaps').checked){
  for(const [name,meta] of Object.entries(data.costmaps)){
   const a=screen(pixelToMap(meta,-.5,-.5)),b=screen(pixelToMap(meta,meta.width-.5,-.5)),c=screen(pixelToMap(meta,-.5,meta.height-.5));
   ctx.save();ctx.transform((b[0]-a[0])/meta.width,(b[1]-a[1])/meta.width,(c[0]-a[0])/meta.height,(c[1]-a[1])/meta.height,...a);ctx.drawImage(costmapImages[name],0,0);ctx.restore();
  }
 }
 if($('grid').checked){
  const corners=[[0,0],[w,0],[w,h],[0,h]].map(world),xs=corners.map(p=>p[0]),ys=corners.map(p=>p[1]);
  const spacing=zoom/data.map.resolution<12?5:1;
  const x0=Math.floor(Math.min(...xs)/spacing)*spacing,x1=Math.max(...xs),y0=Math.floor(Math.min(...ys)/spacing)*spacing,y1=Math.max(...ys);
  for(let x=x0;x<=x1;x+=spacing)line([[x,y0],[x,y1]],'#3e7c9155');
  for(let y=y0;y<=y1;y+=spacing)line([[x0,y],[x1,y]],'#3e7c9155');
 }
 state.polygons.forEach((p,i)=>{line(p.points,p.kind==='restricted'?'#ff5363':'#c995ff',i===polygonSelection?3:2,true);p.points.forEach(q=>dot(q,i===polygonSelection?'white':'#bd8eee',4));});
 line(draft,'#ffff80',2);draft.forEach(p=>dot(p,'yellow',4));
 if($('sweep').checked&&pose(selection)){
  const p=pose(selection);for(let i=0;i<=72;i++){
   const a=p[2]+i*Math.PI*2/72;const result=sweepResults?.samples[i];
   line(transform(data.footprint,[...p.slice(0,2),a]),result?(result.clear?'#3ee6a455':'#ff455555'):'#e0e0e022',1,true);
  }
 }
 data.initial.goals.forEach(g=>drawPose(g.name,[...g.position,g.yaw_rad],'#ffb84d',true));
 state.goals.forEach(g=>drawPose(g.name,[...g.position,g.yaw_rad],'#00e4ee'));
 Object.entries(state.markers).forEach(([name,p])=>drawPose(name,p,'#ff8fff'));
}
function refresh(){
 const names=state.goals.map(g=>g.name).concat(Object.keys(state.markers));if(!names.includes(selection))selection='G1';
 $('selected').replaceChildren(...names.map(name=>{const o=document.createElement('option');o.value=name;o.textContent=name;return o;}));$('selected').value=selection;
 const p=pose(selection);['x','y','yaw'].forEach((id,i)=>$(id).value=p[i]);
 $('values').textContent=JSON.stringify({original:data.initial.goals,candidate:state.goals,markers:state.markers},null,2);
 $('undo').disabled=history.index===0;$('redo').disabled=history.index===history.states.length-1;draw();
}
function changed(){history.push(state);sweepResults=null;$('checks').textContent='Changed — run geometry checks again';refresh();}
async function api(path,body){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Editor-Token':data.token},body:JSON.stringify(body)});const result=await r.json();if(!r.ok)throw Error(result.error||r.statusText);return result;}
async function check(){const snapshot=JSON.stringify(state),selected=selection;status('Checking geometry…');const result=await api('/check',{state,selected,sweep:$('sweep').checked});if(snapshot!==JSON.stringify(state)||selected!==selection)return; $('checks').textContent=JSON.stringify(result,null,2);sweepResults=result.sweep;status('Checks use filled configured footprint; sweeps sample every 5°. Physical feasibility remains unproven.');draw();}
function run(action){return async()=>{try{await action();}catch(e){status(e.message);}};}
$('undo').onclick=()=>{state=history.undo();sweepResults=null;refresh();$('checks').textContent='History changed — recheck';};
$('redo').onclick=()=>{state=history.redo();sweepResults=null;refresh();$('checks').textContent='History changed — recheck';};
$('reset').onclick=()=>{state=history.reset();draft=[];polygonSelection=null;sweepResults=null;refresh();$('checks').textContent='Reset — recheck';};
function fit(){zoom=Math.min(canvas.clientWidth/data.map.width,canvas.clientHeight/data.map.height)*.94;offset=[(canvas.clientWidth-data.map.width*zoom)/2,(canvas.clientHeight-data.map.height*zoom)/2];draw();}
$('fit').onclick=fit;$('selected').onchange=()=>{selection=$('selected').value;polygonSelection=null;sweepResults=null;refresh();};
$('grid').onchange=draw;$('costmaps').onchange=draw;$('sweep').onchange=()=>{sweepResults=null;draw();};
$('mode').onchange=()=>{draft=[];draw();};
$('apply').onclick=()=>{const p=['x','y','yaw'].map(id=>Number($(id).value));if(p.some(v=>!Number.isFinite(v)||Math.abs(v)>1e6)||['x','y','yaw'].some(id=>$(id).value.trim()===''))return status('Enter finite x, y, yaw');setPose(selection,p);changed();};
$('finish').onclick=run(async()=>{if(draft.length<3)throw Error('Need at least three vertices');const next=clone(state);next.polygons.push({kind:$('mode').value,points:draft});await api('/validate',{state:next});state=next;polygonSelection=state.polygons.length-1;draft=[];changed();});
$('cancel').onclick=()=>{draft=[];draw();};
$('delete').onclick=()=>{if(polygonSelection!==null){state.polygons.splice(polygonSelection,1);polygonSelection=null;}else if(selection in state.markers)delete state.markers[selection];changed();};
$('check').onclick=run(check);
$('export').onclick=run(async()=>{const result=await api('/export',{state});status(`Exported to ${result.path}. ${result.warning}`);});
canvas.oncontextmenu=e=>e.preventDefault();
canvas.onwheel=e=>{e.preventDefault();const p=local(e),factor=Math.exp(-e.deltaY*.001),next=Math.max(.1,Math.min(50,zoom*factor));offset=p.map((v,i)=>v-(v-offset[i])*next/zoom);zoom=next;draw();};
canvas.onpointerdown=e=>{
 const p=local(e),xy=world(p),mode=$('mode').value;
 if(e.button===1||e.button===2||mode==='pan'){drag={kind:'pan',start:p,offset:[...offset]};canvas.setPointerCapture(e.pointerId);return;}
 if(mode==='corridor'||mode==='restricted'){draft.push(xy);draw();return;}
 if(mode==='entrance'||mode==='exit'){state.markers[mode]=[...xy,0];selection=mode;polygonSelection=null;changed();return;}
 polygonSelection=null;
 for(const name of [...state.goals.map(g=>g.name),...Object.keys(state.markers)]){
  const q=pose(name);
  if(distance(p,screen(arrowTip(q)))<12){selection=name;drag={kind:'yaw',before:clone(state)};break;}
  if(distance(p,screen(q))<12){selection=name;drag={kind:'pose',before:clone(state)};break;}
 }
 if(!drag)for(let i=state.polygons.length-1;i>=0;i--){const j=state.polygons[i].points.findIndex(q=>distance(p,screen(q))<12);if(j>=0){polygonSelection=i;drag={kind:'vertex',i,j,before:clone(state)};break;}}
 if(!drag)drag={kind:'pan',start:p,offset:[...offset]};
 canvas.setPointerCapture(e.pointerId);refresh();
};
canvas.onpointermove=e=>{
 const p=local(e),xy=world(p),uv=mapToPixel(data.map,...xy);
 $('cursor').textContent=`map x=${xy[0].toFixed(4)} y=${xy[1].toFixed(4)} m | image u=${uv[0].toFixed(3)} v=${uv[1].toFixed(3)} | cell ${Math.floor(uv[0]+.5)}, ${Math.floor(data.map.height-uv[1]-.5)}`;
 if(!drag)return;
 if(drag.kind==='pan'){offset=p.map((v,i)=>drag.offset[i]+v-drag.start[i]);draw();return;}
 const q=pose(selection);
 if(drag.kind==='pose')setPose(selection,[...xy,q[2]]);
 if(drag.kind==='yaw')setPose(selection,[...q.slice(0,2),Math.atan2(xy[1]-q[1],xy[0]-q[0])]);
 if(drag.kind==='vertex')state.polygons[drag.i].points=moveVertex(state.polygons[drag.i].points,drag.j,xy);
 sweepResults=null;refresh();
};
canvas.onpointerup=run(async()=>{
 if(!drag)return;const d=drag;drag=null;if(d.kind==='pan')return;
 try{await api('/validate',{state});changed();}catch(e){state=d.before;refresh();throw e;}
});
canvas.onpointercancel=()=>{if(drag?.before)state=drag.before;drag=null;refresh();};
window.addEventListener('keydown',e=>{if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;if(e.key==='Escape'){draft=[];if(drag?.before)state=drag.before;drag=null;refresh();}if((e.ctrlKey||e.metaKey)&&e.key==='z'){e.preventDefault();$(e.shiftKey?'redo':'undo').click();}});
window.addEventListener('resize',draw);
try{
 data=await (await fetch('/bootstrap')).json();state=clone(data.initial);history=new History(state);image=new Image();image.src='/map.png';await image.decode();
 for(const name of Object.keys(data.costmaps)){const im=new Image();im.src=`/${name}-costmap.png`;await im.decode();costmapImages[name]=im;}
 $('identity').textContent=JSON.stringify({map:data.identity,occupancy_policy:data.occupancy_policy,sources:data.sources,padded_footprint:data.footprint},null,2);refresh();fit();
 status('Ready. Gray 205 is conservatively blocked as unknown. Original positions are shown in this saved map; cross-session registration is unverified.');
}catch(e){status(e.message);}
