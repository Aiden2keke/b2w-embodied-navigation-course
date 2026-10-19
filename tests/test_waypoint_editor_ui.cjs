const assert = require('node:assert/strict');
(async()=>{
 const {pixelToMap,mapToPixel,History,moveVertex,toleranceCircle}=await import('../scripts/waypoint_editor/model.mjs');
 const m={width:800,height:800,resolution:.05,origin:[-20,-20,Math.PI/2]};
 assert.deepEqual(pixelToMap({...m,origin:[-20,-20,0]},0,0),[-19.975,19.975]);
 for(const p of [[0,0],[799,799],[42.123,315.678]]){
  const q=mapToPixel(m,...pixelToMap(m,...p));q.forEach((v,i)=>assert.ok(Math.abs(v-p[i])<1e-10));
 }
 const initial={goals:[{position:[1,2],yaw_rad:0}],polygons:[],markers:{}};
 const h=new History(initial); let s=h.current();s.goals[0].yaw_rad=1.25;h.push(s);
 assert.equal(h.undo().goals[0].yaw_rad,0);assert.equal(h.redo().goals[0].yaw_rad,1.25);
 h.undo();s=h.current();s.markers.entrance=[1,2,0];h.push(s);assert.equal(h.redo().markers.entrance[0],1);
 h.reset();assert.deepEqual(h.current(),initial);assert.deepEqual(h.undo().markers.entrance,[1,2,0]);
 assert.deepEqual(moveVertex([[0,0],[1,0],[1,1]],1,[2,0]),[[0,0],[2,0],[1,1]]);
 for(const yaw of [0,.3,Math.PI/2])for(const z of [.1,1,3.7,50])for(const offset of [[0,0],[71,-29]]){
  const meta={...m,origin:[-20,-20,yaw]};
  for(const xy of [[1.123,2.456],[-4.1,6.2]]){
   const circle=toleranceCircle(meta,xy,z,offset);
   assert.equal(circle.radiusPx,5*z);
   const center=mapToPixel(meta,...xy).map((v,i)=>(v+.5)*z+offset[i]);
   circle.center.forEach((v,i)=>assert.ok(Math.abs(v-center[i])<1e-10));
   for(const [dx,dy] of [[circle.radiusPx,0],[0,circle.radiusPx]]){
    const world=pixelToMap(meta,(circle.center[0]+dx-offset[0])/z-.5,(circle.center[1]+dy-offset[1])/z-.5);
    assert.ok(Math.abs(Math.hypot(world[0]-xy[0],world[1]-xy[1])-.25)<1e-11);
   }
  }
 }
 console.log('UI model: transforms, history isolation, branching, undoable reset, polygon vertex edit PASS');
})();
