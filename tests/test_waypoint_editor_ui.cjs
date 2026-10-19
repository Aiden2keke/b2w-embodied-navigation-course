const assert = require('node:assert/strict');
(async()=>{
 const {pixelToMap,mapToPixel,History,moveVertex}=await import('../scripts/waypoint_editor/model.mjs');
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
 console.log('UI model: transforms, history isolation, branching, undoable reset, polygon vertex edit PASS');
})();
