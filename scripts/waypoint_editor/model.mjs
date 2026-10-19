export function pixelToMap(m,u,v){
 const [ox,oy,a]=m.origin,x=(u+.5)*m.resolution,y=(m.height-v-.5)*m.resolution;
 return [ox+Math.cos(a)*x-Math.sin(a)*y,oy+Math.sin(a)*x+Math.cos(a)*y];
}
export function mapToPixel(m,x,y){
 const [ox,oy,a]=m.origin; x-=ox;y-=oy;
 return [(Math.cos(a)*x+Math.sin(a)*y)/m.resolution-.5,m.height-(-Math.sin(a)*x+Math.cos(a)*y)/m.resolution-.5];
}
export function transform(points,[x,y,a]){return points.map(([u,v])=>[x+Math.cos(a)*u-Math.sin(a)*v,y+Math.sin(a)*u+Math.cos(a)*v]);}
export const clone=x=>JSON.parse(JSON.stringify(x));
export function moveVertex(points,index,xy){const p=clone(points);p[index]=[...xy];return p;}
export class History{
 constructor(initial){this.initial=clone(initial);this.states=[clone(initial)];this.index=0;}
 current(){return clone(this.states[this.index]);}
 push(state){if(JSON.stringify(state)===JSON.stringify(this.states[this.index]))return;this.states.splice(this.index+1);this.states.push(clone(state));this.index++;}
 undo(){this.index=Math.max(0,this.index-1);return this.current();}
 redo(){this.index=Math.min(this.states.length-1,this.index+1);return this.current();}
 reset(){this.push(this.initial);return this.current();}
}

// A planar map origin is a rigid rotation: Euclidean circles stay circles.
export function toleranceCircle(meta,xy,zoom,offset){
 return {center:mapToPixel(meta,...xy).map((v,i)=>(v+.5)*zoom+offset[i]),radiusPx:.25/meta.resolution*zoom};
}
