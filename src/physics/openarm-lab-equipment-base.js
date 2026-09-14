// Bounded dry rigid-body assets for the standalone OpenArm Lab Builder.
// No arbitrary XML, URLs, scripts, plugins or state setters are accepted.
export const OPENARM_LAB_EQUIPMENT_VERSION = 'robobuddy.openarm.lab-equipment.v1';
export const LAB_EQUIPMENT_KINDS = Object.freeze(['bench', 'tray', 'receiver', 'rack', 'vial', 'block', 'obstacle', 'assembly']);
export const LAB_EQUIPMENT_LIMITS = Object.freeze({ items: 12, partsPerAssembly: 12, geometries: 128, inputBytes: 24000 });
const DEFAULTS = { bench: [.75,.55,.06], tray: [.14,.10,.025], receiver: [.12,.10,.04], rack: [.12,.09,.04], vial: [.025,.025,.055], block: [.025,.025,.025], obstacle: [.08,.08,.12], assembly: [.12,.10,.05] };
const ID = /^[a-z][a-z0-9_]{0,23}$/;
const plain = (v, label) => { if (!v || typeof v !== 'object' || Array.isArray(v) || ![Object.prototype, null].includes(Object.getPrototypeOf(v))) throw new TypeError(`${label} must be a plain object`); };
const only = (v, keys, label) => { for (const key of Object.keys(v)) if (!keys.includes(key)) throw new TypeError(`Unknown ${label} field ${key}`); };
const number = (v, lo, hi, label) => { if (typeof v !== 'number' || !Number.isFinite(v) || v < lo || v > hi) throw new RangeError(`${label} must be ${lo}..${hi}`); return v; };
const vec = (v, n, label) => { if (!Array.isArray(v) || v.length !== n) throw new TypeError(`${label} must contain ${n} numbers`); return v.map((x,i) => number(x, -10, 10, `${label}[${i}]`)); };
const yawQuat = a => [Math.cos(a/2),0,0,Math.sin(a/2)];
const join = a => a.join(' ');

export function validateOpenArmLabEquipment(items) {
  if (!Array.isArray(items) || items.length > LAB_EQUIPMENT_LIMITS.items) throw new RangeError(`equipment must contain 0..${LAB_EQUIPMENT_LIMITS.items} items`);
  if (new TextEncoder().encode(JSON.stringify(items)).length > LAB_EQUIPMENT_LIMITS.inputBytes) throw new RangeError('Equipment input exceeds byte budget');
  const ids = new Set();
  return items.map((raw,index) => {
    const label = `equipment[${index}]`; plain(raw,label);
    only(raw,['id','label','kind','position_m','yaw_rad','dimensions_m','mass_kg','dynamic','parts'],label);
    if (typeof raw.id !== 'string' || !ID.test(raw.id) || ids.has(raw.id)) throw new TypeError(`${label}.id must be a unique lowercase identifier`);
    ids.add(raw.id);
    if (!LAB_EQUIPMENT_KINDS.includes(raw.kind)) throw new TypeError(`${label}.kind is unsupported in Lab Builder`);
    if (raw.label !== undefined && (typeof raw.label !== 'string' || raw.label.length > 64 || /[\x00-\x1f]/.test(raw.label))) throw new TypeError(`${label}.label must be printable text up to 64 characters`);
    const dimensions = vec(raw.dimensions_m ?? DEFAULTS[raw.kind],3,`${label}.dimensions_m`);
    const max = raw.kind === 'bench' ? [1.5,1.5,1.2] : [.6,.6,.6];
    dimensions.forEach((value,axis) => number(value, raw.kind === 'bench' && axis < 2 ? .2 : .008, max[axis], `${label}.dimensions_m[${axis}]`));
    if (raw.kind === 'vial' && Math.abs(dimensions[0]-dimensions[1]) > 1e-9) throw new RangeError('A cylindrical vial requires equal X/Y diameters');
    const position = vec(raw.position_m,3,`${label}.position_m`);
    const yaw = number(raw.yaw_rad ?? 0,-Math.PI,Math.PI,`${label}.yaw_rad`);
    const hx = (Math.abs(Math.cos(yaw))*dimensions[0]+Math.abs(Math.sin(yaw))*dimensions[1])/2;
    const hy = (Math.abs(Math.sin(yaw))*dimensions[0]+Math.abs(Math.cos(yaw))*dimensions[1])/2;
    if (position[0]-hx < -.5 || position[0]+hx > 1.5 || position[1]-hy < -1 || position[1]+hy > 1 || position[2] < 0 || position[2]+dimensions[2] > 1.8) throw new RangeError('Asset exceeds the bounded 2 m x 2 m x 1.8 m Lab Builder volume');
    if (raw.dynamic !== undefined && typeof raw.dynamic !== 'boolean') throw new TypeError(`${label}.dynamic must be boolean`);
    const dynamic = raw.dynamic ?? ['vial','block'].includes(raw.kind);
    if (['bench','receiver','obstacle','tray','rack'].includes(raw.kind) && dynamic) throw new RangeError(`${raw.kind} is fixed in the current capability; loose manipulation supports vial and block`);
    const mass = number(raw.mass_kg ?? (dynamic ? .05 : raw.kind === 'bench' ? 20 : .2),.005,raw.kind === 'bench' ? 100 : 2,`${label}.mass_kg`);
    let parts;
    if (raw.kind === 'assembly') {
      if (!Array.isArray(raw.parts) || raw.parts.length < 1 || raw.parts.length > LAB_EQUIPMENT_LIMITS.partsPerAssembly) throw new RangeError('assembly requires 1..12 parts');
      parts = raw.parts.map((part,i) => {
        plain(part,`part[${i}]`); only(part,['shape','position_m','dimensions_m','radius_m','height_m'],`part[${i}]`);
        if (!['box','cylinder','sphere'].includes(part.shape)) throw new TypeError('Assembly shape must be box, cylinder or sphere');
        const p = vec(part.position_m,3,`part[${i}].position_m`);
        let half;
        if (part.shape === 'box') {
          const dim = vec(part.dimensions_m,3,`part[${i}].dimensions_m`); dim.forEach((x,j)=>number(x,.002,.6,`part dimension ${j}`)); half=dim.map(x=>x/2);
          if (Math.abs(p[0])+half[0] > dimensions[0]/2 || Math.abs(p[1])+half[1] > dimensions[1]/2 || p[2]-half[2] < 0 || p[2]+half[2] > dimensions[2]) throw new RangeError(`Part ${i} exceeds assembly envelope`);
          return { shape:'box', position_m:p, dimensions_m:dim };
        }
        const radius = number(part.radius_m,.001,.3,`part[${i}].radius_m`);
        const height = part.shape === 'cylinder' ? number(part.height_m,.002,.6,`part[${i}].height_m`) : radius*2;
        half=[radius,radius,height/2];
        if (Math.abs(p[0])+half[0] > dimensions[0]/2 || Math.abs(p[1])+half[1] > dimensions[1]/2 || p[2]-half[2] < 0 || p[2]+half[2] > dimensions[2]) throw new RangeError(`Part ${i} exceeds assembly envelope`);
        return { shape:part.shape, position_m:p, radius_m:radius, ...(part.shape === 'cylinder' ? {height_m:height}: {}) };
      });
    } else if (raw.parts !== undefined) throw new TypeError('Only assembly accepts parts');
    return { id:raw.id, label:raw.label ?? raw.id, kind:raw.kind, position_m:position, yaw_rad:yaw, dimensions_m:dimensions, mass_kg:mass, dynamic, ...(parts ? {parts}: {}) };
  });
}

const volume = part => part.type === 'box' ? 8*part.sizeM[0]*part.sizeM[1]*part.sizeM[2] : part.type === 'cylinder' ? Math.PI*part.sizeM[0]**2*(2*part.sizeM[1]) : 4/3*Math.PI*part.sizeM[0]**3;
export function compileOpenArmLabEquipment(equipment) {
  const geoms=[], bodies=[], records=[]; let xml='';
  for (const e of equipment) {
    const bodyId=`lab_${e.id}`, [w,d,h]=e.dimensions_m, parts=[];
    const add=(type,positionM,sizeM,suffix,rgba)=>parts.push({id:`${bodyId}_${suffix}`,bodyId,type,positionM,quaternionWxyz:[1,0,0,0],sizeM,rgba:rgba ?? (e.dynamic ? [.25,.65,.77,1]:[.35,.46,.54,1])});
    if (['bench','block','obstacle'].includes(e.kind)) add('box',[0,0,h/2],[w/2,d/2,h/2],'solid',e.kind==='bench'?[.58,.55,.49,1]:null);
    else if (e.kind==='vial') add('cylinder',[0,0,h/2],[w/2,h/2],'solid');
    else if (e.kind==='assembly') for (const [i,p] of e.parts.entries()) add(p.shape,p.position_m,p.shape==='box'?p.dimensions_m.map(x=>x/2):p.shape==='sphere'?[p.radius_m]:[p.radius_m,p.height_m/2],`part${i}`);
    else {
      const t=Math.min(.003,w/6,d/6,h/4); add('box',[0,0,t/2],[w/2,d/2,t/2],'base');
      for (const sign of [-1,1]) { add('box',[sign*(w-t)/2,0,h/2],[t/2,d/2,h/2],`wall_x${sign>0?'p':'n'}`); add('box',[0,sign*(d-t)/2,h/2],[w/2,t/2,h/2],`wall_y${sign>0?'p':'n'}`); }
      if (e.kind==='rack') { for (const x of [-w/6,w/6]) add('box',[x,0,h/2],[t/2,d/2,h/2],`divider_${x<0?'a':'b'}`); }
    }
    const weights=parts.map(volume), total=weights.reduce((a,b)=>a+b,0); if (!(total>0)) throw new Error(`Asset ${e.id} has no physical volume`);
    bodies.push({id:bodyId,...(e.dynamic?{freeJointId:`${bodyId}_free`}:{})});
    const geomXml=(p,i)=>`<geom name="${p.id}" type="${p.type}" pos="${join(p.positionM)}" size="${join(p.sizeM)}" mass="${e.mass_kg*weights[i]/total}" contype="3" conaffinity="3" friction="0.8 0.005 0.0005" solref="0.005 1" rgba="${join(p.rgba)}"/>`;
    xml += `<body name="${bodyId}" pos="${join(e.position_m)}" quat="${join(yawQuat(e.yaw_rad))}">${e.dynamic?`<freejoint name="${bodyId}_free"/>`:''}${parts.map(geomXml).join('')}</body>`;
    let affordances={graspReferenceM:[0,0,h/2],topReferenceM:[0,0,h]};
    if (['receiver','tray','rack'].includes(e.kind)) { const t=Math.min(.003,w/6,d/6,h/4); affordances={receivingBottomM:[0,0,t],receivingHalfExtentsM:[Math.max(.001,w/2-t),Math.max(.001,d/2-t)],openingTopM:[0,0,h],supportGeometryId:`${bodyId}_base`}; }
    if (e.kind==='bench') affordances={topReferenceM:[0,0,h],supportGeometryId:`${bodyId}_solid`,boundaryCondition:'world-fixed'};
    geoms.push(...parts); records.push({...e,bodyId,geometryIds:parts.map(p=>p.id),affordances,processModel:e.kind==='bench'?'world-fixed rigid support boundary condition':'dry rigid-body geometry only'});
  }
  if (geoms.length > LAB_EQUIPMENT_LIMITS.geometries) throw new RangeError('Equipment exceeds collision geometry budget');
  return {xml,geoms,bodies,records,passiveJoints:[]};
}
export function appendOpenArmLabEquipmentXml(baseXml,equipment) {
  const compiled=compileOpenArmLabEquipment(equipment);
  if ((baseXml.match(/<\/worldbody>/g)||[]).length!==1) throw new Error('Unexpected OpenArm worldbody structure');
  return {...compiled,xml:baseXml.replace('</worldbody>',`${compiled.xml}</worldbody>`)};
}
