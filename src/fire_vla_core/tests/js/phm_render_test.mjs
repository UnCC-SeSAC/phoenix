// index.html 의 PHM 렌더링을 최소 DOM 셰임 위에서 **실제로 실행**합니다.
//
// 왜 이런 걸 두나
//   PHM 패널이 지켜야 할 것 세 가지는 마크업만 봐서는 확인이 안 됩니다.
//     1) 경보 이름을 바꾸지 않는다 — 이 검출기는 들림만 잡습니다(LIFT_SUSPECTED).
//     2) not_detected 를 드러낸다 — 경보가 **없을 때** 특히 필요합니다.
//        안 그러면 'ALL CLEAR' 처럼 읽혀서 안 잡는 고장(슬립)까지 없다고 말합니다.
//     3) stale / 축별 fresh 를 흐리게 표시한다 — 끊긴 값이 정상으로 보이면 안 됩니다.
//   전부 조건부 렌더링이라 실제로 돌려봐야 압니다.
//
//   실행:  node tests/js/phm_render_test.mjs fire_vla_core/web/index.html
//   pytest 에서도 부릅니다 (tests/test_firefighter_ui_phm.py, node 없으면 skip).
import fs from 'node:fs';

const html = fs.readFileSync(process.argv[2], 'utf8');
const js = html.slice(html.indexOf('<script>') + 8, html.lastIndexOf('</script>'));

class El {
  constructor(tag){ this.tag=tag; this.children=[]; this.className=''; this._text='';
                    this.style=new Proxy({},{set:(t,k,v)=>{t[k]=v;return true;}}); this.hidden=false; }
  append(...kids){ this.children.push(...kids); }
  replaceChildren(...kids){ this.children=kids; }
  set textContent(v){ this._text=String(v); this.children=[]; }
  get textContent(){ return this._text + this.children.map(c=>c.textContent).join(' '); }
  get childElementCount(){ return this.children.length; }
  setAttribute(){}
  addEventListener(){}
  querySelector(){ return null; }
  querySelectorAll(){ return []; }
}
const els = {};
const ID = ['phmHealth','phmAge','phmAxes','phmBlocked','phmLimit','phmBattery','phmCpu','phmTemp','phmCmd',
            'connection','updatedAt','modeSelector','missionForm','ruleControls','missionResult','currentMission',
            'robotState','robotPose','actionName','actionTarget','actionLabel','decisionLabel','decisionTitle',
            'decisionReason','validation','submission','resultAction','resultStatus','resultDetail','timeline',
            'objects','visionToggle','visionStream','visionBoxes','visionFallback','visionStat','visionFrame',
            'slamMap','semanticMap','mapEmpty','mapMode','missionInput'];
for (const id of ID) els[id] = new El('div');
globalThis.document = {
  getElementById: id => els[id] || (els[id] = new El('div')),
  createElement: t => new El(t),
  createElementNS: (_ns,t) => new El(t),
  querySelector: () => null,
  querySelectorAll: () => [],
  addEventListener: () => {},
};
globalThis.window = { addEventListener(){}, location:{} };
globalThis.setInterval = () => 0;
globalThis.setTimeout = () => 0;
globalThis.Image = class { set src(_v){} addEventListener(){} };
globalThis.URL = { createObjectURL:()=> '', revokeObjectURL(){} };
globalThis.fetch = async () => { throw new Error('no network in test'); };

const ctx = {};
const fn = new Function(js + '\nreturn {renderPhm, PHM_HEALTH};');
const { renderPhm } = fn();

let failed = 0;
const check = (name, cond, detail='') => {
  console.log(`${cond ? '  통과' : '  ★실패'}  ${name}${cond ? '' : '  ' + detail}`);
  if (!cond) failed++;
};

const base = (over={}) => ({
  schema_version:1, mode:'PHM', health:'OK', alarms:[], not_detected:['SLIP'],
  available:true, stale:false, age_sec:0.4, battery_mv:7826,
  host:{cpu_used_pct:14.0, thermal_c:{'thermal_zone0:cpu-thermal':52.6}},
  cmd_source:'/controller/cmd_vel', blocked_reason:null,
  rules:{yaw:{thr:0.35,frac:0.9167}, fwd:{thr:0.15,frac:0.8333}},
  axes:{
    yaw:{residual:0.088, threshold:0.35, ratio:0.10, alarm:false, evaluated:476, fresh:true, age_sec:0.02,
         unit:'rad/s', label:'요레이트', meas:'자이로'},
    fwd:{residual:0.016, threshold:0.15, ratio:0.05, alarm:false, evaluated:462, fresh:true, age_sec:0.10,
         unit:'m/s', label:'전진속도', meas:'rf2o'},
  }, ...over});

console.log('[1] 정상');
renderPhm(base());
check('health 배지 NOMINAL', els.phmHealth.textContent==='NOMINAL', els.phmHealth.textContent);
check('배지 색 ok', els.phmHealth.className.includes('ok'), els.phmHealth.className);
check('축 카드 2개', els.phmAxes.childElementCount===2);
check('★ 경보 없어도 not_detected 노출', !els.phmLimit.hidden && els.phmLimit.textContent.includes('SLIP'));
check('배터리 V 변환', els.phmBattery.textContent==='battery 7.83 V', els.phmBattery.textContent);
check('온도 표시', els.phmTemp.textContent==='temp 52.6°C', els.phmTemp.textContent);

console.log('[2] 경보');
renderPhm(base({health:'ALARM',
  alarms:[{name:'LIFT_SUSPECTED',axis:'fwd',residual:0.187,threshold:0.15}],
  axes:{...base().axes, fwd:{...base().axes.fwd, residual:0.187, ratio:1.0, alarm:true}}}));
check('health 배지 ALARM', els.phmHealth.textContent==='ALARM');
check('배지 색 bad', els.phmHealth.className.includes('bad'));
const fwdCard = els.phmAxes.children.find(c=>c.textContent.includes('전진속도'));
check('경보 축 카드에 alarm 클래스', fwdCard && fwdCard.className.includes('alarm'), fwdCard && fwdCard.className);
const yawCard = els.phmAxes.children.find(c=>c.textContent.includes('요레이트'));
check('정상 축은 alarm 아님', yawCard && !yawCard.className.includes('alarm'));

console.log('[3] stale — 끊긴 값을 정상으로 보이면 안 됨');
renderPhm(base({health:'UNKNOWN', stale:true, age_sec:9.3,
  blocked_reason:'PHM status가 9.3초째 갱신되지 않았습니다.',
  axes:{...base().axes, yaw:{...base().axes.yaw, fresh:false, age_sec:9.3}}}));
check('health UNKNOWN', els.phmHealth.textContent==='UNKNOWN');
check('blocked 사유 표시', !els.phmBlocked.hidden && els.phmBlocked.textContent.includes('9.3초'));
const staleCard = els.phmAxes.children.find(c=>c.textContent.includes('요레이트'));
check('★ 끊긴 축 흐리게(dim)', staleCard && staleCard.className.includes('dim'), staleCard && staleCard.className);
check('끊긴 축에 경과 표기', staleCard && staleCard.textContent.includes('갱신 없음'));

console.log('[4] phm_monitor 미기동');
renderPhm({schema_version:1,mode:'PHM',available:false,health:'UNKNOWN',alarms:[],
           blocked_reason:'PHM status를 기다리는 중입니다. phm_monitor 노드가 떠 있는지 확인하세요.'});
check('축 없음 안내', els.phmAxes.textContent.includes('축 데이터 없음'));
check('age 표시 안 함', els.phmAge.textContent==='-', els.phmAge.textContent);
check('not_detected 없으면 숨김', els.phmLimit.hidden);

console.log(failed ? `\n실패 ${failed}건` : '\n전부 통과');
process.exit(failed ? 1 : 0);
